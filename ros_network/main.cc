#include "geometry_msgs/msg/twist.hpp"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/float32_array.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <mutex>
#include <sstream>
#include <thread>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LteRos2RealtimeExample");

class Ns3RosNode;
void ReportUeMeasurementsCallback(std::string, uint16_t, uint16_t, double,
                                  double, bool, uint8_t);

class Ns3RosNode : public rclcpp::Node {
public:
  Ns3RosNode() : Node("ns3_ros_node") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/X3/cmd_vel", 10);
    pose_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/model/X3/pose", 10,
        std::bind(&Ns3RosNode::poseCallback, this, std::placeholders::_1));
    rsrp_publisher_ =
        this->create_publisher<ros_gz_interfaces::msg::Float32Array>(
            "/rsrp_values", 10);

    rsrp_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Ns3RosNode::publishAllRsrpValues, this));
  }

  void publishTwistFromPacket(const std::string &data) {
    auto twist_msg = geometry_msgs::msg::Twist();
    std::istringstream iss(data);
    std::string token;

    while (std::getline(iss, token, ',')) {
      size_t pos = token.find(':');
      if (pos != std::string::npos) {
        std::string key = token.substr(0, pos);
        std::string value = token.substr(pos + 1);
        double val = std::stod(value);

        if (key == "linear.x")
          twist_msg.linear.x = val;
        else if (key == "linear.y")
          twist_msg.linear.y = val;
        else if (key == "linear.z")
          twist_msg.linear.z = val;
        else if (key == "angular.x")
          twist_msg.angular.x = val;
        else if (key == "angular.y")
          twist_msg.angular.y = val;
        else if (key == "angular.z")
          twist_msg.angular.z = val;
      }
    }
    publisher_->publish(twist_msg);
  }

  void sendTwistOverUdp(const geometry_msgs::msg::Twist &twist) {
    std::lock_guard<std::mutex> lock(node_mutex_);
    if (udp_socket_) {
      std::ostringstream oss;
      oss << "linear.x:" << twist.linear.x << ",linear.y:" << twist.linear.y
          << ",linear.z:" << twist.linear.z << ",angular.x:" << twist.angular.x
          << ",angular.y:" << twist.angular.y
          << ",angular.z:" << twist.angular.z;
      std::string data = oss.str();

      Ptr<Packet> packet = Create<Packet>((uint8_t *)data.c_str(), data.size());
      Ptr<Socket> sock = udp_socket_;
      Ipv4Address addr = ue_ip_addr_;
      Simulator::ScheduleNow([sock, packet, addr]() {
        if (sock)
          sock->SendTo(packet, 0, InetSocketAddress(addr, 1234));
      });
    }
  }

  void setUeNode(ns3::Ptr<ns3::Node> node) {
    std::lock_guard<std::mutex> lock(node_mutex_);
    ueNode = node;
  }

  void setLteHelper(ns3::Ptr<ns3::LteHelper> helper) {
    std::lock_guard<std::mutex> lock(node_mutex_);
    lteHelper = helper;
  }

  void setUeDevice(ns3::NetDeviceContainer devices) {
    std::lock_guard<std::mutex> lock(node_mutex_);
    ueLteDevs = devices;
  }

  void setEnbDevices(ns3::NetDeviceContainer devices) {
    std::lock_guard<std::mutex> lock(node_mutex_);
    enbLteDevs = devices;
  }

  void setUdpSocket(Ptr<Socket> socket, Ipv4Address ue_ip) {
    std::lock_guard<std::mutex> lock(node_mutex_);
    udp_socket_ = socket;
    ue_ip_addr_ = ue_ip;
  }

  void createCmdVelSubscription() {
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/model_movement_commands", 10,
        std::bind(&Ns3RosNode::cmdVelCallback, this, std::placeholders::_1));
  }

  void recordRsrpValue(uint16_t cellId, double rsrp) {
    std::lock_guard<std::mutex> lock(rsrp_mutex_);
    rsrp_values_[cellId] = static_cast<float>(rsrp);
  }

  void publishAllRsrpValues() {
    std::lock_guard<std::mutex> lock(rsrp_mutex_);
    auto rsrp_msg = ros_gz_interfaces::msg::Float32Array();
    for (const auto &pair : rsrp_values_) {
      rsrp_msg.data.push_back(pair.second);
    }
    if (!rsrp_msg.data.empty()) {
      rsrp_publisher_->publish(rsrp_msg);
    }
  }

  void connectUePhyTraces() {
    std::lock_guard<std::mutex> lock(node_mutex_);
    if (ueLteDevs.GetN() == 0)
      return;

    Ptr<LteUeNetDevice> ueDevice =
        DynamicCast<LteUeNetDevice>(ueLteDevs.Get(0));
    if (!ueDevice)
      return;

    Ptr<LteUePhy> uePhy = ueDevice->GetPhy();
    if (!uePhy)
      return;

    uePhy->TraceConnect("ReportUeMeasurements", std::string(""),
                        MakeCallback(&ReportUeMeasurementsCallback));
    NS_LOG_UNCOND("RSRP measurement trace connected directly to UE PHY");
  }

private:
  void poseCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    for (const auto &transform : msg->transforms) {
      if (transform.child_frame_id.find("X3") != std::string::npos) {
        scheduleUeMove(transform.transform.translation.x,
                       transform.transform.translation.y,
                       transform.transform.translation.z);
      }
    }
  }

  void scheduleUeMove(double x, double y, double z) {
    Vector3D newPos{1.0 * x, 1.0 * y, 1.0 * z};
    Simulator::ScheduleNow([this, newPos]() {
      std::lock_guard<std::mutex> lock(node_mutex_);
      if (ueNode)
        ueNode->GetObject<MobilityModel>()->SetPosition(newPos);
    });
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    this->sendTwistOverUdp(*msg);
  }

  ns3::Ptr<ns3::Node> ueNode;
  ns3::Ptr<ns3::LteHelper> lteHelper;
  ns3::NetDeviceContainer ueLteDevs;
  ns3::NetDeviceContainer enbLteDevs;
  std::mutex node_mutex_;
  std::mutex rsrp_mutex_;
  std::map<uint16_t, float> rsrp_values_;
  Ptr<Socket> udp_socket_;
  Ipv4Address ue_ip_addr_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<ros_gz_interfaces::msg::Float32Array>::SharedPtr
      rsrp_publisher_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr rsrp_timer_;
};

std::shared_ptr<Ns3RosNode> g_ros_node = nullptr;

void PacketReceivedCallback(std::string, Ptr<const Packet> p, const Address &) {
  if (g_ros_node) {
    uint8_t buffer[p->GetSize()];
    p->CopyData(buffer, p->GetSize());
    std::string data(reinterpret_cast<char *>(buffer), p->GetSize());
    g_ros_node->publishTwistFromPacket(data);
  }
}

void ReportUeMeasurementsCallback(std::string, uint16_t, uint16_t cellId,
                                  double rsrp, double, bool, uint8_t) {
  if (g_ros_node) {
    g_ros_node->recordRsrpValue(cellId, rsrp);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  GlobalValue::Bind("SimulatorImplementationType",
                    StringValue("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

  double simTime = 600000.0;
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
  lteHelper->SetEpcHelper(epcHelper);

  Ptr<Node> pgw = epcHelper->GetPgwNode();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create(1);
  Ptr<Node> remoteHost = remoteHostContainer.Get(0);

  InternetStackHelper internet;
  internet.Install(remoteHostContainer);

  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
  p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
  p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
  NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase("1.0.0.0", "255.0.0.0");
  ipv4h.Assign(internetDevices);

  NodeContainer enbNodes, ueNodes;
  enbNodes.Create(3);
  ueNodes.Create(1);

  MobilityHelper enbMobility;
  enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  enbMobility.Install(enbNodes);
  enbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 0));
  enbNodes.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(300, 0, 0));
  enbNodes.Get(2)->GetObject<MobilityModel>()->SetPosition(Vector(150, 300, 0));

  MobilityHelper ueMobility;
  ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  ueMobility.Install(ueNodes);
  ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 0));

  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

  // TODO: Check if lowering power like this is going to affect anything other
  // than just signal strength
  for (uint32_t i = 0; i < enbLteDevs.GetN(); i++) {
    Ptr<LteEnbNetDevice> enbDevice =
        DynamicCast<LteEnbNetDevice>(enbLteDevs.Get(i));
    if (enbDevice) {
      Ptr<LteEnbPhy> enbPhy = enbDevice->GetPhy();
      enbPhy->SetTxPower(20.0); // Reduce from default ~46 dBm to 10 dBm
    }
  }
  internet.Install(ueNodes);
  epcHelper->AssignUeIpv4Address(ueLteDevs);

  for (uint16_t i = 0; i < ueNodes.GetN(); i++) {
    lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(0));
  }

  Ptr<Ipv4> ueIpv4 = ueNodes.Get(0)->GetObject<Ipv4>();
  Ipv4Address ueIpAddr = ueIpv4->GetAddress(1, 0).GetLocal();

  uint16_t dlPort = 1234;
  PacketSinkHelper packetSinkHelper(
      "ns3::UdpSocketFactory",
      InetSocketAddress(Ipv4Address::GetAny(), dlPort));
  ApplicationContainer sinkApps = packetSinkHelper.Install(ueNodes.Get(0));
  sinkApps.Start(Seconds(0.5));
  sinkApps.Stop(Seconds(simTime));

  UdpClientHelper client(ueIpAddr, dlPort);
  client.SetAttribute("MaxPackets", UintegerValue(100000));
  client.SetAttribute("Interval", TimeValue(Seconds(1e6)));
  client.SetAttribute("PacketSize", UintegerValue(1024));
  ApplicationContainer clientApps = client.Install(remoteHost);
  clientApps.Start(Seconds(1.0));
  clientApps.Stop(Seconds(simTime));

  uint16_t ulPort = 2000;
  PacketSinkHelper ulPacketSinkHelper(
      "ns3::UdpSocketFactory",
      InetSocketAddress(Ipv4Address::GetAny(), ulPort));
  ApplicationContainer ulSinkApps = ulPacketSinkHelper.Install(remoteHost);
  ulSinkApps.Start(Seconds(0.5));
  ulSinkApps.Stop(Seconds(simTime));

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
      ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
  remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                             Ipv4Mask("255.0.0.0"), 1);

  UdpClientHelper ulClient(Ipv4Address("1.0.0.1"), ulPort);
  ulClient.SetAttribute("MaxPackets", UintegerValue(100000));
  ulClient.SetAttribute("Interval", TimeValue(MilliSeconds(100)));
  ulClient.SetAttribute("PacketSize", UintegerValue(1024));
  ApplicationContainer ulClientApps = ulClient.Install(ueNodes.Get(0));
  ulClientApps.Start(Seconds(1.0));
  ulClientApps.Stop(Seconds(simTime));

  AnimationInterface anim("network_animations/ns3_ros2.xml");
  anim.UpdateNodeDescription(enbNodes.Get(0), "eNodeB 0");
  anim.UpdateNodeDescription(enbNodes.Get(1), "eNodeB 1");
  anim.UpdateNodeDescription(enbNodes.Get(2), "eNodeB 2");
  anim.UpdateNodeDescription(ueNodes.Get(0), "UE X3");

  anim.UpdateNodeColor(enbNodes.Get(0), 255, 0, 0);
  anim.UpdateNodeColor(enbNodes.Get(1), 255, 100, 0);
  anim.UpdateNodeColor(enbNodes.Get(2), 255, 200, 0);
  anim.UpdateNodeColor(ueNodes.Get(0), 0, 0, 255);

  anim.SetConstantPosition(enbNodes.Get(0), 0, 0);
  anim.SetConstantPosition(enbNodes.Get(1), 300, 0);
  anim.SetConstantPosition(enbNodes.Get(2), 150, 300);

  Ptr<Socket> udpSocket = Socket::CreateSocket(
      remoteHost, TypeId::LookupByName("ns3::UdpSocketFactory"));
  InetSocketAddress remoteAddr(Ipv4Address::GetAny(), 0);
  udpSocket->Bind(remoteAddr);
  udpSocket->SetAllowBroadcast(true);

  auto ros_node = std::make_shared<Ns3RosNode>();
  g_ros_node = ros_node;
  ros_node->setUeNode(ueNodes.Get(0));
  ros_node->setLteHelper(lteHelper);
  ros_node->setUeDevice(ueLteDevs);
  ros_node->setEnbDevices(enbLteDevs);
  ros_node->setUdpSocket(udpSocket, ueIpAddr);
  ros_node->createCmdVelSubscription();

  for (uint32_t i = 0; i < sinkApps.GetN(); i++) {
    Ptr<PacketSink> sink = DynamicCast<PacketSink>(sinkApps.Get(i));
    if (sink) {
      sink->TraceConnect("Rx", std::string(""),
                         MakeCallback(&PacketReceivedCallback));
      NS_LOG_UNCOND("Packet sink trace connected");
    }
  }

  ros_node->connectUePhyTraces();

  Simulator::Schedule(Seconds(2.0), [ros_node]() {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.z = 1.0;
    ros_node->sendTwistOverUdp(twist);
    NS_LOG_UNCOND("Scheduled twist: Going UP");
  });

  Simulator::Schedule(Seconds(5.0), [ros_node]() {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = 1.0;
    ros_node->sendTwistOverUdp(twist);
    NS_LOG_UNCOND("Scheduled twist: Going FORWARD");
  });

  Simulator::Schedule(Seconds(8.0), [ros_node]() {
    auto twist = geometry_msgs::msg::Twist();
    ros_node->sendTwistOverUdp(twist);
    NS_LOG_UNCOND("Scheduled twist: STOP");
  });

  std::thread ros_spin_thread([&]() {
    while (rclcpp::ok()) {
      rclcpp::spin_some(ros_node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  Simulator::Stop(Seconds(simTime));
  Simulator::Run();

  RCLCPP_INFO(ros_node->get_logger(),
              "Simulation complete, shutting down ROS2...");
  rclcpp::shutdown();
  if (ros_spin_thread.joinable())
    ros_spin_thread.join();

  Simulator::Destroy();
  return 0;
}

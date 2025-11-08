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
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <mutex>
#include <sstream>
#include <thread>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LteRos2RealtimeExample");

class Ns3RosNode : public rclcpp::Node {
public:
  Ns3RosNode() : Node("ns3_ros_node") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/X3/cmd_vel", 10);
    pose_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/model/X3/pose", 10,
        std::bind(&Ns3RosNode::poseCallback, this, std::placeholders::_1));
  }

  void setUeNode(ns3::Ptr<ns3::Node> node) {
    std::lock_guard<std::mutex> lock(node_mutex_);
    ueNode = node;
  }

  void publishTwistFromPacket(const std::string &data) {
    // Parse the serialized Twist data
    auto twist_msg = geometry_msgs::msg::Twist();

    // Simple parsing: "linear.x:1.5,linear.y:2.0,..."
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
      // Serialize the Twist message into a byte buffer
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
        if (sock) {
          sock->SendTo(packet, 0, InetSocketAddress(addr, 1234));
        }
      });
    }
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

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    this->sendTwistOverUdp(*msg);
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
    Vector3D newPos{20.0 * x, 20.0 * y, 20.0 * z};
    Simulator::ScheduleNow([this, newPos]() {
      std::lock_guard<std::mutex> lock(node_mutex_);
      if (ueNode) {
        ueNode->GetObject<MobilityModel>()->SetPosition(newPos);
      }
    });
  }

  ns3::Ptr<ns3::Node> ueNode;
  std::mutex node_mutex_;
  Ptr<Socket> udp_socket_;
  Ipv4Address ue_ip_addr_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
};

// Global ROS node pointer for packet sink callback
std::shared_ptr<Ns3RosNode> g_ros_node = nullptr;

void PacketReceivedCallback(std::string context, Ptr<const Packet> p,
                            const Address &addr) {
  if (g_ros_node) {
    // Extract payload data from packet
    uint8_t buffer[p->GetSize()];
    p->CopyData(buffer, p->GetSize());
    std::string data(reinterpret_cast<char *>(buffer), p->GetSize());

    // Parse and publish as Twist message
    g_ros_node->publishTwistFromPacket(data);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // NS-3 real-time simulation
  GlobalValue::Bind("SimulatorImplementationType",
                    StringValue("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

  double simTime = 60.0;

  // LTE setup
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

  // Nodes
  NodeContainer enbNodes;
  NodeContainer ueNodes;
  enbNodes.Create(3);
  ueNodes.Create(1);

  // eNodeB mobility - stationary, spread around the grid
  MobilityHelper enbMobility;
  enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  enbMobility.Install(enbNodes);
  enbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 0));
  enbNodes.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(300, 0, 0));
  enbNodes.Get(2)->GetObject<MobilityModel>()->SetPosition(Vector(150, 300, 0));

  // UE mobility - ROS transforms will move it around via scaling
  MobilityHelper ueMobility;
  ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  ueMobility.Install(ueNodes);
  ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 0));

  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

  internet.Install(ueNodes);
  epcHelper->AssignUeIpv4Address(ueLteDevs);

  // Attach UE to eNodeB
  for (uint16_t i = 0; i < ueNodes.GetN(); i++) {
    lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(0));
  }

  // Get UE IP address from the assigned interfaces
  Ptr<Ipv4> ueIpv4 = ueNodes.Get(0)->GetObject<Ipv4>();
  Ipv4Address ueIpAddr =
      ueIpv4->GetAddress(1, 0).GetLocal(); // Interface 1 is the LTE device

  // Create traffic: Remote host sends to UE
  uint16_t dlPort = 1234;
  ApplicationContainer clientApps;

  // Packet sink on UE (receives data)
  PacketSinkHelper packetSinkHelper(
      "ns3::UdpSocketFactory",
      InetSocketAddress(Ipv4Address::GetAny(), dlPort));
  ApplicationContainer sinkApps = packetSinkHelper.Install(ueNodes.Get(0));
  sinkApps.Start(Seconds(0.5));
  sinkApps.Stop(Seconds(simTime));

  // UDP client on remote host (sends data to UE) - no interval, triggered by
  // ROS
  UdpClientHelper client(ueIpAddr, dlPort);
  client.SetAttribute("MaxPackets", UintegerValue(100000));
  client.SetAttribute(
      "Interval",
      TimeValue(Seconds(1e6))); // Very large interval, we'll trigger manually
  client.SetAttribute("PacketSize", UintegerValue(1024));
  clientApps = client.Install(remoteHost);
  clientApps.Start(Seconds(1.0));
  clientApps.Stop(Seconds(simTime));

  // Reverse direction: UE sends to remote host
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

  // NetAnim visualization
  AnimationInterface anim("network_animations/ns3_ros2.xml");

  // Assign readable labels for NetAnim visualization
  anim.UpdateNodeDescription(enbNodes.Get(0), "eNodeB 0");
  anim.UpdateNodeDescription(enbNodes.Get(1), "eNodeB 1");
  anim.UpdateNodeDescription(enbNodes.Get(2), "eNodeB 2");
  anim.UpdateNodeDescription(ueNodes.Get(0), "UE X3");

  // Optional: color them differently
  anim.UpdateNodeColor(enbNodes.Get(0), 255, 0, 0);   // red for eNodeB 0
  anim.UpdateNodeColor(enbNodes.Get(1), 255, 100, 0); // orange for eNodeB 1
  anim.UpdateNodeColor(enbNodes.Get(2), 255, 200, 0); // yellow for eNodeB 2
  anim.UpdateNodeColor(ueNodes.Get(0), 0, 0, 255);    // blue for UE

  // Fixed positions for visualization
  anim.SetConstantPosition(enbNodes.Get(0), 0, 0);
  anim.SetConstantPosition(enbNodes.Get(1), 300, 0);
  anim.SetConstantPosition(enbNodes.Get(2), 150, 300);

  // Create a UDP socket on the remote host for sending data
  Ptr<Socket> udpSocket = Socket::CreateSocket(
      remoteHost, TypeId::LookupByName("ns3::UdpSocketFactory"));
  InetSocketAddress remoteAddr(Ipv4Address::GetAny(), 0);
  udpSocket->Bind(remoteAddr);
  udpSocket->SetAllowBroadcast(true);

  // ROS2 node
  auto ros_node = std::make_shared<Ns3RosNode>();
  g_ros_node = ros_node; // Set global pointer for callback
  ros_node->setUeNode(ueNodes.Get(0));
  ros_node->setUdpSocket(udpSocket, ueIpAddr);
  ros_node->createCmdVelSubscription();

  // Connect packet received trace to callback
  for (uint32_t i = 0; i < sinkApps.GetN(); i++) {
    Ptr<PacketSink> sink = DynamicCast<PacketSink>(sinkApps.Get(i));
    if (sink) {
      sink->TraceConnect("Rx", std::string(""),
                         MakeCallback(&PacketReceivedCallback));
      NS_LOG_UNCOND("Packet sink trace connected");
    }
  }

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

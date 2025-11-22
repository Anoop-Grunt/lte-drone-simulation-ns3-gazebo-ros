#include "ros_node.h"
#include "ns3/lte-module.h"
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Ns3RosNode");

// Define the global pointer
std::shared_ptr<Ns3RosNode> g_ros_node = nullptr;

// Constructor
Ns3RosNode::Ns3RosNode() : Node("ns3_ros_node"), current_cell_id_(0) {
  publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/X3/cmd_vel", 10);

  pose_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/model/X3/pose", 10,
      std::bind(&Ns3RosNode::poseCallback, this, std::placeholders::_1));

  rsrp_publisher_ =
      this->create_publisher<ros_gz_interfaces::msg::Float32Array>(
          "/rsrp_values", 10);

  handover_publisher_ =
      this->create_publisher<std_msgs::msg::UInt32>("/current_cell_id", 10);

  rsrp_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&Ns3RosNode::publishAllRsrpValues, this));
}

void Ns3RosNode::publishTwistFromPacket(const std::string &data) {
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

void Ns3RosNode::sendTwistOverUdp(const geometry_msgs::msg::Twist &twist) {
  std::lock_guard<std::mutex> lock(node_mutex_);
  if (udp_socket_) {
    std::ostringstream oss;
    oss << "linear.x:" << twist.linear.x << ",linear.y:" << twist.linear.y
        << ",linear.z:" << twist.linear.z << ",angular.x:" << twist.angular.x
        << ",angular.y:" << twist.angular.y << ",angular.z:" << twist.angular.z;
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

void Ns3RosNode::setUeNode(ns3::Ptr<ns3::Node> node) {
  std::lock_guard<std::mutex> lock(node_mutex_);
  ueNode = node;
}

void Ns3RosNode::setLteHelper(ns3::Ptr<ns3::LteHelper> helper) {
  std::lock_guard<std::mutex> lock(node_mutex_);
  lteHelper = helper;
}

void Ns3RosNode::setUeDevice(ns3::NetDeviceContainer devices) {
  std::lock_guard<std::mutex> lock(node_mutex_);
  ueLteDevs = devices;
}

void Ns3RosNode::setEnbDevices(ns3::NetDeviceContainer devices) {
  std::lock_guard<std::mutex> lock(node_mutex_);
  enbLteDevs = devices;
}

void Ns3RosNode::setUdpSocket(ns3::Ptr<ns3::Socket> socket,
                              ns3::Ipv4Address ue_ip) {
  std::lock_guard<std::mutex> lock(node_mutex_);
  udp_socket_ = socket;
  ue_ip_addr_ = ue_ip;
}

void Ns3RosNode::createCmdVelSubscription() {
  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/model_movement_commands", 10,
      std::bind(&Ns3RosNode::cmdVelCallback, this, std::placeholders::_1));
}

void Ns3RosNode::recordRsrpValue(uint16_t cellId, double rsrp) {
  std::lock_guard<std::mutex> lock(rsrp_mutex_);
  rsrp_values_[cellId] = static_cast<float>(rsrp);
}

void Ns3RosNode::publishAllRsrpValues() {
  std::lock_guard<std::mutex> lock(rsrp_mutex_);
  auto rsrp_msg = ros_gz_interfaces::msg::Float32Array();
  for (const auto &pair : rsrp_values_) {
    rsrp_msg.data.push_back(pair.second);
  }
  if (!rsrp_msg.data.empty()) {
    rsrp_publisher_->publish(rsrp_msg);
  }
}

void Ns3RosNode::notifyHandover(uint16_t newCellId) {
  RCLCPP_INFO(this->get_logger(), "Cell change: %d -> %d", current_cell_id_,
              newCellId);

  auto msg = std_msgs::msg::UInt32();
  msg.data = static_cast<uint32_t>(newCellId - 1);
  handover_publisher_->publish(msg);

  current_cell_id_ = newCellId;
}

void Ns3RosNode::connectUePhyTraces() {
  std::lock_guard<std::mutex> lock(node_mutex_);
  if (ueLteDevs.GetN() == 0)
    return;

  Ptr<LteUeNetDevice> ueDevice = DynamicCast<LteUeNetDevice>(ueLteDevs.Get(0));
  if (!ueDevice)
    return;

  Ptr<LteUePhy> uePhy = ueDevice->GetPhy();
  if (!uePhy)
    return;

  uePhy->TraceConnect("ReportUeMeasurements", std::string(""),
                      MakeCallback(&ReportUeMeasurementsCallback));
  NS_LOG_UNCOND("RSRP measurement trace connected directly to UE PHY");
}

void Ns3RosNode::connectUeRrcTraces() {
  std::lock_guard<std::mutex> lock(node_mutex_);
  if (ueLteDevs.GetN() == 0)
    return;

  Ptr<LteUeNetDevice> ueDevice = DynamicCast<LteUeNetDevice>(ueLteDevs.Get(0));
  if (!ueDevice)
    return;

  Ptr<LteUeRrc> ueRrc = ueDevice->GetRrc();
  if (!ueRrc)
    return;

  ueRrc->TraceConnect("ConnectionEstablished", std::string(""),
                      MakeCallback(&ConnectionEstablishedCallback));
  ueRrc->TraceConnect("HandoverEndOk", std::string(""),
                      MakeCallback(&HandoverEndOkCallback));

  NS_LOG_UNCOND(
      "UE RRC traces connected (ConnectionEstablished and HandoverEndOk)");
}

void Ns3RosNode::poseCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
  for (const auto &transform : msg->transforms) {
    if (transform.child_frame_id.find("X3") != std::string::npos) {
      scheduleUeMove(transform.transform.translation.x,
                     transform.transform.translation.y,
                     transform.transform.translation.z);
    }
  }
}

void Ns3RosNode::scheduleUeMove(double x, double y, double z) {
  Vector3D newPos{1.0 * x, 1.0 * y, 1.0 * z};
  Simulator::ScheduleNow([this, newPos]() {
    std::lock_guard<std::mutex> lock(node_mutex_);
    if (ueNode)
      ueNode->GetObject<MobilityModel>()->SetPosition(newPos);
  });
}

void Ns3RosNode::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  this->sendTwistOverUdp(*msg);
}

// NS3 Callback implementations
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

void ConnectionEstablishedCallback(std::string context, uint64_t imsi,
                                   uint16_t cellId, uint16_t rnti) {
  if (g_ros_node) {
    g_ros_node->notifyHandover(cellId);
  }
  NS_LOG_UNCOND("Initial connection established - IMSI: "
                << imsi << " connected to Cell " << cellId);
}

void HandoverEndOkCallback(std::string context, uint64_t imsi, uint16_t cellId,
                           uint16_t rnti) {
  if (g_ros_node) {
    g_ros_node->notifyHandover(cellId);
  }
  NS_LOG_UNCOND("Handover completed successfully - IMSI: "
                << imsi << " now connected to Cell " << cellId);
}

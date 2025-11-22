#ifndef NS3_ROS_NODE_HPP
#define NS3_ROS_NODE_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/float32_array.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <map>
#include <mutex>

class Ns3RosNode : public rclcpp::Node {
public:
  Ns3RosNode();

  // Message publishing methods
  void publishTwistFromPacket(const std::string &data);
  void sendTwistOverUdp(const geometry_msgs::msg::Twist &twist);
  void publishAllRsrpValues();

  // Setter methods for NS3 objects
  void setUeNode(ns3::Ptr<ns3::Node> node);
  void setLteHelper(ns3::Ptr<ns3::LteHelper> helper);
  void setUeDevice(ns3::NetDeviceContainer devices);
  void setEnbDevices(ns3::NetDeviceContainer devices);
  void setUdpSocket(ns3::Ptr<ns3::Socket> socket, ns3::Ipv4Address ue_ip);

  // Subscription creation
  void createCmdVelSubscription();

  // RSRP and handover methods
  void recordRsrpValue(uint16_t cellId, double rsrp);
  void notifyHandover(uint16_t newCellId);

  // Trace connection methods
  void connectUePhyTraces();
  void connectUeRrcTraces();

private:
  // ROS callback methods
  void poseCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // NS3 mobility helper
  void scheduleUeMove(double x, double y, double z);

  // NS3 objects
  ns3::Ptr<ns3::Node> ueNode;
  ns3::Ptr<ns3::LteHelper> lteHelper;
  ns3::NetDeviceContainer ueLteDevs;
  ns3::NetDeviceContainer enbLteDevs;
  ns3::Ptr<ns3::Socket> udp_socket_;
  ns3::Ipv4Address ue_ip_addr_;

  // State variables
  uint16_t current_cell_id_;
  std::map<uint16_t, float> rsrp_values_;

  // Thread safety
  std::mutex node_mutex_;
  std::mutex rsrp_mutex_;

  // ROS publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<ros_gz_interfaces::msg::Float32Array>::SharedPtr
      rsrp_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr handover_publisher_;

  // ROS subscribers
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  // ROS timers
  rclcpp::TimerBase::SharedPtr rsrp_timer_;
};

// Global pointer to the ROS node (needed for NS3 callbacks)
extern std::shared_ptr<Ns3RosNode> g_ros_node;

// NS3 callback function declarations
void PacketReceivedCallback(std::string context, ns3::Ptr<const ns3::Packet> p,
                            const ns3::Address &addr);
void ReportUeMeasurementsCallback(std::string context, uint16_t rnti,
                                  uint16_t cellId, double rsrp, double rsrq,
                                  bool servingCell, uint8_t componentCarrierId);
void ConnectionEstablishedCallback(std::string context, uint64_t imsi,
                                   uint16_t cellId, uint16_t rnti);
void HandoverEndOkCallback(std::string context, uint64_t imsi, uint16_t cellId,
                           uint16_t rnti);

#endif // NS3_ROS_NODE_HPP

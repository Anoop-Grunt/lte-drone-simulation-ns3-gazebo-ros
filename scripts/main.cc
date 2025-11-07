
#include "geometry_msgs/msg/twist.hpp"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <thread>

using namespace ns3;
using namespace std::chrono_literals;

NS_LOG_COMPONENT_DEFINE("LteRos2RealtimeExample");

class Ns3RosNode : public rclcpp::Node {
public:
  Ns3RosNode() : Node("ns3_twist_publisher") {
    // Publisher to Gazebo teleop
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/X3/cmd_vel", 10);

    // Subscriber to world pose topic (change topic name if needed)
    pose_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/world/quadcopter_teleop/pose/info", 10,
        std::bind(&Ns3RosNode::poseCallback, this, std::placeholders::_1));
  }

  void publishTwist(double linear_x, double angular_z) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f, angular.z=%.2f",
                linear_x, angular_z);
    publisher_->publish(msg);
  }

private:
  void poseCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    for (const auto &transform : msg->transforms) {
      // Each transform has:
      // - header.frame_id     → usually "world" or the parent frame
      // - child_frame_id      → name of the model or link (e.g.,
      // "X3::base_link")
      std::string model_name = transform.child_frame_id;

      // If you want only top-level models (not links inside them)
      auto pos = model_name.find("::");
      if (pos != std::string::npos) {
        model_name = model_name.substr(0, pos);
      }

      RCLCPP_INFO(rclcpp::get_logger("pose_callback"),
                  "Model: %s | Parent: %s | Position: [%.2f, %.2f, %.2f]",
                  model_name.c_str(), transform.header.frame_id.c_str(),
                  transform.transform.translation.x,
                  transform.transform.translation.y,
                  transform.transform.translation.z);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_sub_;
};

int main(int argc, char *argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<Ns3RosNode>();

  // Enable real-time simulation
  GlobalValue::Bind("SimulatorImplementationType",
                    StringValue("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

  double simTime = 100.0;
  uint16_t numberOfNodes = 2;

  // NS-3 setup
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

  NodeContainer enbNodes;
  NodeContainer ueNodes;
  enbNodes.Create(1);
  ueNodes.Create(numberOfNodes);
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(enbNodes);
  mobility.Install(ueNodes);

  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);
  internet.Install(ueNodes);
  epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));

  for (uint16_t i = 0; i < numberOfNodes; i++) {
    lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(0));
  }

  // Start a background thread to keep ROS spinning
  std::thread ros_spin_thread([&]() { rclcpp::spin(ros_node); });

  // Schedule Twist publishing in ns-3 time (real-time aligned)
  Simulator::Schedule(Seconds(5.0),
                      [ros_node]() { ros_node->publishTwist(0.5, 0.0); });
  Simulator::Schedule(Seconds(15.0),
                      [ros_node]() { ros_node->publishTwist(0.3, 0.5); });
  Simulator::Schedule(Seconds(25.0),
                      [ros_node]() { ros_node->publishTwist(0.0, 1.0); });

  NS_LOG_INFO("Running simulation in real-time mode...");
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();

  NS_LOG_INFO("Simulation complete, shutting down ROS2...");
  rclcpp::shutdown();
  if (ros_spin_thread.joinable())
    ros_spin_thread.join();

  Simulator::Destroy();
  return 0;
}

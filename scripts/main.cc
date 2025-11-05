#include "geometry_msgs/msg/twist.hpp"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "rclcpp/rclcpp.hpp"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LteRos2Example");

class Ns3RosPublisher : public rclcpp::Node {
public:
  Ns3RosPublisher() : Node("ns3_twist_publisher") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/X3/cmd_vel", 10);
  }

  void publishTwist(double linear_x, double angular_z) {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = linear_x;
    message.linear.y = 0.0;
    message.linear.z = 0.0;
    message.angular.x = 0.0;
    message.angular.y = 0.0;
    message.angular.z = angular_z;

    RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f, angular.z=%.2f",
                linear_x, angular_z);
    publisher_->publish(message);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  // Initialize ROS2
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<Ns3RosPublisher>();

  // NS-3 simulation parameters
  double simTime = 10.0;
  uint16_t numberOfNodes = 2;

  // Create LTE helper objects
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
  lteHelper->SetEpcHelper(epcHelper);

  // Get PGW node
  Ptr<Node> pgw = epcHelper->GetPgwNode();

  // Create remote host
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create(1);
  Ptr<Node> remoteHost = remoteHostContainer.Get(0);
  InternetStackHelper internet;
  internet.Install(remoteHostContainer);

  // Connect remote host to PGW
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
  p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
  p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
  NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

  // Create eNB and UE nodes
  NodeContainer enbNodes;
  NodeContainer ueNodes;
  enbNodes.Create(1);
  ueNodes.Create(numberOfNodes);

  // Install mobility
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(enbNodes);
  mobility.Install(ueNodes);

  // Install LTE devices
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

  // Install Internet stack on UEs
  internet.Install(ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));

  // Attach UEs to eNB
  for (uint16_t i = 0; i < numberOfNodes; i++) {
    lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(0));
  }

  // Schedule ROS2 Twist message publishing from NS-3 simulation
  Simulator::Schedule(Seconds(1.0), [&ros_node]() {
    ros_node->publishTwist(0.5, 0.0); // Move forward
  });

  Simulator::Schedule(Seconds(3.0), [&ros_node]() {
    ros_node->publishTwist(0.3, 0.5); // Move forward while turning
  });

  Simulator::Schedule(Seconds(5.0), [&ros_node]() {
    ros_node->publishTwist(0.0, 1.0); // Rotate in place
  });

  Simulator::Schedule(Seconds(7.0), [&ros_node]() {
    ros_node->publishTwist(0.0, 0.0); // Stop
  });

  NS_LOG_INFO("Starting simulation...");
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();

  // Spin ROS2 for a moment to ensure messages are sent
  rclcpp::spin_some(ros_node);

  Simulator::Destroy();
  rclcpp::shutdown();

  NS_LOG_INFO("Simulation finished.");
  return 0;
}

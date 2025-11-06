
#include "geometry_msgs/msg/twist.hpp"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "rclcpp/rclcpp.hpp"

#include <thread>

using namespace ns3;
using namespace std::chrono_literals;

NS_LOG_COMPONENT_DEFINE("LteRos2RealtimeExample");

class Ns3RosPublisher : public rclcpp::Node {
public:
  Ns3RosPublisher() : Node("ns3_twist_publisher") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/X3/cmd_vel", 10);
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
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<Ns3RosPublisher>();

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
  Simulator::Schedule(Seconds(35.0),
                      [ros_node]() { ros_node->publishTwist(0.0, 0.0); });
  Simulator::Schedule(Seconds(45.0),
                      [ros_node]() { ros_node->publishTwist(0.5, 0.0); });
  Simulator::Schedule(Seconds(55.0),
                      [ros_node]() { ros_node->publishTwist(0.2, -0.3); });
  Simulator::Schedule(Seconds(65.0),
                      [ros_node]() { ros_node->publishTwist(0.0, 0.5); });
  Simulator::Schedule(Seconds(75.0),
                      [ros_node]() { ros_node->publishTwist(0.0, 0.0); });
  Simulator::Schedule(Seconds(85.0),
                      [ros_node]() { ros_node->publishTwist(0.4, 0.0); });
  Simulator::Schedule(Seconds(95.0),
                      [ros_node]() { ros_node->publishTwist(0.0, -0.5); });
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

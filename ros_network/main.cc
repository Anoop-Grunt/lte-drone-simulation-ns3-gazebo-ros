#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ros_node.h"
#include <thread>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LteRos2RealtimeExample");

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  GlobalValue::Bind("SimulatorImplementationType",
                    StringValue("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));

  double simTime = 600000.0;
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
  lteHelper->SetEpcHelper(epcHelper);

  lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
  lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(0.0));
  lteHelper->SetHandoverAlgorithmAttribute("TimeToTrigger",
                                           TimeValue(MilliSeconds(40)));

  NS_LOG_UNCOND("Handover algorithm configured");

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
  enbNodes.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(150, 0, 0));
  enbNodes.Get(2)->GetObject<MobilityModel>()->SetPosition(Vector(75, 150, 0));

  MobilityHelper ueMobility;
  ueMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  ueMobility.Install(ueNodes);
  ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 10));

  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
  lteHelper->AddX2Interface(enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

  LogComponentEnable("A3RsrpHandoverAlgorithm", LOG_LEVEL_ALL);

  // Reduce transmit power for clearer handover demonstration
  for (uint32_t i = 0; i < enbLteDevs.GetN(); i++) {
    Ptr<LteEnbNetDevice> enbDevice =
        DynamicCast<LteEnbNetDevice>(enbLteDevs.Get(i));
    if (enbDevice) {
      Ptr<LteEnbPhy> enbPhy = enbDevice->GetPhy();
      enbPhy->SetTxPower(20.0);
    }
  }

  internet.Install(ueNodes);
  epcHelper->AssignUeIpv4Address(ueLteDevs);

  for (uint16_t i = 0; i < ueNodes.GetN(); i++) {
    lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(0));
  }

  Ptr<Ipv4> ueIpv4 = ueNodes.Get(0)->GetObject<Ipv4>();
  Ipv4Address ueIpAddr = ueIpv4->GetAddress(1, 0).GetLocal();

  // Downlink UDP application
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

  // Uplink UDP application
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

  // NetAnim configuration
  AnimationInterface anim("network_animations/ns3_ros2.xml");

  // my netanim bin directory has a file called softgray.png that i created with
  // the following command: convert -size 4x4 xc:"rgb(200,200,200)" softgray.png
  anim.SetBackgroundImage("softgray.png", -150.0, -150.0, 150.0, 150.0, 1.0);
  anim.UpdateNodeDescription(enbNodes.Get(0), "eNodeB 0");
  anim.UpdateNodeDescription(enbNodes.Get(1), "eNodeB 1");
  anim.UpdateNodeDescription(enbNodes.Get(2), "eNodeB 2");
  anim.UpdateNodeDescription(ueNodes.Get(0), "UE X3");

  anim.UpdateNodeColor(enbNodes.Get(0), 255, 0, 0);
  anim.UpdateNodeColor(enbNodes.Get(1), 255, 100, 0);
  anim.UpdateNodeColor(enbNodes.Get(2), 255, 200, 0);
  anim.UpdateNodeColor(ueNodes.Get(0), 0, 0, 255);

  anim.SetConstantPosition(enbNodes.Get(0), 0, 0);
  anim.SetConstantPosition(enbNodes.Get(1), 150, 0);
  anim.SetConstantPosition(enbNodes.Get(2), 70, 150);

  // Create UDP socket for remote host
  Ptr<Socket> udpSocket = Socket::CreateSocket(
      remoteHost, TypeId::LookupByName("ns3::UdpSocketFactory"));
  InetSocketAddress remoteAddr(Ipv4Address::GetAny(), 0);
  udpSocket->Bind(remoteAddr);
  udpSocket->SetAllowBroadcast(true);

  // Initialize ROS node
  auto ros_node = std::make_shared<Ns3RosNode>();
  g_ros_node = ros_node;
  ros_node->setUeNode(ueNodes.Get(0));
  ros_node->setLteHelper(lteHelper);
  ros_node->setUeDevice(ueLteDevs);
  ros_node->setEnbDevices(enbLteDevs);
  ros_node->setUdpSocket(udpSocket, ueIpAddr);
  ros_node->createCmdVelSubscription();

  // Connect packet sink trace
  for (uint32_t i = 0; i < sinkApps.GetN(); i++) {
    Ptr<PacketSink> sink = DynamicCast<PacketSink>(sinkApps.Get(i));
    if (sink) {
      sink->TraceConnect("Rx", std::string(""),
                         MakeCallback(&PacketReceivedCallback));
    }
  }

  // Connect UE traces
  ros_node->connectUePhyTraces();
  ros_node->connectUeRrcTraces();

  // Schedule test movements
  Simulator::Schedule(Seconds(2.0), [ros_node]() {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.z = 1.0;
    ros_node->sendTwistOverUdp(twist);
  });

  Simulator::Schedule(Seconds(5.0), [ros_node]() {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = 1.0;
    ros_node->sendTwistOverUdp(twist);
  });

  Simulator::Schedule(Seconds(8.0), [ros_node]() {
    auto twist = geometry_msgs::msg::Twist();
    ros_node->sendTwistOverUdp(twist);
  });

  // ROS2 spin thread
  std::thread ros_spin_thread([&]() {
    while (rclcpp::ok()) {
      rclcpp::spin_some(ros_node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  // Run simulation
  Simulator::Stop(Seconds(simTime));
  // trying to flush the netanim animation more frequently
  for (double t = 10.0; t < simTime; t += 10.0) {
    Simulator::Schedule(Seconds(t), [&anim, &ueNodes]() {
      anim.UpdateNodeDescription(ueNodes.Get(0), "UE X3");
    });
  }
  Simulator::Run();

  RCLCPP_INFO(ros_node->get_logger(),
              "Simulation complete, shutting down ROS2...");
  rclcpp::shutdown();
  if (ros_spin_thread.joinable())
    ros_spin_thread.join();

  Simulator::Destroy();
  return 0;
}

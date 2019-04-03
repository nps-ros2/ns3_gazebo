// adapted from https://raw.githubusercontent.com/ros2/demos/master/demo_nodes_cpp/src/topics/listener.cpp

#include <cstdio>
#include <memory>
#include <string>

#include "ns3/core-module.h"
#include "rclcpp/rclcpp.hpp"
#include "diff_drive_robot.hpp"

// network devices, do not exceed COUNT in nns_setup.py
static const int COUNT=5;

void set_up_ns3(ns3::NodeContainer& ns3_nodes) {

  // run ns3 real-time with checksums
  ns3::GlobalValue::Bind("SimulatorImplementationType",
                          ns3::StringValue("ns3::RealtimeSimulatorImpl"));
  ns3::GlobalValue::Bind("ChecksumEnabled", ns3::BooleanValue(true));

  // Create ns3_nodes
  ns3_nodes.Create(COUNT);

  // Wifi settings
  ns3::WifiHelper wifi;
  wifi.SetStandard(ns3::WIFI_PHY_STANDARD_80211a);
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                          "DataMode", ns3::StringValue("OfdmRate54Mbps"));

  // ad-hoc Wifi network
  ns3::WifiMacHelper wifiMac;
  wifiMac.SetType("ns3::AdhocWifiMac");

  // physical layer
  ns3::YansWifiChannelHelper wifiChannel(ns3::YansWifiChannelHelper::Default());
  ns3::YansWifiPhyHelper wifiPhy(ns3::YansWifiPhyHelper::Default());
  wifiPhy.SetChannel(wifiChannel.Create());

  // Install the wireless devices onto our ghost ns3_nodes.
  ns3::NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, ns3_nodes);

  // antenna locations
  ns3::Ptr<ns3::ListPositionAllocator>positionAlloc =
                         ns3::CreateObject<ns3::ListPositionAllocator>();
  for (int i=0; i<COUNT; i++) {
    positionAlloc->Add(ns3::Vector(i, 0.0, 0.0));
  }
  ns3::MobilityHelper mobility;
  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(ns3_nodes);

  // connect Wifi through TapBridge devices
  ns3::TapBridgeHelper tapBridge;
  tapBridge.SetAttribute("Mode", ns3::StringValue("UseLocal"));
  char buffer[10];
  for (int i=0; i<COUNT; i++) {
    sprintf(buffer, "wifi_tap%d", i+1);
    tapBridge.SetAttribute("DeviceName", ns3::StringValue(buffer));
    tapBridge.Install(ns3_nodes.Get(i), devices.Get(i));
  }
}

void robot_thread_function(ns3::NodeContainer* ns3_nodes_ptr) {

  // Create the ROS2 node
  std::shared_ptr<rclcpp::Node> rclcpp_node = rclcpp::Node::make_shared(
                                                   "diff_drive_node");

  // create the robot, give the rclcpp_node and ns3_compnent nodes to it
  std::unique_ptr<diff_drive_robot::DiffDriveRobot> robot_ = 
             std::make_unique<diff_drive_robot::DiffDriveRobot>(rclcpp_node,
                                                      ns3_nodes_ptr);

  // spin will block until work comes in, execute work as it becomes
  // available, and keep blocking.  It will only be interrupted by Ctrl-C.
  std::cout << "Starting robot in thread.\n";
  rclcpp::spin(rclcpp_node);
  rclcpp::shutdown();
  std::cout << "Stopped robot in thread.\n";
}

int main(int argc, char * argv[]) {
  // information
  std::cout << "Starting ns-3 Wifi simulator and ns-3 feedback robot.\n"
            << "Press Ctrl-C twice to stop both.\n";

  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Initialize ROS2 first, this should be called exactly once per process.
  rclcpp::init(argc, argv);

  // create the ns-3 nodes container
  ns3::NodeContainer ns3_nodes;

  // set up ns-3
  set_up_ns3(ns3_nodes);

  // start the robot as a second thread
  std::thread robot_thread(robot_thread_function, &ns3_nodes);

  // set to run for one year
  ns3::Simulator::Stop(ns3::Seconds(60*60*24*365.));

  // run until Ctrl-C
  std::cout << "Starting ns-3 Wifi simulator in main.\n";
  ns3::Simulator::Run();

  ns3::Simulator::Destroy();
  robot_thread.join(); // gracefully let the robot thread stop
  std::cout << "Stopped ns-3 Wifi simulator in main.\n";
  std::cout << "Done.\n";
  return 0;
}

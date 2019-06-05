// adapted from https://raw.githubusercontent.com/ros2/demos/master/demo_nodes_cpp/src/topics/listener.cpp

#include <cstdio>
#include <memory>
#include <string>

#include "ns3/core-module.h"
#include "ns3/node-container.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/random-variable-stream.h"

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
    positionAlloc->Add(ns3::Vector(0.0, 0.0, 0.0));
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

int main(int argc, char * argv[]) {
  // information
  std::cout << "Initializing ns-3 Wifi simulator.\n";

  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Initialize ROS2 first, this should be called exactly once per process.
//  rclcpp::init(argc, argv);

  // create the ns-3 nodes container
  ns3::NodeContainer ns3_nodes;

  // set up ns-3
  set_up_ns3(ns3_nodes);

  // set to run for one year
//  ns3::Simulator::Stop(ns3::Seconds(60*60*24*365.));
  ns3::Simulator::Stop(ns3::Seconds(6.0));

  // run until Ctrl-C
  std::cout << "Starting ns-3 Wifi simulator.\n";
  ns3::Simulator::Run();

  ns3::Simulator::Destroy();
  std::cout << "Stopped ns-3 Wifi simulator.\n";
  std::cout << "Done.\n";
  return 0;
}


#include <cstdio>
//#include <memory>
//#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>

#include "ns3/core-module.h"
#include "ns3/node-container.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"

namespace ns3_gazebo_world {

static const int COUNT=5;

void ns3_setup(ns3::NodeContainer& ns3_nodes) {

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

static void ns3_thread_function(void) {
  std::cout << "Starting ns-3 Wifi simulator in thread.\n";
  ns3::Simulator::Run();
  ns3::Simulator::Destroy();
  std::cout << "Ending ns-3 Wifi simulator in thread.\n";
}

class NS3GazeboWorld : public gazebo::WorldPlugin {
  private:
  ns3::NodeContainer ns3_nodes;
  std::thread ns3_thread;
//  std::thread ns3_thread(ns3_thread_function);
  gazebo::physics::ModelPtr model_ptr;
  gazebo::event::ConnectionPtr update_connection;

  public:
  NS3GazeboWorld() : gazebo::WorldPlugin(), 
                     ns3_nodes(), ns3_thread(),
                     model_ptr(0), update_connection(0) {
    printf("Hello World!\n");
  }

  ~NS3GazeboWorld() {
    if (&ns3_thread != NULL) {
      ns3_thread.join(); // gracefully let the robot thread stop
      std::cout << "Stopped ns-3 Wifi simulator in main.\n";
    }
  }

  void OnUpdate(const gazebo::common::UpdateInfo& _info) {
    ignition::math::Pose3d pose = model_ptr->WorldPose();
    float x = pose.Pos().X();
//    std::cout << "OnUpdate Point.x: " << x << "\n";
  }

  void Init() {
    std::cout << "Init\n";
  }

  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) {
    std::cout << "Load: model by name\n";
    model_ptr = _world->ModelByName("vehicle");
    ignition::math::Pose3d pose = model_ptr->WorldPose();
    float x = pose.Pos().X();
    std::cout << "Load Point.x: " << x << "\n";

    // do every Gazebo simulation iteration
    update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
                    std::bind(&NS3GazeboWorld::OnUpdate,
                    this, std::placeholders::_1));
  
    // ns-3
//    // create the ns-3 nodes container
//    ns3::NodeContainer ns3_nodes;

    // set up ns-3
    ns3_setup(ns3_nodes);

    // set to run for one year
    ns3::Simulator::Stop(ns3::Seconds(60*60*24*365.));

    // start the ns3 thread
    ns3_thread = std::thread(ns3_thread_function);
  }
};
GZ_REGISTER_WORLD_PLUGIN(NS3GazeboWorld)

} // namespace


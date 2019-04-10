#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>

namespace ns3_gazebo {

class WorldPluginTutorial : public gazebo::WorldPlugin {
  private:
  gazebo::physics::ModelPtr model_ptr;
  gazebo::event::ConnectionPtr update_connection;

  public:
  WorldPluginTutorial() : gazebo::WorldPlugin(), model_ptr(0),
                          update_connection(0) {
    printf("Hello World!\n");
  }

  void OnUpdate(const gazebo::common::UpdateInfo& _info) {
    ignition::math::Pose3d pose = model_ptr->WorldPose();
    float x = pose.Pos().X();
    std::cout << "OnUpdate Point.x: " << x << "\n";
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

    // do every simulation iteration
    update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
                    std::bind(&WorldPluginTutorial::OnUpdate,
                    this, std::placeholders::_1));
  
  }
};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)

} // namespace


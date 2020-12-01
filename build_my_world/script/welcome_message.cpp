#include <gazebo/gazebo.hh>

namespace gazebo
{
  class WorldPluginMyMessage : public WorldPlugin
  {
    public: WorldPluginMyMessage() : WorldPlugin()
            {
              printf("Welcome to Prasun's World!\n");
            }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
            }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginMyMessage)
}

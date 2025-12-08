/*******************************************************************************
 * DeadZonePlugin - Gazebo Harmonic (gz-sim8) World Plugin
 *
 * PURPOSE:
 *   Detects when wildlife models enter a smoke zone radius and marks them as "dead".
 *   Publishes death notifications to Gazebo transport for scoring/logging.
 *
 * COMPATIBILITY:
 *   - Gazebo: Harmonic (gz-sim8)
 *   - ROS2: Jazzy
 *   - Paired with: sydney_regatta_smoke_wildlife.sdf
 *
 * BUILD:
 *   cd ~/seal_ws && colcon build --packages-select environment_plugins
 *   source install/setup.bash
 *
 * USAGE IN SDF:
 *   <plugin filename="libdead_zone_plugin.so" name="dead_zone_plugin">
 *     <dead_zone_radius>6.0</dead_zone_radius>
 *     <smoke_models>
 *       <name>smoke_generator</name>
 *       <name>smoke_generator_east</name>
 *       <name>smoke_generator_north</name>
 *       <name>smoke_generator_platypus</name>
 *       <name>smoke_generator_crocodile</name>
 *       <name>smoke_generator_turtle</name>
 *     </smoke_models>
 *     <wildlife_models>
 *       <name>crocodile</name>
 *       <name>platypus</name>
 *       <name>turtle</name>
 *     </wildlife_models>
 *   </plugin>
 *
 * DEBUG COMMANDS:
 *   # Launch with filtered plugin output:
 *   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta_smoke_wildlife 2>&1 | grep -E "DeadZonePlugin"
 *
 *   # Monitor death notifications (Gazebo transport):
 *   gz topic -e -t /vrx/wildlife/death
 *
 *   # List all Gazebo topics:
 *   gz topic -l
 *
 * CONFIGURATION PARAMETERS:
 *   - dead_zone_radius: Distance from smoke center that kills wildlife (default: 6.0m)
 *   - smoke_models/name: List of smoke generator model names to track
 *   - wildlife_models/name: List of wildlife model names to monitor
 *
 * KNOWN LIMITATIONS (gz-sim8 / Harmonic):
 *   1. NO VISUAL DEATH EFFECTS: Visual changes (belly flip, color change, transparency)
 *      are NOT possible in Gazebo Harmonic because:
 *      - Physics plugins (buoyancy, TrajectoryFollower) override pose changes
 *      - Material/visual properties cannot be modified at runtime through ECM
 *      - Making models "static" doesn't work for already-spawned dynamic models
 *
 *   2. DRAG-AND-DROP DETECTION: Manually dragging a model into a smoke zone may
 *      not trigger death detection. Only pre-configured wildlife models are tracked.
 *
 *   3. ENTITY RESOLUTION DELAY: Wildlife models loaded from Fuel URLs are not
 *      available at Configure() time. This plugin uses lazy resolution in
 *      PreUpdate() to find entities after they spawn.
 *
 * WHAT WORKS:
 *   - Death detection when wildlife enters smoke radius
 *   - Terminal logging of death events with coordinates
 *   - Gazebo transport publication to /vrx/wildlife/death
 *   - Periodic debug logging of wildlife positions (every ~5 seconds)
 *
 ******************************************************************************/

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Model.hh>

#include <gz/math/Vector3.hh>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <string>

using namespace gz;
using namespace gz::sim;

class DeadZonePlugin : public System,
                       public ISystemConfigure,
                       public ISystemPreUpdate
{
public:
  void Configure(const Entity &,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &,
                 EventManager &) override
  {
    gzmsg << "[DeadZonePlugin] ========== PLUGIN LOADING ==========\n";

    // Parse smoke model names
    if (sdf && sdf->HasElement("smoke_models"))
    {
      auto elem = const_cast<sdf::Element *>(sdf.get())->GetElement("smoke_models");
      for (auto child = elem->GetFirstElement(); child;
           child = child->GetNextElement())
      {
        if (child->GetName() == "name")
        {
          smokeNames.push_back(child->Get<std::string>());
          gzmsg << "[DeadZonePlugin] Config smoke: " << child->Get<std::string>() << "\n";
        }
      }
    }
    if (smokeNames.empty())
    {
      smokeNames = {"smoke_generator", "smoke_generator_east", "smoke_generator_north",
                    "smoke_generator_platypus", "smoke_generator_crocodile", "smoke_generator_turtle"};
    }

    // Parse wildlife model names
    if (sdf && sdf->HasElement("wildlife_models"))
    {
      auto elem = const_cast<sdf::Element *>(sdf.get())->GetElement("wildlife_models");
      for (auto child = elem->GetFirstElement(); child;
           child = child->GetNextElement())
      {
        if (child->GetName() == "name")
        {
          wildlifeNames.push_back(child->Get<std::string>());
          gzmsg << "[DeadZonePlugin] Config wildlife: " << child->Get<std::string>() << "\n";
        }
      }
    }
    if (wildlifeNames.empty())
    {
      wildlifeNames = {"crocodile", "platypus", "turtle"};
    }

    // Parse dead zone radius
    if (sdf && sdf->HasElement("dead_zone_radius"))
    {
      deadZoneRadius = sdf->Get<double>("dead_zone_radius");
    }

    // Advertise death notification topic
    deathPub = node.Advertise<gz::msgs::StringMsg>("/vrx/wildlife/death");

    gzmsg << "[DeadZonePlugin] Dead zone radius: " << deadZoneRadius << "m\n";
    gzmsg << "[DeadZonePlugin] ========== CONFIGURE DONE ==========\n";
  }

  // PreUpdate is called every simulation step before physics runs.
  // This is where we check for wildlife entering smoke zones.
  void PreUpdate(const UpdateInfo &, EntityComponentManager &ecm) override
  {
    // LAZY ENTITY RESOLUTION: Models from Fuel URLs aren't available at Configure() time.
    // We must search for them here in PreUpdate() after they've spawned.
    if (!entitiesResolved)
    {
      TryResolveEntities(ecm);
      return; // Don't process until entities are resolved
    }

    // Collect smoke positions
    std::vector<math::Vector3d> smokePositions;
    for (auto ent : smokeEntities)
    {
      auto pose = ecm.Component<components::Pose>(ent);
      if (pose)
        smokePositions.push_back(pose->Data().Pos());
    }

    // Debug: periodically log positions (every ~5 seconds at 250Hz)
    static int debugCounter = 0;
    debugCounter++;
    bool shouldLog = (debugCounter % 1250 == 0);

    // Check living wildlife for entering dead zones
    for (auto ent : wildlifeEntities)
    {
      if (deadEntities.count(ent))
        continue;

      auto poseComp = ecm.Component<components::Pose>(ent);
      if (!poseComp)
        continue;

      const auto pos = poseComp->Data().Pos();

      // Debug: log wildlife position periodically
      if (shouldLog && wildlifeNameMap.count(ent))
      {
        double minDist = 999999;
        for (const auto &smokePos : smokePositions)
        {
          double d = pos.Distance(smokePos);
          if (d < minDist) minDist = d;
        }
        gzmsg << "[DeadZonePlugin] " << wildlifeNameMap[ent]
              << " at (" << pos.X() << ", " << pos.Y() << ") - nearest smoke: " << minDist << "m\n";
      }

      for (size_t i = 0; i < smokePositions.size(); ++i)
      {
        const double dist = pos.Distance(smokePositions[i]);
        if (dist <= deadZoneRadius)
        {
          MarkDead(ent, pos);
          break;
        }
      }
    }
  }

private:
  void TryResolveEntities(EntityComponentManager &ecm)
  {
    static int tryCount = 0;
    tryCount++;

    // Log every 100 attempts
    if (tryCount % 100 == 1)
    {
      gzmsg << "[DeadZonePlugin] Searching for entities (attempt " << tryCount << ")...\n";
    }

    // Iterate all entities looking for models with matching names
    ecm.Each<components::Model, components::Name>(
      [this](const Entity &entity, const components::Model *, const components::Name *nameComp) -> bool
      {
        std::string name = nameComp->Data();

        // Check if this is a smoke model
        for (const auto &smokeName : smokeNames)
        {
          if (name == smokeName && std::find(smokeEntities.begin(), smokeEntities.end(), entity) == smokeEntities.end())
          {
            smokeEntities.push_back(entity);
            gzmsg << "[DeadZonePlugin] Found smoke model: " << name << " (entity " << entity << ")\n";
          }
        }

        // Check if this is a wildlife model
        for (const auto &wildlifeName : wildlifeNames)
        {
          if (name == wildlifeName && std::find(wildlifeEntities.begin(), wildlifeEntities.end(), entity) == wildlifeEntities.end())
          {
            wildlifeEntities.push_back(entity);
            wildlifeNameMap[entity] = name;
            gzmsg << "[DeadZonePlugin] Found wildlife model: " << name << " (entity " << entity << ")\n";
          }
        }

        return true; // Continue iteration
      });

    // Check if all entities are resolved
    if (smokeEntities.size() >= smokeNames.size() && wildlifeEntities.size() >= wildlifeNames.size())
    {
      entitiesResolved = true;
      gzmsg << "[DeadZonePlugin] ========== ALL ENTITIES FOUND ==========\n";
      gzmsg << "[DeadZonePlugin] " << smokeEntities.size() << " smoke zones, "
            << wildlifeEntities.size() << " wildlife\n";

      // Log positions
      for (size_t i = 0; i < smokeEntities.size(); ++i)
      {
        auto pose = ecm.Component<components::Pose>(smokeEntities[i]);
        if (pose)
        {
          auto p = pose->Data().Pos();
          gzmsg << "[DeadZonePlugin] Smoke at (" << p.X() << ", " << p.Y() << ", " << p.Z() << ")\n";
        }
      }
      for (const auto &[ent, name] : wildlifeNameMap)
      {
        auto pose = ecm.Component<components::Pose>(ent);
        if (pose)
        {
          auto p = pose->Data().Pos();
          gzmsg << "[DeadZonePlugin] Wildlife '" << name << "' at (" << p.X() << ", " << p.Y() << ", " << p.Z() << ")\n";
        }
      }
      gzmsg << "[DeadZonePlugin] ========================================\n";
    }
  }

  // MarkDead: Called when wildlife enters smoke zone radius.
  // Logs the death event and publishes to Gazebo transport for scoring.
  // NOTE: Visual effects are not possible in Gazebo Harmonic (see header comments).
  void MarkDead(Entity ent, const math::Vector3d &deathPos)
  {
    deadEntities.insert(ent);

    // Get animal name for logging
    std::string animalName = "unknown";
    if (wildlifeNameMap.count(ent))
      animalName = wildlifeNameMap[ent];

    // Log death event
    gzmsg << "[DeadZonePlugin] ***** " << animalName << " DIED at ("
          << deathPos.X() << ", " << deathPos.Y() << ", " << deathPos.Z()
          << ") *****\n";

    // Publish death notification to Gazebo transport
    // Monitor with: gz topic -e -t /vrx/wildlife/death
    gz::msgs::StringMsg msg;
    msg.set_data(animalName + " died in smoke zone");
    deathPub.Publish(msg);
  }

  // Member variables
  std::vector<std::string> smokeNames;
  std::vector<std::string> wildlifeNames;
  std::vector<Entity> smokeEntities;
  std::vector<Entity> wildlifeEntities;
  std::unordered_set<Entity> deadEntities;
  std::unordered_map<Entity, std::string> wildlifeNameMap;

  double deadZoneRadius{6.0};
  bool entitiesResolved{false};

  gz::transport::Node node;
  gz::transport::Node::Publisher deathPub;
};

GZ_ADD_PLUGIN(DeadZonePlugin,
              System,
              DeadZonePlugin::ISystemConfigure,
              DeadZonePlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(DeadZonePlugin, "dead_zone_plugin")

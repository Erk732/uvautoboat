// Simple Gazebo (gz-sim) world plugin that marks wildlife as "dead" if they enter
// a smoke dead zone. On death: flips the animal belly-up and zeroes velocities.
// Build as part of your workspace and reference via filename="libdead_zone_plugin.so".

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>

#include <unordered_set>
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
                 EntityComponentManager &ecm,
                 EventManager &) override
  {
    if (sdf && sdf->HasElement("smoke_models"))
    {
      auto elem = sdf->GetElement("smoke_models");
      for (auto child = elem->GetFirstElement(); child;
           child = child->GetNextElement())
      {
        if (child->GetName() == "name")
        {
          smokeNames.push_back(child->Get<std::string>());
        }
      }
    }
    if (smokeNames.empty())
    {
      smokeNames = {"smoke_generator", "smoke_generator_east", "smoke_generator_north"};
    }

    if (sdf && sdf->HasElement("wildlife_models"))
    {
      auto elem = sdf->GetElement("wildlife_models");
      for (auto child = elem->GetFirstElement(); child;
           child = child->GetNextElement())
      {
        if (child->GetName() == "name")
        {
          wildlifeNames.push_back(child->Get<std::string>());
        }
      }
    }
    if (wildlifeNames.empty())
    {
      wildlifeNames = {"crocodile", "platypus", "turtle"};
    }

    if (sdf && sdf->HasElement("dead_zone_radius"))
    {
      deadZoneRadius = sdf->Get<double>("dead_zone_radius");
    }
    else
    {
      deadZoneRadius = 6.0;
    }

    // Resolve entity IDs from names.
    for (const auto &name : smokeNames)
    {
      auto ent = sim::entitiesFromScopedName(name, ecm, false);
      if (!ent.empty())
        smokeEntities.push_back(ent.front());
    }
    for (const auto &name : wildlifeNames)
    {
      auto ent = sim::entitiesFromScopedName(name, ecm, false);
      if (!ent.empty())
        wildlifeEntities.push_back(ent.front());
    }
  }

  void PreUpdate(const UpdateInfo &, EntityComponentManager &ecm) override
  {
    // Refresh poses for smoke entities
    std::vector<math::Vector3d> smokePositions;
    smokePositions.reserve(smokeEntities.size());
    for (auto ent : smokeEntities)
    {
      auto pose = ecm.Component<components::Pose>(ent);
      if (pose)
        smokePositions.push_back(pose->Data().Pos());
    }

    for (auto ent : wildlifeEntities)
    {
      if (deadEntities.count(ent))
        continue;

      auto poseComp = ecm.Component<components::Pose>(ent);
      if (!poseComp)
        continue;

      const auto pos = poseComp->Data().Pos();

      for (const auto &smokePos : smokePositions)
      {
        const double dist = pos.Distance(smokePos);
        if (dist <= deadZoneRadius)
        {
          MarkDead(ent, ecm);
          break;
        }
      }
    }
  }

private:
  void MarkDead(Entity ent, EntityComponentManager &ecm)
  {
    deadEntities.insert(ent);

    // Flip belly-up: roll 180 degrees, keep current position.
    auto poseComp = ecm.Component<components::Pose>(ent);
    if (poseComp)
    {
      auto pose = poseComp->Data();
      pose.Set(pose.Pos(), math::Quaterniond(GZ_PI, 0, 0));
      ecm.SetComponentData(ent, components::Pose(pose));
    }

    // Zero velocities to stop motion.
    ecm.SetComponentData(ent, components::LinearVelocity(math::Vector3d::Zero));
    ecm.SetComponentData(ent, components::AngularVelocity(math::Vector3d::Zero));
  }

private:
  std::vector<std::string> smokeNames;
  std::vector<std::string> wildlifeNames;
  std::vector<Entity> smokeEntities;
  std::vector<Entity> wildlifeEntities;
  std::unordered_set<Entity> deadEntities;
  double deadZoneRadius{6.0};
};

GZ_ADD_PLUGIN(DeadZonePlugin,
              System,
              DeadZonePlugin::ISystemConfigure,
              DeadZonePlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(DeadZonePlugin, "dead_zone_plugin")


// Enhanced Gazebo (gz-sim) world plugin that marks wildlife as "dead" if they enter
// a smoke dead zone. On death: flips the animal belly-up, slowly sinks, and spawns bubbles.
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
#include <gz/sim/components/ParticleEmitter.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/ParentEntity.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/particle_emitter.pb.h>
#include <gz/msgs/material.pb.h>
#include <gz/transport/Node.hh>

#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <string>
#include <chrono>

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
        }
      }
    }
    if (smokeNames.empty())
    {
      smokeNames = {"smoke_generator", "smoke_generator_east", "smoke_generator_north", "smoke_generator_wildlife"};
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

    // Parse sink speed (m/s)
    if (sdf && sdf->HasElement("sink_speed"))
    {
      sinkSpeed = sdf->Get<double>("sink_speed");
    }

    // Parse minimum depth (how far to sink)
    if (sdf && sdf->HasElement("sink_depth"))
    {
      sinkDepth = sdf->Get<double>("sink_depth");
    }

    // Resolve entity IDs from names
    for (const auto &name : smokeNames)
    {
      auto ents = sim::entitiesFromScopedName(name, ecm, false);
      if (!ents.empty())
        smokeEntities.push_back(*ents.begin());
    }
    for (const auto &name : wildlifeNames)
    {
      auto ents = sim::entitiesFromScopedName(name, ecm, false);
      if (!ents.empty())
      {
        wildlifeEntities.push_back(*ents.begin());
        wildlifeNameMap[*ents.begin()] = name;
      }
    }

    // Advertise death notification topic
    deathPub = node.Advertise<gz::msgs::StringMsg>("/vrx/wildlife/death");

    gzmsg << "[DeadZonePlugin] Initialized with " << smokeEntities.size()
          << " smoke zones and " << wildlifeEntities.size() << " wildlife entities.\n";
    gzmsg << "[DeadZonePlugin] Dead zone radius: " << deadZoneRadius << "m, "
          << "Sink speed: " << sinkSpeed << " m/s, Sink depth: " << sinkDepth << "m\n";
  }

  void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override
  {
    // Calculate dt in seconds
    double dt = std::chrono::duration<double>(info.dt).count();

    // Refresh poses for smoke entities
    std::vector<math::Vector3d> smokePositions;
    smokePositions.reserve(smokeEntities.size());
    for (auto ent : smokeEntities)
    {
      auto pose = ecm.Component<components::Pose>(ent);
      if (pose)
        smokePositions.push_back(pose->Data().Pos());
    }

    // Check living wildlife for entering dead zones
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
          MarkDead(ent, ecm, pos);
          break;
        }
      }
    }

    // Update sinking for dead entities
    for (auto ent : deadEntities)
    {
      auto poseComp = ecm.Component<components::Pose>(ent);
      if (!poseComp)
        continue;

      auto pose = poseComp->Data();
      double currentZ = pose.Pos().Z();

      // Only sink if above minimum depth
      if (currentZ > sinkDepth)
      {
        double newZ = currentZ - (sinkSpeed * dt);
        if (newZ < sinkDepth)
          newZ = sinkDepth;

        pose.Set(math::Vector3d(pose.Pos().X(), pose.Pos().Y(), newZ),
                 pose.Rot());
        ecm.SetComponentData<components::Pose>(ent, pose);
      }

      // Keep velocities zeroed (counteract buoyancy)
      ecm.SetComponentData<components::LinearVelocity>(ent, math::Vector3d::Zero);
      ecm.SetComponentData<components::AngularVelocity>(ent, math::Vector3d::Zero);
    }

    // Update bubble emitters (make them follow dead entities briefly then stop)
    UpdateBubbles(info, ecm);
  }

private:
  void MarkDead(Entity ent, EntityComponentManager &ecm, const math::Vector3d &deathPos)
  {
    deadEntities.insert(ent);
    deathTimes[ent] = std::chrono::steady_clock::now();
    deathPositions[ent] = deathPos;

    // Flip belly-up: roll 180 degrees
    auto poseComp = ecm.Component<components::Pose>(ent);
    if (poseComp)
    {
      auto pose = poseComp->Data();
      pose.Set(pose.Pos(), math::Quaterniond(GZ_PI, 0, 0));
      ecm.SetComponentData<components::Pose>(ent, pose);
    }

    // Zero velocities
    ecm.SetComponentData<components::LinearVelocity>(ent, math::Vector3d::Zero);
    ecm.SetComponentData<components::AngularVelocity>(ent, math::Vector3d::Zero);

    // Tint material to pale blue/grey to indicate death
    gz::msgs::Material mat;
    mat.mutable_ambient()->set_r(0.6);
    mat.mutable_ambient()->set_g(0.7);
    mat.mutable_ambient()->set_b(0.8);
    mat.mutable_ambient()->set_a(0.7);
    mat.mutable_diffuse()->set_r(0.6);
    mat.mutable_diffuse()->set_g(0.7);
    mat.mutable_diffuse()->set_b(0.8);
    mat.mutable_diffuse()->set_a(0.7);
    // Material component stores sdf::Material; we set only diffuse/ambient colors
    sdf::Material sdfMat;
    sdfMat.SetAmbient({mat.ambient().r(), mat.ambient().g(), mat.ambient().b(), mat.ambient().a()});
    sdfMat.SetDiffuse({mat.diffuse().r(), mat.diffuse().g(), mat.diffuse().b(), mat.diffuse().a()});
    ecm.SetComponentData<components::Material>(ent, sdfMat);

    // Get animal name for logging
    std::string animalName = "unknown";
    if (wildlifeNameMap.count(ent))
      animalName = wildlifeNameMap[ent];

    // Log death
    gzmsg << "[DeadZonePlugin] ☠️ " << animalName << " DIED at ("
          << deathPos.X() << ", " << deathPos.Y() << ", " << deathPos.Z()
          << ") - entering smoke zone!\n";

    // Publish death notification
    gz::msgs::StringMsg msg;
    msg.set_data(animalName + " died in smoke zone");
    deathPub.Publish(msg);

    // Spawn bubble emitter attached to this entity
    CreateBubbleEmitter(ent, ecm, deathPos);
  }

  void CreateBubbleEmitter(Entity ent, EntityComponentManager &ecm, const math::Vector3d &pos)
  {
    Entity emitterEnt = ecm.CreateEntity();
    ecm.CreateComponent(emitterEnt, components::Name("bubbles_" + std::to_string(emitterEnt)));
    ecm.CreateComponent(emitterEnt, components::ParentEntity(ent));

    gz::msgs::ParticleEmitter emitter;
    emitter.set_type(gz::msgs::ParticleEmitter::BOX);
    emitter.mutable_emitting()->set_data(true);
    emitter.mutable_rate()->set_data(120.0);
    emitter.mutable_duration()->set_data(4.0);
    emitter.mutable_lifetime()->set_data(2.5);
    auto *size = emitter.mutable_size();
    size->set_x(0.8);
    size->set_y(0.8);
    size->set_z(0.8);
    auto *pSize = emitter.mutable_particle_size();
    pSize->set_x(0.05);
    pSize->set_y(0.05);
    pSize->set_z(0.05);
    emitter.mutable_color_start()->set_r(0.8);
    emitter.mutable_color_start()->set_g(0.9);
    emitter.mutable_color_start()->set_b(1.0);
    emitter.mutable_color_start()->set_a(0.8);
    emitter.mutable_color_end()->set_r(0.8);
    emitter.mutable_color_end()->set_g(0.9);
    emitter.mutable_color_end()->set_b(1.0);
    emitter.mutable_color_end()->set_a(0.0);
    auto *pose = emitter.mutable_pose();
    pose->mutable_position()->set_x(0.0);
    pose->mutable_position()->set_y(0.0);
    pose->mutable_position()->set_z(0.2);

    ecm.CreateComponent(emitterEnt, components::ParticleEmitter(emitter));

    bubbleEmitters[ent] = emitterEnt;
  }

  void UpdateBubbles(const UpdateInfo &info, EntityComponentManager &ecm)
  {
    auto now = std::chrono::steady_clock::now();

    for (auto &[deadEnt, emitterEnt] : bubbleEmitters)
    {
      if (!deathTimes.count(deadEnt))
        continue;

      auto elapsed = std::chrono::duration<double>(now - deathTimes[deadEnt]).count();

      // Bubbles emit for 5 seconds after death
      if (elapsed > 5.0)
      {
        // Could disable emitter here if needed
        continue;
      }
    }
  }

private:
  std::vector<std::string> smokeNames;
  std::vector<std::string> wildlifeNames;
  std::vector<Entity> smokeEntities;
  std::vector<Entity> wildlifeEntities;
  std::unordered_set<Entity> deadEntities;
  std::unordered_map<Entity, std::string> wildlifeNameMap;
  std::unordered_map<Entity, std::chrono::steady_clock::time_point> deathTimes;
  std::unordered_map<Entity, math::Vector3d> deathPositions;
  std::unordered_map<Entity, Entity> bubbleEmitters;

  double deadZoneRadius{6.0};
  double sinkSpeed{0.15};      // meters per second
  double sinkDepth{-2.0};      // sink to 2m below water surface

  gz::transport::Node node;
  gz::transport::Node::Publisher deathPub;
};

GZ_ADD_PLUGIN(DeadZonePlugin,
              System,
              DeadZonePlugin::ISystemConfigure,
              DeadZonePlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(DeadZonePlugin, "dead_zone_plugin")

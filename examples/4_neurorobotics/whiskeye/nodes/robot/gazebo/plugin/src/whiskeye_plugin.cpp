// Copyright (c) 2023 Lee Perry

#include "whiskeye_plugin.hpp"

#include <stdexcept>

#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(
    whiskeye_plugin::WhiskeyePlugin,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate,
    gz::sim::ISystemPostUpdate)

namespace
{
    auto World(gz::sim::EntityComponentManager& ecm)
    {
        return ecm.EntityByComponents(gz::sim::components::World());
    }

    auto Gravity(gz::sim::EntityComponentManager& ecm)
    {
        return ecm.Component<gz::sim::components::Gravity>(World(ecm));
    }

    auto EntityName(
        gz::sim::EntityComponentManager& ecm,
        const gz::sim::Entity entity)
    {
        using Name = gz::sim::components::Name;
        return ecm.Component<Name>(entity)->Data();
    }

    template<typename SensorType = gz::sim::components::Sensor>
    auto SensorByName(
        gz::sim::EntityComponentManager& ecm,
        const std::string& name)
    {
        const auto sensor = ecm.EntityByComponents(
            gz::sim::components::Name(name),
            SensorType());
        gzdbg << "Sensor " << name << " [" << sensor << "]" << std::endl;
        return sensor;
    }

    auto JointByName(
        gz::sim::EntityComponentManager& ecm,
        const std::string& name)
    {
        const auto joint = ecm.EntityByComponents(
            gz::sim::components::Name(name),
            gz::sim::components::Joint());
        gzdbg << "Joint " << name << " [" << joint << "]" << std::endl;
        return joint;
    }

    // https://github.com/gazebosim/gz-sim/blob/gz-sim7/examples/plugin/custom_sensor_system/OdometerSystem.cc
    auto LoadWhiskers(gz::sim::EntityComponentManager& ecm)
    {
        auto whiskers = std::vector<whiskeye_plugin::Whisker>();

        for (const auto link : ecm.EntitiesByComponents(
                                    gz::sim::components::Link()))
        {
            auto whisker = whiskeye_plugin::Whisker{};
            whisker.name = EntityName(ecm, link);
            whisker.link = link;

            if (whisker.name.rfind("whisker", 0) == 0)
            {
                const auto sensor_name = whisker.name + "_contact";
                whisker.sensor = SensorByName(ecm, sensor_name);
                const auto joint_name = "head_" + whisker.name;
                whisker.joint  = JointByName(ecm, joint_name);
                gzdbg << "Whisker "
                      <<   "name [" << whisker.name   << "], "
                      <<   "link [" << whisker.link   << "], "
                      << "sensor [" << whisker.sensor << "], "
                      <<  "joint [" << whisker.joint  << "]" << std::endl;
                whiskers.push_back(whisker);
            }
        }

        if (whiskers.empty()) gzerr << "No whiskers found!" << std::endl;
        return whiskers;
    }

    auto LoadImu(
        gz::sim::EntityComponentManager& ecm,
        gz::sensors::SensorFactory& sensorFactory)
    {
        using ImuPtr = std::unique_ptr<gz::sensors::ImuSensor>;
        std::map<gz::sim::Entity, ImuPtr> imus;
        const auto gravity = Gravity(ecm);

        ecm.Each<gz::sim::components::Imu,
                 gz::sim::components::ParentEntity>(
            [&](const gz::sim::Entity& entity,
                const gz::sim::components::Imu* imu,
                const gz::sim::components::ParentEntity*) -> bool
            {
                auto data = imu->Data();
                const auto name = EntityName(ecm, entity);
                data.SetName(name);
                auto sensor = sensorFactory
                    .CreateSensor<gz::sensors::ImuSensor>(data);
                sensor->SetGravity(gravity->Data());
                sensor->SetOrientationReference(worldPose(entity, ecm).Rot());
                gzdbg << "IMU "
                      <<   "name [" << name   << "], "
                      << "entity [" << entity << "]" << std::endl;
                imus.insert({entity, std::move(sensor)});
                return true;
            });

        if (imus.empty()) gzerr << "No IMUs found!" << std::endl;
        return imus;
    }
}

namespace whiskeye_plugin
{
    void WhiskeyePlugin::Configure(
        const gz::sim::Entity& model,
        const std::shared_ptr<const sdf::Element>& /*sdf*/,
        gz::sim::EntityComponentManager& ecm,
        gz::sim::EventManager& /*eventMgr*/)
    {
        m_robot = model;
        const auto robotName = EntityName(ecm, m_robot);

        gzmsg << "Configuring plugin for "
              << robotName << " [" << m_robot << "]" << std::endl;

        m_robotPoseX     = JointByName(ecm, "pose_x");
        m_robotPoseY     = JointByName(ecm, "pose_y");
        m_robotPoseTheta = JointByName(ecm, "pose_theta");
        m_whiskers       = LoadWhiskers(ecm);
        m_imus           = LoadImu(ecm, m_sensorFactory);

        // Create ROS2 node
        m_node = rclcpp::Node::make_shared(robotName);
        m_executor.add_node(m_node);

        const auto QUEUE_SIZE = 1;
        m_subscriber = m_node->create_subscription<msg::BrainToBody>(
            "/whiskeye/brain_to_body",
            QUEUE_SIZE,
            [this](const msg::BrainToBody::SharedPtr msg)
            { m_brainToBody = msg; });
        m_publisher = m_node->create_publisher<msg::BodyToBrain>(
            "/whiskeye/brain_to_body",
            QUEUE_SIZE);
    }

    void WhiskeyePlugin::PreUpdate(
        const gz::sim::UpdateInfo& /*info*/,
        gz::sim::EntityComponentManager& ecm)
    {
        // Wait for inputs from NeuROS
        while (m_brainToBody == nullptr)
        {
            m_executor.spin_once(std::chrono::seconds(0));
        }

        using Velocity = gz::sim::components::JointVelocityCmd;

        const auto& position = m_brainToBody->position_command.data;
        ecm.SetComponentData<Velocity>(m_robotPoseX, {position[0]});
        ecm.SetComponentData<Velocity>(m_robotPoseY, {position[1]});

        const auto& rotation = m_brainToBody->rotation_command.data;
        ecm.SetComponentData<Velocity>(m_robotPoseTheta, {rotation[2]});

        m_brainToBody = nullptr;
    }

    void WhiskeyePlugin::PostUpdate(
        const gz::sim::UpdateInfo& /*info*/,
        const gz::sim::EntityComponentManager& ecm)
    {
        auto bodyToBrain = msg::BodyToBrain();

        const auto pose = worldPose(m_robot, ecm);

        const auto& pos = pose.Pos();
        bodyToBrain.position.data = {pos.X(), pos.Y(), pos.Z()};

        const auto& rot = pose.Rot();
        bodyToBrain.rotation.data = {rot.Roll(), rot.Pitch(), rot.Yaw()};

        // TODO
        // * Camera 0, 1, 2

        ecm.Each<gz::sim::components::Imu,
                 gz::sim::components::WorldPose,
                 gz::sim::components::AngularVelocity,
                 gz::sim::components::LinearAcceleration>(
            [this](const gz::sim::Entity& entity,
                   const gz::sim::components::Imu*,
                   const gz::sim::components::WorldPose* pose,
                   const gz::sim::components::AngularVelocity* ang,
                   const gz::sim::components::LinearAcceleration* lin) -> bool
            {
                auto imu = m_imus.find(entity);
                if (imu != m_imus.end())
                {
                    imu->second->SetWorldPose(pose->Data());
                    imu->second->SetAngularVelocity(ang->Data());
                    imu->second->SetLinearAcceleration(lin->Data());
                }
                return true;
            });

        // Send outputs to NeuROS
        m_publisher->publish(bodyToBrain);
    }
}

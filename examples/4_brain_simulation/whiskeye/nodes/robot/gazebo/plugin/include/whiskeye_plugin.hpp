// Copyright (c) 2023 Lee Perry

#pragma once

#include "visibility_control.h"

#include <memory>
#include <optional>
#include <string>

#include <gz/math.hh>
#include <gz/physics.hh>
#include <gz/sensors.hh>
#include <gz/sim.hh>
#include <gz/utils.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <whiskeye_plugin/msg/body_to_brain.hpp>
#include <whiskeye_plugin/msg/brain_to_body.hpp>

namespace whiskeye_plugin
{
    struct Whisker
    {
        std::string     name;
        gz::sim::Entity link;
        gz::sim::Entity sensor;
        gz::sim::Entity joint;
    };

    class WhiskeyePlugin:
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate,
        public gz::sim::ISystemPostUpdate
    {
    public:

        void Configure(
            const gz::sim::Entity& model,
            const std::shared_ptr<const sdf::Element>& sdf,
            gz::sim::EntityComponentManager& ecm,
            gz::sim::EventManager& eventMgr);

        void PreUpdate(
            const gz::sim::UpdateInfo& info,
            gz::sim::EntityComponentManager& ecm);

        void PostUpdate(
            const gz::sim::UpdateInfo& info,
            const gz::sim::EntityComponentManager& ecm);

    private:

        using ImuPtr = std::unique_ptr<gz::sensors::ImuSensor>;

        gz::sim::Entity m_robot;
        gz::sim::Entity m_robotPoseX;
        gz::sim::Entity m_robotPoseY;
        gz::sim::Entity m_robotPoseTheta;
        gz::sensors::SensorFactory m_sensorFactory;
        std::vector<Whisker> m_whiskers;
        std::map<gz::sim::Entity, ImuPtr> m_imus;

        std::shared_ptr<rclcpp::Node> m_node;
        rclcpp::executors::SingleThreadedExecutor m_executor;
        rclcpp::Subscription<msg::BrainToBody>::SharedPtr m_subscriber;
        rclcpp::Publisher<msg::BodyToBrain>::SharedPtr m_publisher;
        msg::BrainToBody::SharedPtr m_brainToBody;
    };
}

// Copyright (c) 2023 Lee Perry

#pragma once

#include "whiskeye_plugin/visibility_control.h"

#include <memory>
#include <string>

#include <gz/common5/gz/common.hh>
#include <gz/math7/gz/math.hh>
#include <gz/physics6/gz/physics.hh>
#include <gz/sim7/gz/sim.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <whiskeye_plugin/msg/bridge_u.hpp>

namespace whiskeye_plugin
{
    struct Whisker
    {
        std::string     name;
        gz::sim::Entity link;
        gz::sim::Entity sensor;
        gz::sim::Entity joint;
    };

    struct Camera
    {
        std::string     name;
        gz::sim::Entity sensor;
    };

    class WhiskeyePlugin:
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate,
        public gz::sim::ISystemPostUpdate
    {
    public:

        WhiskeyePlugin();
        virtual ~WhiskeyePlugin() = default;

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

        gz::sim::Entity      m_robot;

        std::vector<Whisker> m_whiskers;
        std::vector<Camera>  m_cameras;
        gz::sim::Entity      m_imu;

        gz::sim::Entity      m_robot_pose_x;
        gz::sim::Entity      m_robot_pose_y;
        gz::sim::Entity      m_robot_pose_theta;

        gz::sim::Entity      m_body_neck_joint;
        gz::sim::Entity      m_neck_gimbal_joint;
        gz::sim::Entity      m_gimbal_head_joint;

        gz::math::PID        m_robotPose_x_pid;
    };
}

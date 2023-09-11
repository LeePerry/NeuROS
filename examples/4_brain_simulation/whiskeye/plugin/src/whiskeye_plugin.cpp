// Copyright (c) 2023 Lee Perry

#include "whiskeye_plugin/whiskeye_plugin.hpp"

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
    auto entity_name(
        gz::sim::EntityComponentManager& ecm,
        const gz::sim::Entity entity)
    {
        using Name = gz::sim::components::Name;
        return ecm.Component<Name>(entity)->Data();
    }

    template<typename SensorType = gz::sim::components::Sensor>
    auto sensor_by_name(
        gz::sim::EntityComponentManager& ecm,
        const std::string& name)
    {
        const auto sensor = ecm.EntityByComponents(
            gz::sim::components::Name(name),
            SensorType());
        gzdbg << "Sensor " << name << ": " << sensor << std::endl;
        return sensor;
    }

    auto joint_by_name(
        gz::sim::EntityComponentManager& ecm,
        const std::string& name)
    {
        const auto joint = ecm.EntityByComponents(
            gz::sim::components::Name(name),
            gz::sim::components::Joint());
        gzdbg << "Joint " << name << ": " << joint << std::endl;
        return joint;
    }

    auto load_whiskers(gz::sim::EntityComponentManager& ecm)
    {
        auto whiskers = std::vector<whiskeye_plugin::Whisker>();

        for (const auto link : ecm.EntitiesByComponents(
                                    gz::sim::components::Link()))
        {
            auto whisker = whiskeye_plugin::Whisker{};
            whisker.name = entity_name(ecm, link);
            whisker.link = link;

            if (whisker.name.rfind("whisker", 0) == 0)
            {
                const auto sensor_name = whisker.name + "_contact";
                whisker.sensor = sensor_by_name(ecm, sensor_name);
                const auto joint_name = "head_" + whisker.name;
                whisker.joint  = joint_by_name(ecm, joint_name);
                gzdbg << "Whisker name: "   << whisker.name   << std::endl;
                gzdbg << "Whisker link: "   << whisker.link   << std::endl;
                gzdbg << "Whisker sensor: " << whisker.sensor << std::endl;
                gzdbg << "Whisker joint: "  << whisker.joint  << std::endl;
                whiskers.push_back(whisker);
            }
        }

        if (whiskers.empty()) gzerr << "No whiskers found!" << std::endl;
        return whiskers;
    }

    auto load_cameras(gz::sim::EntityComponentManager& ecm)
    {
        auto cameras = std::vector<whiskeye_plugin::Camera>();

        for (const auto entity : ecm.EntitiesByComponents(
                                    gz::sim::components::Sensor()))
        {
            auto camera   = whiskeye_plugin::Camera{};
            camera.name   = entity_name(ecm, entity);
            camera.sensor = entity;

            if (camera.name.rfind("cam", 0) == 0)
            {
                gzdbg << "Camera name: "   << camera.name   << std::endl;
                gzdbg << "Camera sensor: " << camera.sensor << std::endl;
                cameras.push_back(camera);
            }
        }

        if (cameras.empty()) gzerr << "No cameras found!" << std::endl;
        return cameras;
    }

    auto load_imu(gz::sim::EntityComponentManager& ecm)
    {
        const auto imu = sensor_by_name(ecm, "imu_body");
        gzdbg << "IMU sensor: " << imu << std::endl;
        return imu;
    }
}

namespace whiskeye_plugin
{
    WhiskeyePlugin::WhiskeyePlugin()
    {
        if (! rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
    }

    void WhiskeyePlugin::Configure(
        const gz::sim::Entity& model,
        const std::shared_ptr<const sdf::Element>& /*sdf*/,
        gz::sim::EntityComponentManager& ecm,
        gz::sim::EventManager& /*eventMgr*/)
    {
        m_robot = model;
        gzmsg << "Configuring WhiskeyePlugin with model: "
              << entity_name(ecm, model) << ": " << model << std::endl;

        m_whiskers          = load_whiskers(ecm);
        m_cameras           = load_cameras(ecm);
        m_imu               = load_imu(ecm);
        m_robot_pose_x      = joint_by_name(ecm, "pose_x");
        m_robot_pose_y      = joint_by_name(ecm, "pose_y");
        m_robot_pose_theta  = joint_by_name(ecm, "pose_theta");
        m_body_neck_joint   = joint_by_name(ecm, "body_neck");
        m_neck_gimbal_joint = joint_by_name(ecm, "neck_gmbl");
        m_gimbal_head_joint = joint_by_name(ecm, "gmbl_head");

        // make the robot start moving forwards!
        // ls /usr/include/gz/sim7/gz/sim/components
        //ecm.CreateComponent(m_robot_pose_x, gz::sim::components::JointForceCmd({ 1.0 }));

        // JointForceCmd       - works applied once at start
        // JointPosition       - does nothing?
        // JointVelocityCmd    - works if applied continuously (in PreUpdate)
        // ...

        // m_robotPose_x_pid

        ecm.CreateComponent(m_robot_pose_x, gz::sim::components::JointVelocityCmd({ 1.0 }));


        //    initialise their controllers

        /*
        SetPositionPID(state.joints_neck[0].name, ROBOT_NECK_PID_P_0, ROBOT_NECK_PID_D);
        SetPositionPID(state.joints_neck[1].name, ROBOT_NECK_PID_P_1, ROBOT_NECK_PID_D);
        SetPositionPID(state.joints_neck[2].name, ROBOT_NECK_PID_P_2, ROBOT_NECK_PID_D);

        //    find whisker joints
        for (int row=0; row<ROBOT_ROW_COUNT; row++)
        {
            for (int col=0; col<ROBOT_COL_COUNT; col++)
            {
                stringstream ss;
                ss << "head_whisker" << (row+1) << "_" << (col+1);
                state.joints_whisker[row][col] = GetJoint(ss.str());
                SetPositionPID(state.joints_whisker[row][col].name, ROBOT_WHISKER_PID_P, ROBOT_WHISKER_PID_D);
            }
        }

////    WHISKER MAPPING

        // Having mapped contacts into the initial FOR for each whisker, we then need
        // to map them into some canonical FOR. We choose an FOR pointing directly up
        // in +z, since it's intuitive and also since then +x and +y map directly to
        // the x/y outputs we ultimately need to generate. This mapping is implicit in
        // the STLs that we start with, since each whisker's pose at start-up is encoded
        // in its actual position there. We could in principle measure it here, but it's
        // somewhat easier to measure it during model creation and pass it in here using
        // a header file. In fact, since the API available here is good at handling poses
        // and the external file has access to the rotaxe objects, we'll get the model
        // creation process to recover three reference points (joint centre, joint axis,
        // whisker tip) and then use the API available here to convert that to a pose
        // that will map each whisker's physical FOR onto the canonical whisker FOR.

        //    read whisker_pose
        if (sizeof(whisker_pose) != (ROBOT_WHISKER_COUNT * 9 * sizeof(double)))
            __ERROR("bad whisker_pose array size");
        const JointPose* joint_pose = (const JointPose*) whisker_pose;
        for (int r=0; r<ROBOT_ROW_COUNT; r++)
        {
            for (int c=0; c<ROBOT_COL_COUNT; c++)
            {
                //    find transform that brings this initial whisker
                //    pose back to the canonical pose
                WORLD_POSE T = get_pose_transform(joint_pose);

                //    store
                state.inverse_initial_whisker_pose[r][c] = T;

                //    advance
                joint_pose++;
            }
        }

        //    read neck_pose
        if (sizeof(neck_pose) != (3 * 9 * sizeof(double)))
            __ERROR("bad neck_pose array size");
        joint_pose = (const JointPose*) neck_pose;
        for (int i=0; i<3; i++)
        {
            //    find transform that brings this initial whisker
            //    pose back to the canonical pose
            WORLD_POSE T = get_pose_transform_2(joint_pose);

            //    store
            state.inverse_initial_neck_pose[i] = T;

            //    advance
            joint_pose++;
        }

////    INTERFACE

        //    publish
        string topic_root = "/whiskeye";
        image_transport::ImageTransport it(*output.h_ros);
#define __ADVERTISE(field, type, name) \
output.field.pub = output.h_ros->advertise<type> \
    ((topic_root + name).c_str(), ROS_SEND_QUEUE_SIZE);
#define __IT_ADVERTISE(field, name) \
output.field.pub = it.advertise \
    ((topic_root + name).c_str(), ROS_SEND_QUEUE_SIZE);
        __ADVERTISE(bumper, std_msgs::Bool, "/body/bumper");
        __ADVERTISE(imu_body, sensor_msgs::Imu, "/body/imu_body");
        __ADVERTISE(pose, geometry_msgs::Pose2D, "/body/pose");
        __ADVERTISE(bridge_u, whiskeye_plugin::Bridge_u, "/head/bridge_u");
        __ADVERTISE(xy, std_msgs::Float32MultiArray, "/head/xy");
        __ADVERTISE(contact_head, std_msgs::Float32MultiArray, "/head/contact_head");
        __ADVERTISE(contact_world, std_msgs::Float32MultiArray, "/head/contact_world");
        __IT_ADVERTISE(cam[0], "/head/cam0/image_raw");
        __IT_ADVERTISE(cam[1], "/head/cam1/image_raw");
        __IT_ADVERTISE(cam[2], "/head/cam2/image_raw");

        //    subscribe
        string topic;

        //    to neck
        topic = topic_root + "/head/neck_cmd";
        cout << "subscribe: " << topic << endl;
        input.neck.sub = output.h_ros->subscribe(
            topic.c_str(),
            ROS_RECV_QUEUE_SIZE,
            &Whiskeye_ModelPlugin::callback_neck,
            this
            );

        //    to theta
        topic = topic_root + "/head/theta_cmd";
        cout << "subscribe: " << topic << endl;
        input.theta.sub = output.h_ros->subscribe(
            topic.c_str(),
            ROS_RECV_QUEUE_SIZE,
            &Whiskeye_ModelPlugin::callback_theta,
            this
            );

        //    to cmd_vel
        topic = topic_root + "/body/cmd_vel";
        cout << "subscribe: " << topic << endl;
        input.cmd_vel.sub = output.h_ros->subscribe(
            topic.c_str(),
            ROS_RECV_QUEUE_SIZE,
            &Whiskeye_ModelPlugin::callback_cmd_vel,
            this
            );

        //    to cmd_pos
        topic = topic_root + "/body/cmd_pos";
        cout << "subscribe: " << topic << endl;
        input.cmd_pos.sub = output.h_ros->subscribe(
            topic.c_str(),
            ROS_RECV_QUEUE_SIZE,
            &Whiskeye_ModelPlugin::callback_cmd_pos,
            this
            );*/

    }

    void WhiskeyePlugin::PreUpdate(
        const gz::sim::UpdateInfo& /*info*/,
        gz::sim::EntityComponentManager& /*ecm*/)
    {
        //ecm.CreateComponent(m_robot_pose_x, gz::sim::components::JointVelocityCmd({ 1.0 }));
    }

    void WhiskeyePlugin::PostUpdate(
        const gz::sim::UpdateInfo& /*info*/,
        const gz::sim::EntityComponentManager& ecm)
    {
        gzmsg << "m_robot::pose = "
              << ecm.Component<gz::sim::components::Pose>(m_robot)->Data()
              << std::endl;
    }
}

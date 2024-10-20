//
// Created by cilotta on 24-10-18.
//

#pragma once

#include "rm_gimbal_controllers/gimbal_base.h"

#include <effort_controllers/joint_velocity_controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/filters/filters.h>
#include <rm_msgs/GimbalCmd.h>
#include <rm_msgs/TrackData.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/GimbalPosState.h>
#include <rm_gimbal_controllers/StackYawGimbalConfig.h>
#include <rm_gimbal_controllers/bullet_solver.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Eigen>
#include <control_toolbox/pid.h>
#include <urdf/model.h>
#include <dynamic_reconfigure/server.h>
#include <realtime_tools/realtime_publisher.h>

namespace rm_gimbal_controllers
{
struct StackYawConfig
{
    double pitch_k_v_, sec_yaw_k_v_, yaw_k_v_, k_chassis_vel_;
    double accel_pitch_{}, accel_sec_yaw_{}, accel_yaw_{};
};

class StackYawController : public Controller
{
public:
    StackYawController() = default;
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void setDes(const ros::Time& time, double pitch_des, double sec_yaw_des, double yaw_des);
protected:
    void rate(const ros::Time& time, const ros::Duration& period) override;
    void track(const ros::Time& time) override;
    void direct(const ros::Time& time) override;

    void moveJoint(const ros::Time& time, const ros::Duration& period) override;
    void commandCB(const rm_msgs::GimbalCmdConstPtr& msg) override;
    void trackCB(const rm_msgs::TrackDataConstPtr& msg) override;
    void reconfigCB(StackYawGimbalConfig& config, uint32_t);

    rm_control::RobotStateHandle robot_state_handle_;
    hardware_interface::ImuSensorHandle imu_sensor_handle_;
    bool has_imu_ = true;
    effort_controllers::JointVelocityController ctrl_pitch_, ctrl_sec_yaw_, ctrl_yaw_;
    control_toolbox::Pid pid_pitch_pos_, pid_sec_yaw_pos_, pid_yaw_pos_;

    std::shared_ptr<BulletSolver> bullet_solver_;

    // ROS Interface
    ros::Time last_publish_time_{};
    std::unique_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalPosState>> pitch_pos_state_pub_, sec_yaw_pos_state_pub_, yaw_pos_state_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError>> error_pub_;
    ros::Subscriber cmd_gimbal_sub_;
    ros::Subscriber data_track_sub_;
    realtime_tools::RealtimeBuffer<rm_msgs::GimbalCmd> cmd_rt_buffer_;
    realtime_tools::RealtimeBuffer<rm_msgs::TrackData> track_rt_buffer_;
    urdf::JointConstSharedPtr pitch_joint_urdf_, sec_yaw_joint_urdf_, yaw_joint_urdf_;

    rm_msgs::GimbalCmd cmd_gimbal_;
    rm_msgs::TrackData data_track_;
    std::string gimbal_des_frame_id_{}, imu_name_{};
    double publish_rate_{};
    bool state_changed_{};
    bool pitch_des_in_limit_{}, sec_yaw_des_in_limit_{}, yaw_des_in_limit_{};
    int loop_count_{};
    // Transform
    geometry_msgs::TransformStamped odom2gimbal_des_, odom2pitch_, odom2sec_yaw_, odom2base_, last_odom2base_;

    // Gravity Compensation
    geometry_msgs::Vector3 mass_origin_;
    double gravity_;
    bool enable_gravity_compensation_;
    // Chassis
    std::shared_ptr<ChassisVel> chassis_vel_;

    bool dynamic_reconfig_initialized_{};
    StackYawConfig config_{};
    realtime_tools::RealtimeBuffer<StackYawConfig> config_rt_buffer_;
    dynamic_reconfigure::Server<StackYawGimbalConfig>* d_srv_{};

    RampFilter<double>*ramp_rate_pitch_{}, *ramp_rate_sec_yaw_{}, *ramp_rate_yaw_{};
    enum
    {
        RATE,
        TRACK,
        DIRECT
    };
    int state_ = RATE;
};

}

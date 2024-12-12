//
// Created by ham on 2024/11/27.
//

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/Twist.h>
#include <control_toolbox/pid.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace hero_chassis_controller {
class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {


public:
//  构造函数
    HeroChassisController() = default;

//  析枸函数
    ~HeroChassisController() override = default;

    bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                    ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

    void update(const ros::Time& time, const ros::Duration& period) override;

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;

    control_toolbox::Pid front_left_pid_, front_right_pid_, back_left_pid_, back_right_pid_;

    ros::Subscriber joint_state_sub;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster odom_broadcaster_;  //  创建tf广播器
    tf::TransformListener listener_;

private:
    std::string frame_;
    ros::Time current_time_ = ros::Time::now();
    ros::Time last_time_ = ros::Time::now();
    ros::Duration period_;
    double x_ = -0.003691;  // 小车base坐标轴原点相对于odom的位置
    double y_ = -0.001885;
    double th_ = 0.0;
    double wheel_radius_;  // 车轮半径
    double track_;  // 底盘宽度
    double wheelbase_;  // 底盘长度
    double back_left_vel_=0.0;
    double front_left_vel_=0.0;
    double back_right_vel_=0.0;
    double front_right_vel_=0.0;
    double desired_vx_ = 0.0;
    double desired_vy_ = 0.0;
    double desired_omega_ = 0.0;

    };
}


#endif //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

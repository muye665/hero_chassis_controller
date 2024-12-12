//
// Created by ham on 2024/11/27.
//
#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <control_toolbox/pid.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

namespace hero_chassis_controller {
    bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                       ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
//        获取车辆参数
        if (!controller_nh.getParam("wheel_radius", wheel_radius_) ||
            !controller_nh.getParam("track", track_) ||
            !controller_nh.getParam("wheelbase", wheelbase_)) {
            ROS_ERROR("Failed to load wheel parameters.");
            return false;
        }

        if(!root_nh.getParam("controller/hero_chassis_controller/frame", frame_)){
            ROS_ERROR("Cannot get frame id, please check your yaml file");
            return false;
        }

//        获取每个轮子的句柄
        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

//        初始化 PID 控制器
        if (!front_left_pid_.init(ros::NodeHandle(controller_nh, "front_left_pid")) ||
            !front_right_pid_.init(ros::NodeHandle(controller_nh, "front_right_pid")) ||
            !back_left_pid_.init(ros::NodeHandle(controller_nh, "back_left_pid")) ||
            !back_right_pid_.init(ros::NodeHandle(controller_nh, "back_right_pid"))) {
//            PID初始化失败
            ROS_ERROR("Failed to initialize PID controllers.");
            return false;
        }

//        订阅当前速度
        joint_state_sub = root_nh.subscribe("/joint_states", 10, &HeroChassisController::jointStateCallback, this);
        // 订阅动作指令话题
        cmd_vel_sub_ = root_nh.subscribe("/cmd_vel", 10, &HeroChassisController::cmdVelCallback, this);
//        创建里程发布的publisher
        odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("odom", 50);

        // 初始化化时间
        return true;
    }

    void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {

        current_time_ = time;
        period_ = period;

//        如果超过 0.5 秒未收到新的 /cmd_vel 消息，则停止运动
//        double timeout = 0.5;
//        if ((current_time_ - last_cmd_time_).toSec() > timeout) {
//            desired_vx_ = 0.0;
//            desired_vy_ = 0.0;
//            desired_omega_ = 0.0;
//            last_cmd_time_ = time;
//        }

//  ==========================================================================
//        更新底盘整体目标速度，更新四个麦轮当前‘角速度’
//  ==========================================================================

        // 获得底盘整体的目标速度
        double vx = desired_vx_;
        double vy = desired_vy_;
        double omega = desired_omega_;

//        ROS_INFO("target vel %f %f %f", vx, vy, omega);

//      更新麦轮当前实际‘角速度’
        double now_back_left_vel_ = back_left_vel_;
        double now_front_left_vel_ = front_left_vel_;
        double now_back_right_vel_ = back_right_vel_;
        double now_front_right_vel_ = front_right_vel_;

//  ==========================================================================
//        正向里程计，根据麦轮当前速度解出底盘整体速度
//  ==========================================================================

//        使用正向运动学计算小车当前的整体速度
        double now_v_x = wheel_radius_ *
                         (now_front_left_vel_ + now_front_right_vel_ + now_back_left_vel_ + now_back_right_vel_) / 4;
        double now_v_y = wheel_radius_ *
                         (-now_front_left_vel_ + now_front_right_vel_ + now_back_left_vel_ - now_back_right_vel_) / 4;
        double now_omega = wheel_radius_ *
                           (-now_front_left_vel_ + now_front_right_vel_ - now_back_left_vel_ + now_back_right_vel_) /
                           (2 * (track_ + wheelbase_));

        double dt = period_.toSec();
        double delta_x = (now_v_x * cos(th_) - now_v_y * sin(th_)) * dt;
        double delta_y = (now_v_x * sin(th_) + now_v_y * cos(th_)) * dt;
        double delta_th = now_omega * dt;

        x_ += delta_x;
        y_ += delta_y;
        th_ += delta_th;
        ROS_INFO("x, y, th dt = %f %f %f %f ", x_, y_, th_, dt);

//      四元数定义旋转
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);
        geometry_msgs::TransformStamped odom_trans;

//        描述两个坐标之间的转换关系
        odom_trans.header.stamp = current_time_;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
//      发送转换关系
        odom_broadcaster_.sendTransform(odom_trans);

//        创建里程数据并发以发布到 topic "/odom"
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time_;
        odom.header.frame_id = "odom";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = now_v_x;
        odom.twist.twist.linear.y = now_v_y;
        odom.twist.twist.angular.z = now_omega;

        //publish the message
        odom_pub_.publish(odom);

//  ===========================================================================
//        逆运动学得到四轮目标速度，使用PID计算出控制命令，并发送力矩指令
//  ===========================================================================

//       ----------------------------------------------------
//               将目标速度转换为base_link坐标系下
//       ----------------------------------------------------
        if(frame_ == "odom"){
            geometry_msgs::Vector3Stamped msg;
            geometry_msgs::Vector3Stamped out;
            msg.header.frame_id = frame_;
            msg.header.stamp = ros::Time();
            msg.vector.x = vx;
            msg.vector.y = vy;
            msg.vector.z = 0.0;

            try{
                listener_.transformVector("base_link", msg, out);
//                listener_.transformVector("base_link", ros::Time(current_time_), msg, "odom", out);
//                ROS_INFO("msg is :%f %f %f", msg.vector.x, msg.vector.y, msg.vector.z);
//                ROS_INFO("out is :%f %f %f", out.vector.x, out.vector.y, out.vector.z);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            vx = out.vector.x;
            vy = out.vector.y;
        }

//        逆运动学
        double front_left_target = (vx - vy - (track_ / 2 + wheelbase_ / 2) * omega) / wheel_radius_;
        double front_right_target = (vx + vy + (track_ / 2 + wheelbase_ / 2) * omega) / wheel_radius_;
        double back_left_target = (vx + vy - (track_ / 2 + wheelbase_ / 2) * omega) / wheel_radius_;
        double back_right_target = (vx - vy + (track_ / 2 + wheelbase_ / 2) * omega) / wheel_radius_;

        // 计算 PID 控制命令
        double front_left_cmd = front_left_pid_.computeCommand(front_left_target - now_front_left_vel_, period);
        double front_right_cmd = front_right_pid_.computeCommand(front_right_target - now_front_right_vel_, period);
        double back_left_cmd = back_left_pid_.computeCommand(back_left_target - now_back_left_vel_, period);
        double back_right_cmd = back_right_pid_.computeCommand(back_right_target - now_back_right_vel_, period);

        // 设置轮子的速度命令
//        ROS_INFO("final set cmd: %f %f %f %f", front_left_cmd, front_right_cmd, back_left_cmd, back_right_cmd);
        front_left_joint_.setCommand(front_left_cmd);
        front_right_joint_.setCommand(front_right_cmd);
        back_left_joint_.setCommand(back_left_cmd);
        back_right_joint_.setCommand(back_right_cmd);

        last_time_ = time;
    }

//  读取/vmd_vel上的目标速度
    void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        desired_vx_ = msg->linear.x;
        desired_vy_ = msg->linear.y;
        desired_omega_ = msg->angular.z;
        ROS_INFO("keyboard cmd: %f %f %f", desired_vx_, desired_vy_, desired_omega_);
    }
//  读取四个轮子的当前速度
    void HeroChassisController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        back_left_vel_ = msg->velocity[0];
        front_left_vel_= msg->velocity[1];
        back_right_vel_ = msg->velocity[2];
        front_right_vel_= msg->velocity[3];
    }


    PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}  // namespace simple_chassis_controller



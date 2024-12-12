//
// Created by ham on 2024/12/7.
//
//
// Created by ham on 2024/11/30.
//
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h> // 用于读取键盘输入
#include <string>

// 获取键盘输入的函数
char getKey() {
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
//  TCSANOW：不等数据传输完毕就立即改变属性。
//  TCSADRAIN：等待所有数据传输结束才改变属性。
//  TCSAFLUSH：清空输入输出缓冲区才改变属性。
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

geometry_msgs::Twist command(geometry_msgs::Twist twist_, const std::string& frame_){
    static int module = 0; //0：普通模式  1：左旋模式  2：右旋模式
    char key = getKey(); // 获取按键输入

    if(frame_ == "base_link"){
        switch (key) {
            case 'w': // 前进
                twist_.linear.x = 0.5;
                twist_.linear.y = 0.0;
                twist_.angular.z = 0.0;
                break;
            case 's': // 后退
                twist_.linear.x = -0.5;
                twist_.linear.y = 0.0;
                twist_.angular.z = 0.0;
                break;
            case 'a': // 左移
                twist_.linear.x = 0.0;
                twist_.linear.y = 0.5;
                twist_.angular.z = 0.0;

                break;
            case 'd': // 右移
                twist_.linear.x = 0.0;
                twist_.linear.y = -0.5;
                twist_.angular.z = 0.0;
                break;
            case 'q': // 左旋转
                twist_.linear.x = 0.0;
                twist_.linear.y = 0.0;
                twist_.angular.z = 0.5;
                break;
            case 'e': // 右旋转
                twist_.linear.x = 0.0;
                twist_.linear.y = 0.0;
                twist_.angular.z = -0.5;
                break;
            case 'x': // 停止
                twist_.linear.x = 0.0;
                twist_.linear.y = 0.0;
                twist_.angular.z = 0.0;
                break;
            default:
                ROS_WARN("Invalid key pressed.");
//                continue;
        }
    }else if(frame_ == "odom"){
        switch (key) {
            case 'w': // 前进
                twist_.linear.x = 0.5;
                twist_.linear.y = 0.0;
                if(module == 0){
                    twist_.angular.z = 0.0;
                }
                break;
            case 's': // 后退
                twist_.linear.x = -0.5;
                twist_.linear.y = 0.0;
                if(module == 0){
                    twist_.angular.z = 0.0;
                }
                break;
            case 'a': // 左移
                twist_.linear.x = 0.0;
                twist_.linear.y = 0.5;
                if(module == 0){
                    twist_.angular.z = 0.0;
                }
                break;
            case 'd': // 右移
                twist_.linear.x = 0.0;
                twist_.linear.y = -0.5;
                if(module == 0){
                    twist_.angular.z = 0.0;
                }
                break;
            case 'q': // 左旋转
                if(module != 0){
                    ROS_WARN("Invalid key pressed.");
                    break;
                }
                twist_.linear.x = 0.0;
                twist_.linear.y = 0.0;
                if(module == 0){
                    twist_.angular.z = 0.5;
                }
                break;
            case 'e': // 右旋转
                if(module != 0){
                    ROS_WARN("Invalid key pressed.");
                    break;
                }
                twist_.linear.x = 0.0;
                twist_.linear.y = 0.0;
                if(module == 0){
                    twist_.angular.z = -0.5;
                }
                break;
            case 'x': // 停止
                twist_.linear.x = 0.0;
                twist_.linear.y = 0.0;
                if(module == 0){
                    twist_.angular.z = 0.0;
                }
                break;
            case 'k':
                ROS_INFO("Normal mode");
                module = 0;
                twist_.linear.x = 0.0;
                twist_.linear.y = 0.0;
                twist_.angular.z = 0.0;
                break;
            case 'j':
                ROS_INFO("Left rotation mode");
                module = 1;
                twist_.linear.x = 0.0;
                twist_.linear.y = 0.0;
                twist_.angular.z = 2.0;
                break;
            case 'l':
                ROS_INFO("Right rotation mode");
                module = 2;
                twist_.linear.x = 0.0;
                twist_.linear.y = 0.0;
                twist_.angular.z = -2.0;
                break;
            default:
                ROS_WARN("Invalid key pressed.");
//                continue;
        }
    }else{
        ROS_ERROR("Cannot get frame id.");
    }
    return  twist_;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle nh;

    // 创建一个Publisher，发布geometry_msgs::Twist类型的消息
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
//    tf::TransformListener listener;

    // 消息初始化
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.angular.z = 0.0;

    ros::Rate loop_rate(10); // 设置循环频率
    std::string frame = "base_link";
    bool getparam = nh.getParam("controller/hero_chassis_controller/frame", frame);
//    param("controller/ero_chassis_controller/frame", frame, "base_link");
    if(getparam){
        if(frame == "odom"){
            ROS_INFO("-----------------------------------------------------");
            ROS_INFO("Current reference coordinate system: odom");
            ROS_INFO("In the odom frame ,use J/L to go into left and right rotation mode, use K to return to normal mode");
            ROS_INFO(" Normal mode: Use WASD to control the robot, Q/E to rotate, and X to stop.");
            ROS_INFO(" Rotation mode: Use WASD to control the robot, and X to stop.");
            ROS_INFO(" W: forward, S: backward, A: left, D: right, Q: rotate left, E: rotate right, X: stop.(Q/E cannot use in Rotation mode)");
            ROS_INFO("-----------------------------------------------------");
        }else if(frame == "base_link"){
            ROS_INFO("-----------------------------------------------------");
            ROS_INFO("Current reference coordinate system: base_link");
            ROS_INFO("Use WASD to control the robot, Q/E to rotate, and X to stop.");
            ROS_INFO("W: forward, S: backward, A: left, D: right, Q: rotate left, E: rotate right, X: stop.");
            ROS_INFO("-----------------------------------------------------");
        }
    }else{
        ROS_ERROR("Cannot get frame id, please check your yaml file");
    }


    while (ros::ok()) {

        twist = command(twist, frame);
        // 发布消息
        vel_pub.publish(twist);

        // 打印当前速度值
        ROS_INFO("linear.x: %.2f, linear.y: %.2f, angular.z: %.2f",
                 twist.linear.x, twist.linear.y, twist.angular.z);
// 按照循环频率延时
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

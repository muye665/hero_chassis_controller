//
// Created by ham on 2024/11/30.
//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h> // 用于读取键盘输入

// 获取键盘输入的函数
char getKey() {
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle nh;

    // 创建一个Publisher，发布geometry_msgs::Twist类型的消息
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // 消息初始化
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.angular.z = 0.0;

    ROS_INFO("Use WASD to control the robot, Q/E to rotate, and X to stop.");
    ROS_INFO("W: forward, S: backward, A: left, D: right, Q: rotate left, E: rotate right, X: stop.");

    ros::Rate loop_rate(10); // 设置循环频率

    while (ros::ok()) {
        char key = getKey(); // 获取按键输入
        switch (key) {
            case 'w': // 前进
                twist.linear.x = 0.5;
                twist.linear.y = 0.0;
                twist.angular.z = 0.0;
                break;
            case 's': // 后退
                twist.linear.x = -0.5;
                twist.linear.y = 0.0;
                twist.angular.z = 0.0;
                break;
            case 'a': // 左移
                twist.linear.x = 0.0;
                twist.linear.y = 0.5;
                twist.angular.z = 0.0;
                break;
            case 'd': // 右移
                twist.linear.x = 0.0;
                twist.linear.y = -0.5;
                twist.angular.z = 0.0;
                break;
            case 'q': // 左旋转
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.angular.z = 0.5;
                break;
            case 'e': // 右旋转
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.angular.z = -0.5;
                break;
            case 'x': // 停止
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.angular.z = 0.0;
                break;
            default:
                ROS_WARN("Invalid key pressed.");
                continue;
        }

        // 发布消息
        vel_pub.publish(twist);

        // 打印当前速度值
        ROS_INFO("linear.x: %.2f, linear.y: %.2f, angular.z: %.2f",
                 twist.linear.x, twist.linear.y, twist.angular.z);
// 按照循环频率延时
//        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

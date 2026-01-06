#include <ros/ros.h>
#include <signal.h>

// 假设这是你的机械臂控制接口
void stopRobot()
{
    ROS_WARN("Stopping robot...");
    // 1. 速度清零
    // robot.setJointVelocity({0,0,0,0,0,0});

    // 2. 停止运动
    // robot.stopMotion();

    // 3. 失能（下电 / Servo OFF）
    // robot.servoOff();
}

void sigintHandler(int sig)
{
    ROS_WARN("Ctrl+C detected, stopping robot safely!");
    stopRobot();
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_control_node",
              ros::init_options::NoSigintHandler);

    // 注册我们自己的 Ctrl+C 处理函数
    signal(SIGINT, sigintHandler);

    ros::NodeHandle nh;

    ROS_INFO("Robot control node started.");

    ros::Rate loop_rate(100);

    
    while( ros::ok())
    {
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
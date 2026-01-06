#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"
#include "jaka_msgs/RobotMsg.h"
#include "jaka_msgs/Move.h"
#include "jaka_msgs/ServoMoveEnable.h"
#include "jaka_msgs/ServoMove.h"
#include "jaka_msgs/SetUserFrame.h"
#include "jaka_msgs/SetTcpFrame.h"
#include "jaka_msgs/SetPayload.h"
#include "jaka_msgs/SetCollision.h"
#include "jaka_msgs/ClearError.h"
#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/conversion.h"
#include <string>
using namespace std;

BOOL in_pos;
JAKAZuRobot robot;

int main(int argc, char *argv[])
{
    string default_ip = "10.5.5.100";
    robot.login_in(default_ip.c_str());
    // robot.set_status_data_update_time_interval(100);
    // robot.set_block_wait_timeout(120);
    // robot.power_on();
    // sleep(8);
    // robot.enable_robot();
    // sleep(4);
    //Joint-space first-order low-pass filtering in robot servo mode
    //robot.servo_move_use_joint_LPF(2);
    // robot.servo_speed_foresight(15,0.03);
    JointValue joint_pose;
    joint_pose.jVal[0] = 0.001;
    joint_pose.jVal[1] = 0;
    joint_pose.jVal[2] = 0;
    joint_pose.jVal[3] = 0;
    joint_pose.jVal[4] = 0;
    joint_pose.jVal[5] = 0.001;
    // robot.servo_j(&joint_pose, MoveMode::INCR);

    CartesianPose cart;
        cart.tran.x = 0;
        cart.tran.y = -0.2;
        cart.tran.z = 0;
        cart.rpy.rx = 0.0;
        cart.rpy.ry = 0.0;
        cart.rpy.rz = 0.0;

    robot.servo_move_enable(true);
    for (int i = 0; i < 100; i++)
    {
        // servo_j_client.call(servo_pose);
        // cout << "The return value of calling servo_j:" << servo_pose.response.ret << "  ";
        // cout << servo_pose.response.message << endl;

         //robot.servo_j(&joint_pose, MoveMode::INCR);
        robot.servo_p(&cart, MoveMode::INCR);
    }

     robot.servo_move_enable(false);
    

    return 0;
}

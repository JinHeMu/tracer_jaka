#include <ros/ros.h>
#include <moveit_msgs/GetPositionIK.h>
#include <geometry_msgs/PoseStamped.h>
#include <chrono>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ik_test_loop");
    ros::NodeHandle nh;

    int num_targets = 5000;

    if (argc > 1)
    {
        try
        {
            num_targets = std::stoi(argv[1]);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
    }
    
    
    ros::ServiceClient ik_client = nh.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
    ros::service::waitForService("/compute_ik");

    moveit_msgs::GetPositionIK srv;

    srv.request.ik_request.group_name = "arm";
    srv.request.ik_request.timeout = ros::Duration(0.5);
    srv.request.ik_request.avoid_collisions = true;


    // 可以预先定义多个目标位姿
    std::vector<geometry_msgs::Pose> targets;
    geometry_msgs::Pose p;
    p.position.x = 0; p.position.y = 0; p.position.z = 0; p.orientation.w = 1.0;

    for (size_t i = 0; i < num_targets; i++)
    {
        targets.push_back(p);
    }
    

    auto start = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < targets.size(); ++i)
    {
        srv.request.ik_request.pose_stamped.header.frame_id = "gripper_center_link";
        srv.request.ik_request.pose_stamped.pose = targets[i];

        if (ik_client.call(srv))
        {

            
            if (srv.response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
            {
                // //ROS_INFO("IK solution found for target %ld", i);
                // for (size_t j = 0; j < 6; ++j)
                //     //ROS_INFO("%s: %f", srv.response.solution.joint_state.name[j].c_str(),
                //             //  srv.response.solution.joint_state.position[j]);
            }
            else
            {
                ROS_WARN("IK not found for target %ld", i);
            }
        }
        else
        {
            ROS_ERROR("Failed to call service compute_ik");
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    ROS_INFO("IK calculation took %.6f seconds", elapsed.count());

    return 0;
}

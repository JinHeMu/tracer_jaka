#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_tf_dynamic");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster br;

    // 手眼标定矩阵 T_ee_camera
    tf2::Transform T_ee_camera;
    T_ee_camera.setOrigin(tf2::Vector3(0.0235591, 0.0168618, 0.0840892));
    tf2::Matrix3x3 R(
        0.999932, 0.0109242, -0.00402889,
        -0.0108429, 0.999747, 0.0196854,
        0.00424292, -0.0196404, 0.999798);
    tf2::Quaternion q;
    R.getRotation(q);
    T_ee_camera.setRotation(q);

    ros::Rate rate(50); // 50Hz
    while (ros::ok())
    {
        geometry_msgs::TransformStamped ee_to_world;
        try
        {
            // 获取末端在 world 下的位姿
            ee_to_world = tfBuffer.lookupTransform("world", "robotiq_85_base_link", ros::Time(0), ros::Duration(1.0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        // 转成 tf2::Transform
        tf2::Transform T_world_ee;
        tf2::fromMsg(ee_to_world.transform, T_world_ee);

        // 相机在 world 下的位姿
        tf2::Transform T_world_camera = T_world_ee * T_ee_camera;

        geometry_msgs::TransformStamped cam_tf_msg;
        cam_tf_msg.header.stamp = ros::Time::now();
        cam_tf_msg.header.frame_id = "world";
        cam_tf_msg.child_frame_id = "camera_link_optical";
        cam_tf_msg.transform = tf2::toMsg(T_world_camera);

        br.sendTransform(cam_tf_msg);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

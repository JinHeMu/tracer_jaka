#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace cv;
using namespace std;

// 橙色柿子
static int iLowH = 0;
static int iHighH = 56;

static int iLowS = 139;
static int iHighS = 255;

static int iLowV = 42;
static int iHighV = 240;

Eigen::Matrix4d T_e_c; // 手眼标定矩阵

// 深度图
cv::Mat depth_image;

// 相机内参
double fx = 919.1934814453125;
double fy = 919.1843872070312;
double cx = 648.1541748046875;
double cy = 354.546875;

// TF2 监听器（作为全局变量或类成员）
std::unique_ptr<tf2_ros::Buffer> tf_buffer;
std::unique_ptr<tf2_ros::TransformListener> tf_listener;

void Depth_Callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        depth_image = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void RGB_Callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat imgOriginal = cv_ptr->image;

    // 转 HSV
    Mat imgHSV;
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);
    vector<Mat> hsvSplit;
    split(imgHSV, hsvSplit);
    equalizeHist(hsvSplit[2], hsvSplit[2]);
    merge(hsvSplit, imgHSV);

    // HSV 二值化
    Mat imgThresholded;
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

    // 开闭操作
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

    // 计算质心
    int nTargetX = 0, nTargetY = 0, nPixCount = 0;
    for (int y = 0; y < imgThresholded.rows; y++)
    {
        for (int x = 0; x < imgThresholded.cols; x++)
        {
            if (imgThresholded.at<uchar>(y, x) == 255)
            {
                nTargetX += x;
                nTargetY += y;
                nPixCount++;
            }
        }
    }

    if (nPixCount > 0)
    {
        nTargetX /= nPixCount;
        nTargetY /= nPixCount;

        // 绘制质心
        line(imgOriginal, Point(nTargetX - 10, nTargetY), Point(nTargetX + 10, nTargetY), Scalar(255, 0, 0));
        line(imgOriginal, Point(nTargetX, nTargetY - 10), Point(nTargetX, nTargetY + 10), Scalar(255, 0, 0));

        // 读取深度
        if (depth_image.empty())
        {
            ROS_WARN("Depth image not ready!");
            return;
        }

        // 检查坐标是否在图像范围内
        if (nTargetY < 0 || nTargetY >= depth_image.rows || nTargetX < 0 || nTargetX >= depth_image.cols)
        {
            ROS_WARN("Target coordinates out of image bounds!");
            return;
        }

        float Z = depth_image.at<float>(nTargetY, nTargetX) / 1000.0;
        ROS_INFO("Depth value at target pixel: %.3f", Z);
        if (Z <= 0.001) // 避免除零和无效深度
        {
            ROS_WARN("Depth value is invalid at target pixel!");
            return;
        }

        // 相机坐标
        double Xc = (nTargetX - cx) * Z / fx;
        double Yc = (nTargetY - cy) * Z / fy;
        double Zc = Z;
        Eigen::Vector4d P_c(Xc, Yc, Zc, 1.0);

        // 末端坐标
        Eigen::Vector4d P_e = T_e_c * P_c;
        ROS_INFO("Object in end-effector frame: X=%.3f Y=%.3f Z=%.3f", P_e(0), P_e(1), P_e(2));

        // 世界坐标（使用TF2）
        try
        {
            geometry_msgs::TransformStamped transform = tf_buffer->lookupTransform("world", "robotiq_85_base_link", ros::Time(0), ros::Duration(0.5));

            // 将geometry_msgs::Transform转换为Eigen::Affine3d
            Eigen::Affine3d T_b_e = tf2::transformToEigen(transform);

            Eigen::Vector3d P_b = T_b_e * P_e.head<3>();
            ROS_INFO("Object in world frame: X=%.3f Y=%.3f Z=%.3f", P_b(0), P_b(1), P_b(2));

            static tf2_ros::TransformBroadcaster br;
            geometry_msgs::TransformStamped transformStamped;

            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "base_link"; // 父坐标系
            transformStamped.child_frame_id = "object";       // 子坐标系

            // 设置平移
            transformStamped.transform.translation.x = P_b(0);
            transformStamped.transform.translation.y = P_b(1);
            transformStamped.transform.translation.z = P_b(2);

            // 设置旋转（四元数）
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();

            // 发送tf
            br.sendTransform(transformStamped);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("TF2 exception: %s", ex.what());
            return;
        }
    }

    // 显示
    imshow("RGB", imgOriginal);
    imshow("HSV", imgHSV);
    imshow("Result", imgThresholded);
    waitKey(5);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_position_node");
    ros::NodeHandle nh;

    // 初始化TF2
    tf_buffer = std::make_unique<tf2_ros::Buffer>();
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

    // 手眼标定矩阵
    T_e_c << 0.999907, 0.0115691, -0.00728045, 0.0272444,
        -0.0114391, 0.999779, 0.0176516, 0.0171443,
        0.00748305, -0.0175666, 0.999818, 0.0822473,
        0, 0, 0, 1;

    // 订阅深度图
    ros::Subscriber depth_sub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, Depth_Callback);

    // 订阅RGB图
    ros::Subscriber rgb_sub = nh.subscribe("/camera/color/image_raw", 1, RGB_Callback);

    namedWindow("RGB");
    namedWindow("HSV");
    namedWindow("Result");

    ros::spin();

    // 清理
    destroyAllWindows();

    return 0;
}
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <jaka_driver/JAKAZuRobot.h>
#include <signal.h>

using namespace cv;
using namespace std;

static int iLowH = 0;
static int iHighH = 60;

static int iLowS = 143;
static int iHighS = 255;

static int iLowV = 140;
static int iHighV = 255;

JAKAZuRobot robot;

// 假设这是你的机械臂控制接口
void stopRobot()
{
    ROS_WARN("Stopping robot...");
    // 1. 速度清零
    // robot.setJointVelocity({0,0,0,0,0,0});
        CartesianPose cart;
        cart.tran.x = 0;
        cart.tran.y = 0;
        cart.tran.z = 0.0;
        cart.rpy.rx = 0.0;
        cart.rpy.ry = 0.0;
        cart.rpy.rz = 0.0;
    robot.servo_p(&cart, INCR);

    // 2. 停止运动
    robot.servo_move_enable(FALSE);
    // 3. 失能（下电 / Servo OFF）
    //robot.disable_robot();
}

void sigintHandler(int sig)
{
    ROS_WARN("Ctrl+C detected, stopping robot safely!");
    stopRobot();
    ros::shutdown();
}

void Cam_RGB_Callback(const sensor_msgs::Image msg)
{
    // ROS_INFO("Cam_RGB_Callback");
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

    // 将RGB图片转换成HSV
    Mat imgHSV;
    vector<Mat> hsvSplit;
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

    // 在HSV空间做直方图均衡化
    split(imgHSV, hsvSplit);
    equalizeHist(hsvSplit[2], hsvSplit[2]);
    merge(hsvSplit, imgHSV);
    Mat imgThresholded;

    // 使用上面的Hue,Saturation和Value的阈值范围对图像进行二值化
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

    // 开操作 (去除一些噪点)
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

    // 闭操作 (连接一些连通域)
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

    // 遍历二值化后的图像数据
    int nTargetX = 0;
    int nTargetY = 0;
    int nPixCount = 0;
    int nImgWidth = imgThresholded.cols;
    int nImgHeight = imgThresholded.rows;
    int nImgChannels = imgThresholded.channels();
    // printf("w= %d   h= %d   size = %d\n",nImgWidth,nImgHeight,nImgChannels);
    for (int y = 0; y < nImgHeight; y++)
    {
        for (int x = 0; x < nImgWidth; x++)
        {
            // printf("%d  ",imgThresholded.data[y*nImgWidth + x]);
            if (imgThresholded.data[y * nImgWidth + x] == 255)
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
        printf("颜色质心坐标( %d , %d )  点数 = %d\n", nTargetX, nTargetY, nPixCount);

        // 画坐标
        Point line_begin = Point(nTargetX - 10, nTargetY);
        Point line_end = Point(nTargetX + 10, nTargetY);
        line(imgOriginal, line_begin, line_end, Scalar(255, 0, 0));
        line_begin.x = nTargetX;
        line_begin.y = nTargetY - 10;
        line_end.x = nTargetX;
        line_end.y = nTargetY + 10;
        line(imgOriginal, line_begin, line_end, Scalar(255, 0, 0));

        int u0 = nImgWidth / 2;
        int v0 = nImgHeight / 2;
        int ex = nTargetX - u0; // x 像素误差
        int ey = nTargetY - v0; // y 像素误差

        ROS_INFO("像素误差( %d , %d )\n", ex, ey);

        CartesianPose cart;
        cart.tran.x = -ey * 0.01;
        cart.tran.y = -ex * 0.01;
        cart.tran.z = 0.0;
        cart.rpy.rx = 0.0;
        cart.rpy.ry = 0.0;
        cart.rpy.rz = 0.0;


        robot.servo_p(&cart, INCR);
    }
    else
    {

        CartesianPose cart;
        cart.tran.x = 0;
        cart.tran.y = 0;
        cart.tran.z = 0.0;
        cart.rpy.rx = 0.0;
        cart.rpy.ry = 0.0;
        cart.rpy.rz = 0.0;

        robot.servo_p(&cart, INCR);

        printf("目标颜色消失...\n");
    }

    // 显示处理结果
    imshow("RGB", imgOriginal);
    imshow("HSV", imgHSV);
    imshow("Result", imgThresholded);
    cv::waitKey(5);
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "jaka_follow");
    ros::NodeHandle nh;

    string default_ip = "10.5.5.100";
    string robot_ip = nh.param("ip", default_ip);

    robot.login_in(robot_ip.c_str());
    robot.servo_move_enable(true);
    // robot.set_status_data_update_time_interval(100);
    // robot.set_block_wait_timeout(120);

    // robot.enable_robot();
    // sleep(4);
    // robot.servo_speed_foresight(15, 0.03);



    ros::Subscriber rgb_sub = nh.subscribe("/camera/color/image_raw", 1, Cam_RGB_Callback);

    ros::Rate loop_rate(100);

    // 注册我们自己的 Ctrl+C 处理函数
    signal(SIGINT, sigintHandler);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

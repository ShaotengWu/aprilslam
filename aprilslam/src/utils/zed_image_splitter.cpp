#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <string>
#include <vector>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

// #define COMPRESSED

sensor_msgs::CameraInfo getCameraInfo(void)
{ // extract cameraInfo.
    sensor_msgs::CameraInfo cam;

    std::vector<double> D{-0.0394652, 0.00863897, -0.00054791, -0.00016387, -0.00461753};

    boost::array<double, 9> K = {
        530.30, 0, 658.24,
        0, 530.00, 370.78,
        0, 0, 1};

    boost::array<double, 12> P = {
        530.30, 0, 658.24, 0,
        0, 530.00, 370.78, 0,
        0, 0, 1, 0};
    boost::array<double, 9> r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    cam.height = 720;
    cam.width = 1280;
    cam.distortion_model = "plumb_bob";
    cam.D = D;
    cam.K = K;
    cam.P = P;
    cam.R = r;
    cam.binning_x = 0;
    cam.binning_y = 0;
    return cam;
}
cv::Mat image_raw;
std_msgs::Header header;
bool valid = false;

void zedImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    // origin_image_ptr = img;
    if (msg == nullptr)
        return;
    valid = true;
    image_raw = cv_bridge::toCvShare(msg, "bgr8")->image;
    header = msg->header;
}

void zedCompressedImageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    if (msg == nullptr)
        return;
    valid = true;

    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image_raw = cv_ptr_compressed->image;
    
    header = msg->header;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_image_splitter");
    ros::NodeHandle nh;

    sensor_msgs::CameraInfo camera_info = getCameraInfo();

    image_transport::ImageTransport it(nh);
#ifdef COMPRESSED
    ros::Subscriber zed_sub = nh.subscribe("/zed2/image_raw/compressed", 1, zedCompressedImageCallback);
#endif
#ifndef COMPRESSED
    image_transport::Subscriber zed_sub = it.subscribe("/zed2/image_raw", 1, zedImageCallback);
#endif //  COMPRESSED

    ros::Publisher zed_left_pub = nh.advertise<sensor_msgs::Image>("/zed2_left/image_raw", 1, true);
    ros::Publisher zed_camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/zed2_left/camera_info", 1, true);

    ros::Rate freq(30);

    while (ros::ok())
    {
        ros::spinOnce();
        if (!valid)
        {
            ROS_INFO("No image yet");
            freq.sleep();
            continue;
        }
        valid = false;
        // cv::Mat image_raw = cv::Mat(image_raw);
        std::cout<<image_raw.cols<<" "<<image_raw.rows<<std::endl;
        cv::Rect rect_left(0, 0, 1280, 720);
        cv::Mat image_left = image_raw(rect_left);

        cv::Mat image_gray, image_equalized;
        cv::cvtColor(image_left, image_gray, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(image_gray, image_equalized);
        cv_bridge::CvImage image_bridge(header, sensor_msgs::image_encodings::MONO8, image_equalized);

        sensor_msgs::Image image_msg;
        image_bridge.toImageMsg(image_msg);

        camera_info.header = header;

        // pub
        zed_left_pub.publish(image_msg);
        zed_camera_info_pub.publish(camera_info);

        freq.sleep();
    }

    return 0;
}
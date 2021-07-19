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
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

std::string in_bag_path = "/home/wushaoteng/project/electroMechanical/catkin_ws/data/0719-02-origin.bag";
std::string out_bag_path = "/home/wushaoteng/project/electroMechanical/catkin_ws/data/0719-02-histEqualized.bag";

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_camera_info");
    ros::NodeHandle nh;
    rosbag::Bag in_bag;
    rosbag::Bag out_bag;
    in_bag.open(in_bag_path, rosbag::bagmode::Read);
    out_bag.open(out_bag_path, rosbag::bagmode::Write);

    sensor_msgs::CameraInfo camera_info = getCameraInfo();

    ros::Rate freq(30);

    std::vector<std::string> topics;
    topics.push_back(std::string("/zed2/image_raw/compressed"));

    int seq = 0;
    for (rosbag::MessageInstance const m : rosbag::View(in_bag))
    {
        sensor_msgs::CompressedImagePtr c_imgptr = m.instantiate<sensor_msgs::CompressedImage>();
        ROS_INFO("Convert a message");
        c_imgptr->header.stamp = ros::Time::now();

        std_msgs::Header new_header;
        new_header.stamp = ros::Time::now();
        new_header.seq = seq++;
        // sensor_msgs::CompressedImage c_img;
        // c_img.data = img->data;
        // c_img.format = img->format;
        // c_img.header = img->header;
        cv::Mat image_raw = cv::imdecode(cv::Mat(c_imgptr->data), 1);
        cv::Rect rect_left(0, 0, 1280, 720);
        cv::Mat image_left = image_raw(rect_left);

        cv::Mat image_gray, image_equalized;
        // std::cout<<image_left.channels()<<std::endl;
        cv::cvtColor(image_left, image_gray, cv::COLOR_BGR2GRAY);
        // std::cout<<image_gray.channels()<<std::endl;
        
        // cv::namedWindow("Gray", cv::WINDOW_AUTOSIZE);
        // cv::imshow("Gray", image_equalized);
        // cv::waitKey(0);
        // std::cout<<"Debug here"<<std::endl;
        cv::equalizeHist(image_gray, image_equalized);
        
        // cv::namedWindow("Equalized", cv::WINDOW_AUTOSIZE);
        // cv::imshow("Equalized", image_equalized);
        // cv::waitKey(0);
        // std::cout<<"Debug here"<<std::endl;
        cv_bridge::CvImage image_bridge(new_header, sensor_msgs::image_encodings::MONO8, image_equalized);
        // std::cout<<image_bridge.encoding<<std::endl;
        
        sensor_msgs::Image image_msg;
        image_bridge.toImageMsg(image_msg);

        camera_info.header = new_header;

        out_bag.write("/zed2/image_raw", image_msg.header.stamp, image_msg);
        out_bag.write("/zed2/camera_info", camera_info.header.stamp, camera_info);

        freq.sleep();
    }

    in_bag.close();
    out_bag.close();
    return 0;
}
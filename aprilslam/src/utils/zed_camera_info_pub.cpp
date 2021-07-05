#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <vector>

ros::Publisher camera_info_pub;
sensor_msgs::CameraInfo camera_info;

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

void imageCallback(const sensor_msgs::ImageConstPtr &img)
{
    camera_info.header.stamp = ros::Time::now();
    camera_info_pub.publish(camera_info);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_camera_info");
    ros::NodeHandle nh;
    camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/zed2/camera_info", 1, true);
    ros::Subscriber image_sub = nh.subscribe("/zed2/image_raw", 1, imageCallback);
    camera_info = getCameraInfo();
    ros::spin();



    return 0;
}

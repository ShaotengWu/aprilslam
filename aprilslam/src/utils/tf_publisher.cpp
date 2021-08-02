#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_publisher");
    ros::NodeHandle node;

    tf::TransformBroadcaster tf_broadcaster;
    tf::TransformListener tf_listener;
    ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseStamped>("/camera_pose_in_map", 1, true);
    
    ros::Rate freq(30);
    while (ros::ok())
    {
        // 获取 camera 在map下的tf
        tf::StampedTransform transform;
        try
        {
            tf_listener.waitForTransform("/map", "/camera", ros::Time(0), ros::Duration(1.0));
            tf_listener.lookupTransform("/map", "/camera", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            /* code for Catch */
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "/map";
        pose.pose.position.x = transform.getOrigin().x();
        pose.pose.position.y = transform.getOrigin().y();
        pose.pose.position.z = transform.getOrigin().z(); 
        pose.pose.orientation.x = transform.getRotation().x();
        pose.pose.orientation.y = transform.getRotation().y();
        pose.pose.orientation.z = transform.getRotation().z();
        pose.pose.orientation.w = transform.getRotation().w();
        pose_pub.publish(pose);
        freq.sleep();
    }

    return 0;
};

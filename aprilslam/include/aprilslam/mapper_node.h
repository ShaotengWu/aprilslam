#ifndef APRILSLAM_MAPPER_NODE_H_
#define APRILSLAM_MAPPER_NODE_H_

#include <ros/ros.h>
#include <aprilslam/Apriltags.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>
#include <image_geometry/pinhole_camera_model.h>
#include "aprilslam/visualizer.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <yaml-cpp/yaml.h>

#include "aprilslam/mapper.h"
#include "aprilslam/tag_map.h"

namespace aprilslam
{

    class MapperNode
    {
    public:
        MapperNode(const ros::NodeHandle &nh, const std::string &frame_id);

        bool GetGoodTags(const std::vector<aprilslam::Apriltag> tags_c,
                         std::vector<aprilslam::Apriltag> *tags_c_good);

    private:
        void TagsCb(const aprilslam::ApriltagsConstPtr &tags_c_msg);
        void CinfoCb(const sensor_msgs::CameraInfoConstPtr &cinfo_msg);

        ros::NodeHandle nh_;
        ros::Subscriber sub_tags_;
        ros::Subscriber sub_cinfo_;
        ros::Publisher pub_cam_trajectory_;
        ros::Publisher pub_obj_pointcloud_;
        std::string frame_id_;
        aprilslam::TagMap map_;
        aprilslam::Mapper mapper_;
        aprilslam::ApriltagVisualizer tag_viz_;
        image_geometry::PinholeCameraModel model_;
        tf2_ros::TransformBroadcaster tf_broadcaster_;

        // Whether use prior localization knowledge of apriltags
        bool use_tag_prior_info_;
        std::string tag_prior_info_path_;
        YAML::Node tag_prior_info_node_;
        std::map<size_t, geometry_msgs::Pose> tag_prior_poses_;

        nav_msgs::Path cam_trajectory_;
    };

} // namespace aprilslam

#endif // APRILSLAM_MAPPER_NODE_H_

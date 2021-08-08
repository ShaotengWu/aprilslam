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
    /**
     * @brief The majority of beckend for data collection and factor graph optimization. 
     */
    class MapperNode
    {
    public:
        /**
         * @brief Construct a new MapperNode object
         * 
         * @param nh Node handle created in main().
         * @param frame_id Apriltags static frame id. The output camera pose is in this frame. If you need transform camera pose from this frame_id
         * to PointCloudMap frame_id, please refer to tf_publisher.launch
         */
        MapperNode(const ros::NodeHandle &nh, const std::string &frame_id);

        /**
         * @brief Filter good apriltag detections for better localization performance.
         * 
         * @param [in] tags_c Original detections from apriltag_ros detector.
         * @param [out] tags_c_good Filtered detections.
         * @return true :If there is any good detections.
         * @return false :If there is no good detection.
         */
        bool GetGoodTags(const std::vector<aprilslam::Apriltag> tags_c,
                         std::vector<aprilslam::Apriltag> *tags_c_good);

    private:
        /**
         * @brief Detection callback and optimization .It's the core function in Apriltag SLAM.
         * 
         * @param tags_c_msg Original detections from apriltag_ros detector.
         * @todo The callback function should not be so long for efficiency. Move optimization steps out of the callback function.
         */
        void TagsCb(const aprilslam::ApriltagsConstPtr &tags_c_msg);
        
        /**
         * @brief Camera info callback
         * 
         * @param cinfo_msg Const shared_ptr<sensor_msgs::CameraInfo>, refer to http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html for details.
         */
        void CinfoCb(const sensor_msgs::CameraInfoConstPtr &cinfo_msg);

        /// @brief Node handle created in main().
        ros::NodeHandle nh_;
        
        /// @brief Subscirber for apriltag detections. Callback function is TagsCb.
        ros::Subscriber sub_tags_;

        /// @brief Subscirber for camera info. Callback function is CinfoCb.
        ros::Subscriber sub_cinfo_;
        
        /// @brief Publisher for continuous localization result.
        ros::Publisher pub_cam_trajectory_;

        /// @brief Publisher for all Apriltag corners
        ros::Publisher pub_obj_pointcloud_;

        /// @brief Apriltags static frame id. The output camera pose is in this frame. If you need transform camera pose from this frame_id to PointCloudMap frame_id, please refer to tf_publisher.launch
        std::string frame_id_;

        /// @brief Manager of all data.
        aprilslam::TagMap map_;

        /// @brief Optimizer.
        aprilslam::Mapper mapper_;

        /// @brief Apriltag visualizer.
        aprilslam::ApriltagVisualizer tag_viz_;
        
        /// @brief Camera model data structure.
        image_geometry::PinholeCameraModel model_;

        /// @brief TF broadcaster for frame: camera and frame: tag 
        tf2_ros::TransformBroadcaster tf_broadcaster_;

        /// @brief Whether use prior localization knowledge of apriltags
        bool use_tag_prior_info_;
        
        /// @brief Path to prior information yaml.
        std::string tag_prior_info_path_;

        /// @brief The prior info is loaded in this YAML data structure.
        YAML::Node tag_prior_info_node_;

        /// @brief Prior information is extracted from YAML::NODE and stored in this RbTree.
        std::map<int, geometry_msgs::Pose> tag_prior_poses_;

        /// @brief Pose number counter .
        int pose_cnt_;

        /// @brief Camera velocity updated by adjacent frames.
        geometry_msgs::Pose cam_velocity_;

        /// @brief Continuous localization result. Published by pub_cam_trajectory_.
        nav_msgs::Path cam_trajectory_;
        
    };

} // namespace aprilslam

#endif // APRILSLAM_MAPPER_NODE_H_

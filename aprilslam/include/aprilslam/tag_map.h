#ifndef APRILSLAM_TAG_MAP_H_
#define APRILSLAM_TAG_MAP_H_

#include <aprilslam/Apriltags.h>
#include <sensor_msgs/PointCloud.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>
#include <set>

namespace aprilslam
{
    /**
     * @brief Manager of all data.
     * 
     */
    class TagMap
    {
    public:
        /**
         * @brief Construct a new TagMap object
         * 
         * @param tag_family Apriltag family
         * @param tag_size size of tag
         */
        TagMap(std::string tag_family, double tag_size);

        /**
         * @brief Add or update optimization result of tag_w 
         * 
         * @param tag_w The tag to be added or updated. Saved in tags_w_ and tags_w_map_.
         * @param pose The value to be added or updated.
         */
        void AddOrUpdate(const Apriltag &tag_w, const geometry_msgs::Pose &pose);

        /**
         * @brief update optimization result of tag_w 
         * 
         * @param tag_w The tag to be updated. Saved in tags_w_ and tags_w_map_.
         * @param pose The value to be updated. 
         */
        void UpdateTag(Apriltag *tag_w, const geometry_msgs::Pose &pose);

        /**
         * @brief Load prior info from YAML and convert it to a vector.
         * 
         * @param tags_prior_info 
         */
        void UpdateTagsPriorInfo(const std::map<int, geometry_msgs::Pose> tags_prior_info);

        /**
         * @brief 
         * 
         * @param tag_c 
         */
        void AddFirstTag(const Apriltag &tag_c);

        /**
         * @brief Estimate camera initial pose use PnP and calculate const value motion model reprojection value.
         * 
         * @param tags_c Filtered tags used to be solve PnP
         * @param K Camera intrinsics.
         * @param D Distortion coefficients.
         * @param [out] pose Output camera pose.
         * @return true - If PnP is solved properly.
         * @return false - If PnP is not solved properly.
         */
        bool EstimatePose(const std::vector<Apriltag> &tags_c, const cv::Matx33d &K, const cv::Mat_<double> &D, geometry_msgs::Pose *pose);
        
        /**
         * @brief Update current optimized pose.
         * 
         * @param pose current optimized pose.
         */
        void UpdateCurrentCamPose(const geometry_msgs::Pose &pose);

        bool init() const { return !tags_w().empty(); }

        const aprilslam::Apriltag &first_tag() const { return tags_w().front(); }

        const std::vector<aprilslam::Apriltag> &tags_w() const { return tags_w_; }

        const std::vector<aprilslam::Apriltag> &tags_w_prior() const
        {
            return tags_w_prior_;
        }
        const sensor_msgs::PointCloud &obj_pointcloud_viz() { return obj_pointcloud_viz_; }
        
        const geometry_msgs::Pose &getVelocity() const {return cam_velocity_msg_;}

    private:
        void AddTag(const Apriltag &tag, const geometry_msgs::Pose &pose);
        
        /// @brief data structure for optimized landmarks position.
        std::vector<aprilslam::Apriltag> tags_w_;

        /// @brief data structure for prior info.
        std::vector<aprilslam::Apriltag> tags_w_prior_;

        /// @brief data structure for optimized landmarks position.
        std::map<int, aprilslam::Apriltag> tags_w_map_;

        /// @brief data structure for fast search of prior info.
        std::map<int, geometry_msgs::Pose> tags_prior_info_;

        /// @brief Apriltag family
        std::string tag_family_;
        double tag_size_;
        bool first_flag_;
        bool last_cam_pose_valid_;
        bool velocity_valid_;

        /// @brief current pose
        Eigen::Isometry3d current_cam_pose_;
        
        /// @brief last pose for velocity calculation
        Eigen::Isometry3d last_cam_pose_;

        /// @brief velocity estimation from current and last.
        Eigen::Isometry3d cam_velocity_;

        /// @brief velocity estimation from current and last. geometry::Pose
        geometry_msgs::Pose cam_velocity_msg_;

        /// @brief Visualization of Apriltags corner points 
        sensor_msgs::PointCloud obj_pointcloud_viz_;
    };

    std::vector<aprilslam::Apriltag>::const_iterator FindById(int id, const std::vector<aprilslam::Apriltag> &tags);
    std::vector<aprilslam::Apriltag>::iterator FindById(int id, std::vector<aprilslam::Apriltag> &tags);
} // namespace aprilslam

#endif // APRILSLAM_TAG_MAP_H_

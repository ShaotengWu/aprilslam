#ifndef APRILSLAM_TAG_MAP_H_
#define APRILSLAM_TAG_MAP_H_

#include <aprilslam/Apriltags.h>
#include <sensor_msgs/PointCloud.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>
#include <set>

namespace aprilslam
{
    /// @brief  
    class TagMap
    {
    public:
        TagMap(std::string tag_family, double tag_size);

        void AddOrUpdate(const Apriltag &tag_w, const geometry_msgs::Pose &pose);
        void UpdateTag(Apriltag *tag_w, const geometry_msgs::Pose &pose);
        void UpdateTagsPriorInfo(const std::map<size_t, geometry_msgs::Pose> tags_prior_info);
        void AddFirstTag(const Apriltag &tag_c);
        bool EstimatePose(const std::vector<Apriltag> &tags_c, const cv::Matx33d &K, const cv::Mat_<double> &D, geometry_msgs::Pose *pose);
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

        std::vector<aprilslam::Apriltag> tags_w_;
        std::vector<aprilslam::Apriltag> tags_w_prior_;
        std::map<int, aprilslam::Apriltag> tags_w_map_;
        std::map<size_t, geometry_msgs::Pose> tags_prior_info_;

        std::string tag_family_;
        double tag_size_;
        bool first_flag_;
        bool last_cam_pose_valid_;
        bool velocity_valid_;

        Eigen::Isometry3d current_cam_pose_;
        Eigen::Isometry3d last_cam_pose_;
        Eigen::Isometry3d cam_velocity_;
        geometry_msgs::Pose cam_velocity_msg_;

        sensor_msgs::PointCloud obj_pointcloud_viz_;
    };

    std::vector<aprilslam::Apriltag>::const_iterator FindById(int id, const std::vector<aprilslam::Apriltag> &tags);
    std::vector<aprilslam::Apriltag>::iterator FindById(int id, std::vector<aprilslam::Apriltag> &tags);
} // namespace aprilslam

#endif // APRILSLAM_TAG_MAP_H_

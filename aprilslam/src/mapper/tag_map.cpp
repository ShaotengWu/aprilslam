#include "aprilslam/tag_map.h"
#include "aprilslam/utils.h"

#include <ros/ros.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
namespace aprilslam
{

    TagMap::TagMap(std::string tag_family, double tag_size) : tag_family_(tag_family),
                                                              tag_size_(tag_size),
                                                              first_flag_(true),
                                                              last_cam_pose_valid_(false),
                                                              velocity_valid_(false)
    {
    }

    void TagMap::AddOrUpdate(const Apriltag &tag_w, const geometry_msgs::Pose &pose)
    {
        auto it = FindById(tag_w.id, tags_w_);
        if (it == tags_w().end())
        {
            // Not in map, add to map
            AddTag(tag_w, pose);
        }
        else
        {
            // Already in map update
            UpdateTag(&(*it), pose);
        }
    }

    void TagMap::UpdateTag(Apriltag *tag_w, const geometry_msgs::Pose &pose)
    {
        tag_w->pose = pose;
        SetCorners(&tag_w->corners, tag_w->pose, tag_w->size);
        tag_w->center = tag_w->pose.position;
    }

    void TagMap::AddTag(const Apriltag &tag, const geometry_msgs::Pose &pose)
    {
        // 这里的位姿应该是T_w_tag, 从tag坐标系到世界坐标系的转换矩阵，也是tag在世界坐标系中的位姿
        Apriltag tag_w = tag;
        UpdateTag(&tag_w, pose);
        tags_w_.push_back(tag_w);
        tags_w_map_.insert(std::pair<int, aprilslam::Apriltag>(tag_w.id, tag_w));
        
    }

    void TagMap::AddFirstTag(const Apriltag &tag_c)
    {
        // Creat tag in world frame and set to origin
        // Set the first tag to origin
        ROS_INFO("First tag: %d", tag_c.id);
        geometry_msgs::Pose pose;

        if (tags_prior_info_.find(tag_c.id) != tags_prior_info_.end())
        {
            ROS_INFO("\033[1;32m---->There is prior information of first tag: %x \033[0m", tag_c.id);
            pose = tags_prior_info_.find(tag_c.id)->second;
            // SetPose(&pose);
            AddTag(tag_c, pose);
        }
        else
        {
            ROS_WARN("There is no prior information about first tag: %x", tag_c.id);
            ROS_WARN("Tag map initialization failed. Try again.");
        }
    }

    bool TagMap::EstimatePose(const std::vector<Apriltag> &tags_c, const cv::Matx33d &K, const cv::Mat_<double> &D, geometry_msgs::Pose *pose)
    {
        std::vector<cv::Point2f> img_pts;
        std::vector<cv::Point3f> obj_pts;

        for (const Apriltag &tag_c : tags_c)
        {
            // Find 2D-3D correspondences
            auto it = FindById(tag_c.id, tags_w());
            if (it != tags_w().cend())
            {
                const Apriltag &tag_w = *it;
                std::for_each(tag_w.corners.begin(), tag_w.corners.end(),
                              [&obj_pts](const geometry_msgs::Point &p_w)
                              {
                                  obj_pts.emplace_back(p_w.x, p_w.y, p_w.z);
                              });
                std::for_each(tag_c.corners.begin(), tag_c.corners.end(),
                              [&img_pts](const geometry_msgs::Point &p_c)
                              {
                                  img_pts.emplace_back(p_c.x, p_c.y);
                              });
            }
        }

        ROS_ASSERT_MSG(img_pts.size() == obj_pts.size(), "size mismatch!");
        if (img_pts.empty())
            return false;

        // Publish world points
        sensor_msgs::PointCloud obj_pointcloud_viz;
        obj_pointcloud_viz.points.resize(obj_pts.size());
        obj_pointcloud_viz.channels.resize(1);
        obj_pointcloud_viz.channels[0].name = "intensity";
        obj_pointcloud_viz.channels[0].values.resize(obj_pts.size());
        for (size_t i = 0; i < obj_pts.size(); i++)
        {
            obj_pointcloud_viz.points[i].x = obj_pts[i].x;
            obj_pointcloud_viz.points[i].y = obj_pts[i].y;
            obj_pointcloud_viz.points[i].z = obj_pts[i].z;
            obj_pointcloud_viz.channels[0].values[i] = 1.0;
        }
        obj_pointcloud_viz_ = obj_pointcloud_viz;


        // Use all correspondences to solve a PnP
        // Actual pose estimation work here
        cv::Mat c_r_w, c_t_w, c_R_w;
        cv::solvePnPRansac(obj_pts, img_pts, K, D, c_r_w, c_t_w);
        cv::Rodrigues(c_r_w, c_R_w);
        cv::Mat c_T_w(c_t_w);
        cv::Mat w_R_c(c_R_w.t());
        cv::Mat w_T_c = -w_R_c * c_T_w;

        // Check pnp reprojection error
        std::vector<cv::Point2f> reproj_img_pts_pnp;
        cv::projectPoints(obj_pts, c_r_w, c_t_w, K, D, reproj_img_pts_pnp);

        // check motion model reprojection model
        //! error here!!!!
        std::vector<cv::Point2f> reproj_img_pts_mm;
        cv::Mat w_t_c_mm;
        Eigen::Quaterniond w_Q_c_mm;
        if (velocity_valid_)
        {
            // Twc1 * Tc1c2
            Eigen::Isometry3d w_pose_c_cur_mm = last_cam_pose_ * cam_velocity_;
            Eigen::Vector3d w_t_c_mm_eigen(w_pose_c_cur_mm.translation().matrix());
            cv::eigen2cv(w_t_c_mm_eigen, w_t_c_mm);
            w_Q_c_mm = Eigen::Quaterniond(w_pose_c_cur_mm.rotation());

            // inv()
            Eigen::Matrix3d c_pose_w_cur_mm_rot = w_pose_c_cur_mm.rotation().transpose();
            Eigen::Vector3d c_pose_w_cur_mm_t = -c_pose_w_cur_mm_rot * w_pose_c_cur_mm.translation().matrix();
            Eigen::Isometry3d c_pose_w_cur_mm = Eigen::Isometry3d::Identity();
            c_pose_w_cur_mm.rotate(c_pose_w_cur_mm_rot);
            c_pose_w_cur_mm.pretranslate(c_pose_w_cur_mm_t);

            cv::Mat c_R_w_mm, c_r_w_mm, c_t_w_mm;
            Eigen::Matrix3d c_R_w_mm_eigen = c_pose_w_cur_mm.rotation();
            Eigen::Vector3d c_t_w_mm_eigen(c_pose_w_cur_mm.translation().matrix());
            cv::eigen2cv(c_R_w_mm_eigen, c_R_w_mm);
            cv::eigen2cv(c_t_w_mm_eigen, c_t_w_mm);
            cv::Rodrigues(c_R_w_mm, c_r_w_mm);
            cv::projectPoints(obj_pts, c_r_w_mm, c_t_w_mm, K, D, reproj_img_pts_mm);
        }

        float reproj_error_pnp = 0.0;
        float reproj_error_mm = 0.0;
        for (size_t ip = 0; ip < reproj_img_pts_pnp.size(); ip++)
        {
            cv::Point2f origin_pt = img_pts[ip];
            cv::Point2f reproj_pt = reproj_img_pts_pnp[ip];
            reproj_error_pnp += cv::norm(origin_pt - reproj_pt);

            if (velocity_valid_)
            {
                cv::Point2f reproj_pt_mm = reproj_img_pts_mm[ip];
                reproj_error_mm += cv::norm(origin_pt - reproj_pt_mm);
            }
        }
        double rmse_pnp = std::sqrt(reproj_error_pnp / reproj_img_pts_pnp.size());
        double rmse_mm = DBL_MAX;
        if (velocity_valid_)
        {
            rmse_mm = std::sqrt(reproj_error_mm / reproj_img_pts_mm.size());
        }

        ROS_INFO("Current PnP reprojection error: %f", rmse_pnp);
        printf("\033[1A");
        printf("\033[K");
        // ROS_INFO("Current Motion Model reprojection error: %f", rmse_mm);

        double *pt = w_T_c.ptr<double>();
        SetPosition(&pose->position, pt[0], pt[1], pt[2]);

        Eigen::Quaterniond w_Q_c = RodriguesToQuat(c_r_w).inverse();
        SetOrientation(&pose->orientation, w_Q_c);

        if (rmse_pnp >= rmse_mm * 0.75)
        {
            cam_velocity_msg_ = Isometry3dToPoseMsg(cam_velocity_);
            
        }
        

        return true;
    }

    void TagMap::UpdateTagsPriorInfo(const std::map<size_t, geometry_msgs::Pose> tags_prior_info)
    {
        //tags_w_prior_.resize(tags_prior_info.size());
        tags_prior_info_ = tags_prior_info;
        auto iter_begin = tags_prior_info.begin();
        auto iter_end = tags_prior_info.end();

        for (auto iter_tag = iter_begin; iter_tag != iter_end; iter_tag++)
        {
            aprilslam::Apriltag tag_w;
            tag_w.id = iter_tag->first;
            tag_w.pose = iter_tag->second;
            tag_w.family = tag_family_;
            tag_w.size = tag_size_;
            // Other parameters are not needed
            // std::cout<<tag_w.id<<" "<<tag_w.pose.position.x<<std::endl;

            tags_w_prior_.push_back(tag_w);
        }
        // std::cout <<"map: "<<tags_prior_info.size()<< "vector: " << tags_w_prior_.size() << std::endl;
    }

    void TagMap::UpdateCurrentCamPose(const geometry_msgs::Pose &pose)
    {
        if (!last_cam_pose_valid_)
        {
            last_cam_pose_ = PoseMsgToIsometry3d(pose);
            last_cam_pose_valid_ = true;
            // return Eigen::Isometry3d::Identity();
        }
        // get Twc curr
        current_cam_pose_ = PoseMsgToIsometry3d(pose);

        // get velocity Tc1c2 = Tc1w * Twc2
        Eigen::Matrix3d last_cam_pose_inv_rot = last_cam_pose_.rotation().transpose();
        Eigen::Vector3d last_cam_pose_inv_t = -last_cam_pose_inv_rot * last_cam_pose_.translation().matrix();
        Eigen::Isometry3d last_cam_pose_inv = Eigen::Isometry3d::Identity();
        last_cam_pose_inv.rotate(last_cam_pose_inv_rot);
        last_cam_pose_inv.pretranslate(last_cam_pose_inv_t);
        cam_velocity_ = last_cam_pose_inv * current_cam_pose_;

        last_cam_pose_ = current_cam_pose_;

        if (!velocity_valid_)
        {
            velocity_valid_ = true;
            
        }
       
    }

    std::vector<Apriltag>::const_iterator FindById(int id, const std::vector<Apriltag> &tags)
    {
        return std::find_if(tags.cbegin(), tags.cend(),
                            [&id](const Apriltag &tag)
                            { return id == tag.id; });
    }

    std::vector<Apriltag>::iterator FindById(int id, std::vector<Apriltag> &tags)
    {
        return std::find_if(tags.begin(), tags.end(),
                            [&id](const Apriltag &tag)
                            { return id == tag.id; });
    }

} // namespace aprilslam

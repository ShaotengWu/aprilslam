#include "aprilslam/mapper_node.h"
#include "aprilslam/utils.h"

namespace aprilslam
{
    MapperNode::MapperNode(const ros::NodeHandle &nh, const std::string &frame_id)
        : nh_(nh),
          sub_tags_(nh_.subscribe("/aprilslam_apriltags", 1, &MapperNode::TagsCb, this)),
          sub_cinfo_(nh_.subscribe("camera_info", 1, &MapperNode::CinfoCb, this)),
          frame_id_(frame_id),
          mapper_(0.04, 1),
          map_("36h11", 0.4),
          tag_viz_(nh, "apriltags_map"),
          pose_cnt_(0),
          key_frame_interval_(3)

    {

        nh_.getParam(nh_.getNamespace() + "/mapper/use_tag_prior_info", use_tag_prior_info_);
        nh_.getParam(nh_.getNamespace() + "/mapper/tag_prior_info_path", tag_prior_info_path_);

        if (use_tag_prior_info_)
        {
            ROS_INFO("\033[1;32m----> Use prior information of apriltags. \033[0m");
            ROS_INFO("\033[1;32m----> The yaml file path: %s \033[0m", tag_prior_info_path_.c_str());
            tag_prior_info_node_ = YAML::LoadFile(tag_prior_info_path_);

            if (!tag_prior_info_node_["tags"].IsDefined())
            {
                ROS_ERROR("Invalid yaml file!");
                ROS_ERROR("Do not use prior information of apriltags.");
                exit(0);
            }
            else if (tag_prior_info_node_["tags"].IsNull())
            {
                ROS_ERROR("No tags information in yaml file!");
                ROS_ERROR("Do not use prior information of apriltags. ");
                exit(0);
            }
            else
            {
                size_t tags_num = tag_prior_info_node_["tags"].size();
                ROS_INFO("\033[1;32m----> Valid yaml file loaded.\033[0m");
                ROS_INFO("\033[1;32m----> Prior apriltags number: %d \033[0m", tags_num);

                // load prior information
                YAML::const_iterator tag_iter_begin = tag_prior_info_node_["tags"].begin();
                YAML::const_iterator tag_iter_end = tag_prior_info_node_["tags"].end();
                for (YAML::const_iterator tag_iter = tag_iter_begin; tag_iter != tag_iter_end; tag_iter++)
                {
                    YAML::Node tag_node = *tag_iter;
                    size_t id = tag_node["id"].as<size_t>();
                    std::vector<double> translation = tag_node["translation"].as<std::vector<double>>();
                    std::vector<double> rotation = tag_node["rotation"].as<std::vector<double>>();

                    geometry_msgs::Pose tag_prior_pose;
                    tag_prior_pose.position.x = translation[0];
                    tag_prior_pose.position.y = translation[1];
                    tag_prior_pose.position.z = translation[2];
                    tag_prior_pose.orientation.x = rotation[0];
                    tag_prior_pose.orientation.y = rotation[1];
                    tag_prior_pose.orientation.z = rotation[2];
                    tag_prior_pose.orientation.w = rotation[3];

                    tag_prior_poses_.insert(std::pair<size_t, geometry_msgs::Pose>(id, tag_prior_pose));
                }
                mapper_.UpdateTagsPriorInfo(tag_prior_poses_);
                map_.UpdateTagsPriorInfo(tag_prior_poses_);
            }
        }
        else
        {
            ROS_INFO(" Do not use prior information of apriltags. ");
        }

        tag_viz_.SetColor(aprilslam::GREEN);
        tag_viz_.SetAlpha(0.75);

        pub_cam_trajectory_ = nh_.advertise<nav_msgs::Path>("cam_trajectory", 1);
        pub_obj_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud>("object_points", 1);
        cam_trajectory_.header.frame_id = frame_id_;
    }

    void MapperNode::TagsCb(const aprilslam::ApriltagsConstPtr &tags_c_msg)
    {
        // Do nothing if no detection, this prevents checking in the following steps
        // std::cout << "newly detected tags: ";
        // for (int it = 0; it < tags_c_msg->apriltags.size(); it++)
        // {
        //     std::cout << tags_c_msg->apriltags[it].id << std::endl;
        //     std::cout << tags_c_msg->apriltags[it].corners[0]
        //               << tags_c_msg->apriltags[it].corners[1]
        //               << tags_c_msg->apriltags[it].corners[2]
        //               << tags_c_msg->apriltags[it].corners[3]
        //               << std::endl;
        // }
        // std::cout << std::endl;
        // if(pose_cnt_++ % key_frame_interval_ != 0)
        // {
        //     ROS_INFO("keyframe jump");
        //     return;
        // }
        // if (tags_c_msg->apriltags.size() <= 2 || pose_cnt_++ % key_frame_interval_ != 0)
        // {
        //     ROS_INFO("keyframe jump");
        //     return;
        // }
        // pose_cnt_++;

        if (tags_c_msg->apriltags.empty())
        {
            ROS_WARN_THROTTLE(1, "No tags detected.");
            return;
        }
        // Do nothing if camera info not received
        if (!model_.initialized())
        {
            ROS_WARN_THROTTLE(1, "No camera info received");
            return;
        }
        mapper_.InitCameraParams(model_.fullIntrinsicMatrix(), model_.distortionCoeffs());

        // Do nothing if there are no good tags close to the center of the image
        std::vector<Apriltag> tags_c_good;
        tags_c_good.clear();
        if (!GetGoodTags(tags_c_msg->apriltags, &tags_c_good))
        {
            ROS_WARN_THROTTLE(1, "No good tags detected.");
            return;
        }
        // std::cout << "good detected tags: ";
        // for (int it = 0; it < tags_c_good.size(); it++)
        // {
        //     std::cout << tags_c_good[it].id << " ";
        // }
        // std::cout << std::endl;
        // Initialize map by adding the first tag that is not on the edge of the image
        if (!map_.init())
        {
            map_.AddFirstTag(tags_c_good.front());
            ROS_INFO("AprilMap initialized.");
        }
        // Do nothing if no pose can be estimated
        // 到这里tags的pose都是在相机坐标系下
        geometry_msgs::Pose pose;
        if (!map_.EstimatePose(tags_c_good, model_.fullIntrinsicMatrix(), model_.distortionCoeffs(), &pose))
        {
            ROS_WARN_THROTTLE(1, "No 2D-3D correspondence.");
            return;
        }
        sensor_msgs::PointCloud obj_pointcloud_viz = map_.obj_pointcloud_viz();
        obj_pointcloud_viz.header.frame_id = frame_id_;
        obj_pointcloud_viz.header.stamp = ros::Time::now();
        pub_obj_pointcloud_.publish(obj_pointcloud_viz);

        // Now that with the initial pose calculated, we can do some mapping
        /* -------------------------------------------------------------------------- */
        /*          toggle to add/remove const velocity motion model factors          */
        /* -------------------------------------------------------------------------- */
        mapper_.AddPose(pose, cam_velocity_);
        // mapper_.AddPose(pose);

        mapper_.AddFactors(tags_c_good);
        // This will only add new landmarks
        mapper_.AddLandmarks(tags_c_good);
        if (mapper_.init())
        {

            mapper_.Optimize(10);
            // // Get latest estimates from mapper and put into map
            mapper_.Update(&map_, &pose);

            // ! Move clear() into update()
            // Prepare for next iteration
            // mapper_.Clear();

            // update current pose to tag_map

            // mapper_.BatchOptimize();
            // mapper_.BatchUpdate(&map_, &pose);

            cam_velocity_ = map_.getVelocity(); //! return planar relative velocity
            map_.UpdateCurrentCamPose(pose);
            // CALL kalman filter here

            geometry_msgs::PoseStamped cam_pose_stamped;
            cam_pose_stamped.header.stamp = tags_c_msg->header.stamp;
            cam_pose_stamped.header.frame_id = frame_id_;
            cam_pose_stamped.pose = pose;
            cam_trajectory_.poses.push_back(cam_pose_stamped);
            pub_cam_trajectory_.publish(cam_trajectory_);
        }
        else
        {
            // No projectionFactor will be added before initialization. Due to no tags in tags_w_all_
            mapper_.InitCameraParams(model_.fullIntrinsicMatrix(), model_.distortionCoeffs());

            // This will add first landmark at origin and fix scale for first pose and
            // first landmark
            mapper_.Initialize(map_.first_tag());
        }

        // Publish camera to world transform
        std_msgs::Header header;
        header.stamp = tags_c_msg->header.stamp;
        header.frame_id = frame_id_;

        geometry_msgs::Vector3 translation;
        translation.x = pose.position.x;
        translation.y = pose.position.y;
        translation.z = pose.position.z;

        std::cout.precision(4);
        std::cout.width(6);
        std::cout.setf(std::ios::left);
        std::cout << translation.x << "\t" << translation.y << "\t" << translation.z << std::endl;

        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header = header;
        transform_stamped.child_frame_id = "camera";
        transform_stamped.transform.translation = translation;
        transform_stamped.transform.rotation = pose.orientation;

        tf_broadcaster_.sendTransform(transform_stamped);

        // Publish visualisation markers
        tag_viz_.SetColor(aprilslam::GREEN);
        tag_viz_.PublishApriltagsMarker(map_.tags_w(), frame_id_, tags_c_msg->header.stamp);

        // Publish prior tag info
        // ROS_INFO("size of prior tags: %f", map_.tags_w_prior().size());
        // std::cout<<map_.tags_w_prior().size()<<std::endl; 7
        std::vector<aprilslam::Apriltag> tags_w_prior = map_.tags_w_prior();
        tag_viz_.SetColor(aprilslam::YELLOW);
        tag_viz_.PublishPriorApriltagsMarker(tags_w_prior, frame_id_, tags_c_msg->header.stamp);
    }

    void MapperNode::CinfoCb(const sensor_msgs::CameraInfoConstPtr &cinfo_msg)
    {
        if (model_.initialized())
        {
            sub_cinfo_.shutdown();
            ROS_INFO("%s: %s", nh_.getNamespace().c_str(), "Camera initialized");
            return;
        }
        model_.fromCameraInfo(cinfo_msg);
    }

    bool MapperNode::GetGoodTags(const std::vector<Apriltag> tags_c, std::vector<Apriltag> *tags_c_good)
    {
        std::vector<Apriltag> tags_c_tmp = tags_c;

        std::sort(tags_c_tmp.begin(), tags_c_tmp.end(), [](Apriltag tag1, Apriltag tag2)
                  { return tag1.id < tag2.id; });
        for (const Apriltag &tag_c : tags_c_tmp)
        {
            std::cout << tag_c.id << " ";
            if (tags_c_good->size() >= 6)
                break;
            tags_c_good->push_back(tag_c);
            if (IsInsideImageCenter(tag_c.center.x, tag_c.center.y,
                                    model_.cameraInfo().width,
                                    model_.cameraInfo().height, 5))
            {
                // tags_c_good->push_back(tag_c);
            }
        }
        return !tags_c_good->empty();
    }

} // namespace aprilslam

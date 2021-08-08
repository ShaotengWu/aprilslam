#ifndef APRILTAG_ROS_MAPPER_H_
#define APRILTAG_ROS_MAPPER_H_

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/slam/RangeFactor.h>
#include <opencv2/core/eigen.hpp>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <aprilslam/Apriltags.h>
#include "aprilslam/tag_map.h"

namespace aprilslam
{
    using BearingRange3D = gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>;
    /**
     * @brief Backend optimization class.
     * 
     */
    class Mapper
    {
    public:
        ///@brief Pose number counter .
        static int pose_cnt;

        /**
         * @brief Construct a new Mapper object
         * 
         * @param relinearize_thresh iSAM2 parameter
         * @param relinearize_skip iSAM2 parameter
         */
        Mapper(double relinearize_thresh, int relinearize_skip);

        /**
         * @brief Return whether init yet.
         */
        bool init() const { return init_; }

        /**
         * @brief Initialize camera model.
         * 
         * @param intrinsic Camera intrinsics. 
         * @param distCoeff Camera distortion coefficients.
         */
        void InitCameraParams(const cv::Matx33d &intrinsic, const cv::Mat &distCoeff);

        /**
         * @brief iSAM 2 Optimization
         * 
         * @param num_iterations optimization iterations.
         */
        void Optimize(int num_iterations = 1);

        /**
         * @brief Update newest optimization results to tag_map and current pose.
         * 
         * @param map tag_map in mapper_node
         * @param pose current_pose in mapper_node
         */
        void Update(aprilslam::TagMap *map, geometry_msgs::Pose *pose);

        /**
         * @brief Add pose factor
         * 
         * @param pose current camera pose.
         */
        void AddPose(const geometry_msgs::Pose &pose);

        /**
         * @brief Add pose factor and const velocity motion model constraint
         * 
         * @param pose current camera pose.
         * @param vel newest velocity.
         */
        void AddPose(const geometry_msgs::Pose &pose, const geometry_msgs::Pose &vel); // overload

        /**
         * @brief Add factors.
         * @details Add factors. Including:
         * @details  BetweenFactor -- Camera and Apriltags.
         * @details  GenericProjectionFactor --  Visual projection of corners
         * @details  RangeFactor -- Apriltag size constraint between its corner and center
         * 
         * @param tags_c filtered detections.
         */
        void AddFactors(const std::vector<aprilslam::Apriltag> &tags_c);

        /**
         * @brief Only add init esitimate and prior factor of new landmarks
         * 
         * @param tags_c Current detections including old and new landmarks. 
         */
        void AddLandmarks(const std::vector<aprilslam::Apriltag> &tags_c);

        /**
         * @brief Add landmark prior factor
         * 
         * @param tag_id Landmark id to query prior info
         */
        void AddLandmarkPrior(const size_t tag_id);
        /**
         * @brief Initialization. If there is any valid landmark estimation, set init_ true.
         * 
         * @param tag_w First tag. The pose of the tag is in fixed world(tag) frame.
         */
        void Initialize(const Apriltag &tag_w);

        /**
         * @brief Clear factor graph and initial estimation
         * @note Important step while using iSAM 2.
         * 
         */
        void Clear();

        /**
         * @brief Load prior info
         * 
         * @param tag_prior_poses Apriltags prior info extracted from yaml. 
         */
        void UpdateTagsPriorInfo(const std::map<int, geometry_msgs::Pose> tag_prior_poses);

        // Use batch optimization to check data association. No use right now.
        void BatchOptimize() = delete;
        void BatchUpdate(aprilslam::TagMap *map, geometry_msgs::Pose *pose) = delete;

    private:
        /**
         * @brief Add init esitimate and prior factor of tag_w
         * 
         * @param tag_w Landmark to be added.
         * @param pose Initial estimate of the landmark.
         */
        void AddLandmark(const aprilslam::Apriltag &tag_w,
                         const gtsam::Pose3 &pose);

        /// @brief initial flag. If there is any valid landmark estimation, set init_ true.
        bool init_;

        /// @brief For stable initialization. Batch optimize the first min_pose_count_ frames. After initial optimization, init_opt_ is true.
        bool init_opt_;

        /// @brief For stable initialization. Batch optimize the first min_pose_count_ frames. After initial optimization, init_opt_ is true.
        int min_pose_count_;

        /// @brief For stable optimization, new landmark should be observed more than obsv_thr_ times before being added into factor graph.
        int obsv_thr_;

        /// @brief iSAM 2 parameters. For more details, refer to gtsam offcial documentation(https://gtsam-jlblanco-docs.readthedocs.io/en/latest/_static/doxygen/html/modules.html#http://).
        gtsam::ISAM2Params params_;

        /// @brief iSAM 2 data structure. For more details, refer to gtsam offcial documentation(https://gtsam-jlblanco-docs.readthedocs.io/en/latest/_static/doxygen/html/modules.html#http://).
        gtsam::ISAM2 isam2_;

        /// @brief Factor graph. For more details, refer to gtsam offcial documentation(https://gtsam-jlblanco-docs.readthedocs.io/en/latest/_static/doxygen/html/modules.html#http://).
        gtsam::NonlinearFactorGraph graph_;

        /// @brief Initial estiamtes of varibles in iSAM2. For more details, refer to gtsam offcial documentation(https://gtsam-jlblanco-docs.readthedocs.io/en/latest/_static/doxygen/html/modules.html#http://).
        gtsam::Values initial_estimates_;

        /// @brief Results.
        gtsam::Values results_;

        /// @brief Camera pose.
        gtsam::Pose3 pose_;

        /// @brief Camera intrinsics.
        gtsam::Cal3_S2::shared_ptr K_;

        /// @brief noise of relative pose estimation during Apriltag detection.
        gtsam::noiseModel::Diagonal::shared_ptr tag_noise_;

        /// @brief noise of prior information.
        gtsam::noiseModel::Diagonal::shared_ptr small_noise_;

        /// @brief noise of Apriltag size constraint.
        gtsam::noiseModel::Diagonal::shared_ptr tag_size_noise_;

        /// @brief noise of const motion model.
        gtsam::noiseModel::Diagonal::shared_ptr motion_model_noise_;

        /// @brief noise of camera measurement.
        gtsam::noiseModel::Isotropic::shared_ptr measurement_noise_;

        ///@brief Huber kernel for noise of relative pose estimation during Apriltag detection.
        gtsam::noiseModel::Base::shared_ptr tag_noise_huber_;

        ///@brief Huber kernel for noise of prior information.
        gtsam::noiseModel::Base::shared_ptr small_noise_huber_;

        ///@brief Huber kernel for noise of Apriltag size constraint.
        gtsam::noiseModel::Base::shared_ptr tag_size_noise_huber_;

        ///@brief Huber kernel for noise of const motion model.
        gtsam::noiseModel::Base::shared_ptr measurement_noise_huber_;

        ///@brief All apriltags id in SLAM framework.
        std::set<int> all_ids_;

        ///@brief Key: Apriltags id  Value: Pose in camera frame
        std::map<int, aprilslam::Apriltag> all_tags_c_;

        ///@brief Key: Apriltags id  Value: Pose in world(tag) frame
        std::map<int, aprilslam::Apriltag> all_tags_w_;

        ///@brief Key: Apriltags id  Value: Observation times
        std::map<int, std::vector<int>> tags_obsv_;

        ///@brief Key: Apriltags id  Value: If the landmark is already in iSAM2
        std::map<int, bool> tags_in_isam_;

        ///@brief Key: Apriltags id  Value: Prior pose
        std::map<int, geometry_msgs::Pose> tag_prior_poses_;
    };

    gtsam::Pose3 FromGeometryPose(const geometry_msgs::Pose &pose);
} // namespace aprilslam

#endif // APRILSLAM_MAPPER_H_

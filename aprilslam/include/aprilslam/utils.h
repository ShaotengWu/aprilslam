#ifndef APRILSLAM_UTILS_H_
#define APRILSLAM_UTILS_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/core.hpp>

namespace aprilslam {

/**
 * @brief Good detection filter condition
 * 
 * @param x tag detection left upper point
 * @param y tag detection left upper point
 * @param w tag detection pixel width
 * @param h tag detection pixel height
 * @param k center ratio
 */
bool IsInsideImageCenter(double x, double y, int w, int h, double k);

/**
 * @brief Set the Pose from Eigen data structure
 * 
 * @param [out] pose 
 * @param [in] wxyz 
 * @param [in] xyz 
 */
void SetPose(geometry_msgs::Pose* pose,
             const Eigen::Quaterniond& wxyz = Eigen::Quaterniond(1, 0, 0, 0),
             const Eigen::Vector3d& xyz = Eigen::Vector3d(0, 0, 0));

/**
 * @brief Set the Position from Eigen data structure
 * 
 * @param [out] pos 
 * @param [in] xyz 
 */
void SetPosition(geometry_msgs::Point* pos, const Eigen::Vector3d& xyz);

/**
 * @brief Set the Position from double
 * 
 * @param [out] pos 
 * @param [in] x 
 * @param [in] y 
 * @param [in] z 
 */
void SetPosition(geometry_msgs::Point* pos, double x = 0, double y = 0,
                 double z = 0);
/**
 * @brief Set the Orientation from Eigen data structure
 * 
 * @param [out] quat 
 * @param [in] wxyz 
 */
void SetOrientation(geometry_msgs::Quaternion* quat,
                    const Eigen::Quaterniond& wxyz);

/**
 * @brief Set the Orientation from double
 * 
 * @param [out] quat 
 * @param [in] w 
 * @param [in] x 
 * @param [in] y 
 * @param [in] z 
 */
void SetOrientation(geometry_msgs::Quaternion* quat, double w = 1, double x = 0,
                    double y = 0, double z = 0);

/**
 * @brief Convert corners from 2d to 3d
 * 
 * @param corners Input as 2d and output as 3d. 
 * @param pose Pose for inverse-projection
 * @param tag_size Tag size for scale
 */
void SetCorners(std::vector<geometry_msgs::Point>* corners,
                const geometry_msgs::Pose& pose, double tag_size);

/**
 * @brief Convert cv::mat rotation vector to Eigen Quarterniond.
 * 
 * @param r cv::mat rotation vector
 * @return Eigen::Quaterniond 
 */
Eigen::Quaterniond RodriguesToQuat(const cv::Mat& r);

/**
 * @brief Convert geometry::Pose to Eigen Isometry 3d
 * 
 * @param pose geometry::Pose
 * @return Eigen::Isometry3d 
 */
Eigen::Isometry3d PoseMsgToIsometry3d(const geometry_msgs::Pose &pose);

/**
 * @brief Convert Eigen Isometry 3d to geometry::Pose 
 * 
 * @param pose3d Eigen Isometry 3d 
 * @return geometry_msgs::Pose 
 */
geometry_msgs::Pose Isometry3dToPoseMsg(const Eigen::Isometry3d &pose3d);


}  // namespace aprilslam

#endif  // APRILSLAM_UTILS_H_

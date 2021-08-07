#include "aprilslam/utils.h"

namespace aprilslam
{

    bool IsInsideImageCenter(double x, double y, int w, int h, double k)
    {
        return (x > w / k) && (y > h / k) && (x < (w - w / k)) && (y < (h - h / k));
    }

    void SetPose(geometry_msgs::Pose *pose, const Eigen::Quaterniond &wxyz, const Eigen::Vector3d &xyz)
    {
        SetPosition(&pose->position, xyz);
        SetOrientation(&pose->orientation, wxyz);
    }

    void SetPosition(geometry_msgs::Point *pos, const Eigen::Vector3d &xyz)
    {
        SetPosition(pos, xyz(0), xyz(1), xyz(2));
    }

    void SetPosition(geometry_msgs::Point *pos, double x, double y, double z)
    {
        pos->x = x;
        pos->y = y;
        pos->z = z;
    }

    void SetOrientation(geometry_msgs::Quaternion *quat,
                        const Eigen::Quaterniond &wxyz)
    {
        SetOrientation(quat, wxyz.w(), wxyz.x(), wxyz.y(), wxyz.z());
    }

    void SetOrientation(geometry_msgs::Quaternion *quat, double w, double x,
                        double y, double z)
    {
        quat->w = w;
        quat->x = x;
        quat->y = y;
        quat->z = z;
    }

    void SetCorners(std::vector<geometry_msgs::Point> *corners,
                    const geometry_msgs::Pose &pose, double tag_size)
    {
        std::vector<geometry_msgs::Point> &w_corners = *corners;
        double a = tag_size / 2;
        Eigen::Quaterniond w_Q_b(pose.orientation.w,
                                 pose.orientation.x,
                                 pose.orientation.y,
                                 pose.orientation.z);
        Eigen::Vector3d w_T_b(pose.position.x, pose.position.y, pose.position.z);
        // std::cout<<"Set Corners - Tag "<<""<<"position: "<<w_T_b.transpose()<<std::endl;
        const std::vector<Eigen::Vector3d> b_cs = {{-a, -a, 0}, {a, -a, 0}, {a, a, 0}, {-a, a, 0}};
        for (size_t ip = 0; ip < b_cs.size(); ip++)
        {
            Eigen::Vector3d w_corner = w_Q_b.matrix() * b_cs[ip] + w_T_b;
            w_corners[ip].x = w_corner.x();
            w_corners[ip].y = w_corner.y();
            w_corners[ip].z = w_corner.z();
        }
    }

    Eigen::Quaterniond RodriguesToQuat(const cv::Mat &rvec)
    {
        Eigen::Vector3d r(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
        // Copied from kr_math pose
        const double rn = r.norm();
        Eigen::Vector3d rnorm(0.0, 0.0, 0.0);
        if (rn > std::numeric_limits<double>::epsilon() * 10)
            rnorm = r / rn;
        return Eigen::Quaterniond(Eigen::AngleAxis<double>(rn, rnorm));
    }
    Eigen::Isometry3d PoseMsgToIsometry3d(const geometry_msgs::Pose &pose)
    {
        Eigen::Quaterniond q(pose.orientation.w,
                             pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z);
        Eigen::Vector3d t(pose.position.x,
                          pose.position.y,
                          pose.position.z);
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.rotate(q.matrix());
        T.pretranslate(t);
        return T;
    }

    geometry_msgs::Pose Isometry3dToPoseMsg(const Eigen::Isometry3d &pose3d)
    {
        Eigen::Quaterniond q(pose3d.rotation());
        Eigen::Vector3d t(pose3d.translation());

        geometry_msgs::Pose pose;
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        pose.position.x = t.x();
        pose.position.y = t.y();
        pose.position.z = t.z();

        return pose;
    }

} // namespace aprilslam

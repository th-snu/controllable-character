#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "bvh-loader/BVHReader.h"

// Implementation of interpolation algorithm
Eigen::Quaterniond bvh_to_quaternion(const double x, const double y, const double z);
Eigen::Quaterniond bvh_to_quaternion(const Eigen::Vector3d& euler);

Eigen::Vector3d quaternion_to_bvh(const Eigen::Quaterniond& q);

double get_y_rotation(Eigen::Quaterniond curr, Eigen::Quaterniond goal);

Eigen::Quaterniond slerp_frames_orientation(Motion frames);

Eigen::Vector3d resample_pos(const Motion& motion, double frame);

Eigen::Quaterniond resample_ori(const Motion& motion, double frame);

Motion expand_motion(Motion motion, double frames);

Eigen::Quaterniond slerp_frames_orientation(Motion frames);

Motion interpolate_motion(Motion old_motion, Motion next_motion, bool time_shift);


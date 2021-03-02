#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "bvh-loader/BVHReader.h"

// Implementation of interpolation algorithm
Eigen::Quaterniond bvh_to_quaternion(const Eigen::Vector3d& euler);

Eigen::Vector3d quaternion_to_bvh(const Eigen::Quaterniond& q);

Motion expand_motion(Motion motion, double frames);

Eigen::Quaterniond slerp_frames_orientation(Motion frames);

Motion interpolate_motion(Motion old_motion, Motion next_motion, bool time_shift);


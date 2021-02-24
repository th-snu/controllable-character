#include "Interpolation.hpp"

Eigen::Quaterniond EulerToQuaternion(Eigen::Vector3d euler){
    using namespace Eigen;
    // float roll = 1.5707, pitch = 0, yaw = 0.707;    
    Quaterniond q;
    q = AngleAxisd(euler[0], Vector3d::UnitX())
        * AngleAxisd(euler[1], Vector3d::UnitY())
        * AngleAxisd(euler[2], Vector3d::UnitZ());
    return q;
}

Eigen::Vector3d EulerToQuaternionToEuler(Eigen::Quaterniond q){
    return q.toRotationMatrix().eulerAngles(0, 1, 2);
}
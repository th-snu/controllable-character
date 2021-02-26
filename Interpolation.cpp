#include "Interpolation.hpp"
#include <math.h>
#define PI 3.14159265358979323846

Eigen::Quaterniond bvh_to_quaternion(const Eigen::Vector3d& euler){
    using namespace Eigen;
    // float roll = 1.5707, pitch = 0, yaw = 0.707;    
    Quaterniond q;
    q = AngleAxisd(euler[2]*PI/180.0, Vector3d::UnitY())
        * AngleAxisd(euler[1]*PI/180.0, Vector3d::UnitX())
        * AngleAxisd(euler[0]*PI/180.0, Vector3d::UnitZ());
    return q;
}

Eigen::Vector3d quaternion_to_bvh(const Eigen::Quaterniond& q){
    auto angle = q.toRotationMatrix().eulerAngles(1, 0, 2)*(180/PI);
    return Eigen::Vector3d(angle[2], angle[1], angle[0]);
}

Motion interpolate_motion(Motion old_motion, Motion next_motion, int& interpolated_frame, int& next_motion_frame){
    using namespace Eigen;

    old_motion = Motion(old_motion.begin(), old_motion.begin() + interpolated_frame);

    vector<double> last_frame = old_motion.back();
    vector<double> next_frame = next_motion.front();

    Vector3d last_pos(last_frame[0], last_frame[1], last_frame[2]);
    Vector3d next_pos(next_frame[0], next_frame[1], next_frame[2]);
    Quaterniond last_ori = bvh_to_quaternion(Vector3d(last_frame[3], last_frame[4], last_frame[5]));
    Quaterniond next_ori = bvh_to_quaternion(Vector3d(next_frame[3], next_frame[4], next_frame[5]));

    // Rotational Axis : y
    auto vs = last_ori.vec();
    auto ws = last_ori.w();
    auto v0 = next_ori.vec();
    auto w0 = next_ori.w();
    auto yvec = Vector3d::UnitY();

    double alpha = atan((ws * w0 + vs.dot(v0)) /
        (ws * yvec.dot(v0) + w0 * vs.dot(yvec) + vs.dot(yvec.cross(v0))));
    Quaterniond alpha_plus = Quaterniond(AngleAxisd(-alpha + PI/2, yvec)) * next_ori;
    Quaterniond alpha_minus = Quaterniond(AngleAxisd(-alpha - PI/2, yvec)) * next_ori;
    double rot_angle = last_ori.dot(alpha_plus) > last_ori.dot(alpha_minus) ? -alpha + PI/2 : -alpha - PI/2;
    Quaterniond rot(AngleAxisd(rot_angle, yvec));

    auto qd = (rot * next_ori).inverse()*last_ori;
    cout << 2 * atan2(qd.vec().norm(), qd.w()) << endl;

    Vector3d trans = last_pos - rot * next_pos;
    // Remove y component of translation
    trans[1] = 0;
    
    // character sometimes moves in wrong direction, rotation is potentially applied wrongly
    for (auto& frame : next_motion){
        Vector3d pos(frame[0], frame[1], frame[2]);
        Quaterniond ori = bvh_to_quaternion(Vector3d(frame[3], frame[4], frame[5]));
        pos = rot * pos + trans;
        ori *= rot;

        Vector3d new_ori = quaternion_to_bvh(ori);
        frame[0] = pos[0]; frame[1] = pos[1]; frame[2] = pos[2];
        frame[3] = new_ori[0]; frame[4] = new_ori[1]; frame[5] = new_ori[2];
    }

    interpolated_frame = old_motion.size();
    next_motion_frame = old_motion.size();

    old_motion.insert(old_motion.end(), next_motion.begin(), next_motion.end());
    
    return old_motion;
}

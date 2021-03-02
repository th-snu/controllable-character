#include "Interpolation.hpp"
#include <math.h>
#define PI 3.14159265358979323846

Eigen::Quaterniond bvh_to_quaternion(const double x, const double y, const double z){
    return bvh_to_quaternion(Eigen::Vector3d(x, y, z));
}

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

double get_y_rotation(Eigen::Quaterniond curr, Eigen::Quaterniond goal){
    using namespace Eigen;

    auto vs = goal.vec();
    auto ws = goal.w();
    auto v0 = curr.vec();
    auto w0 = curr.w();
    auto yvec = Vector3d::UnitY();

    double alpha = atan((ws * w0 + vs.dot(v0)) /
        (ws * yvec.dot(v0) + w0 * vs.dot(yvec) + vs.dot(yvec.cross(v0))));
    Quaterniond alpha_plus = Quaterniond(AngleAxisd(-alpha + PI/2, yvec)) * curr;
    Quaterniond alpha_minus = Quaterniond(AngleAxisd(-alpha - PI/2, yvec)) * curr;
    return 2 * (goal.dot(alpha_plus) > goal.dot(alpha_minus) ? -alpha + PI/2 : -alpha - PI/2);
}

Eigen::Quaterniond slerp_frames_orientation(Motion frames){
    Eigen::Quaterniond ori = bvh_to_quaternion(Eigen::Vector3d(frames[0][3], frames[0][4], frames[0][5]));
    for (int i = 1; i < frames.size(); i++){
        ori = ori.slerp((float)1 / (i + 1), bvh_to_quaternion(Eigen::Vector3d(frames[i][3], frames[i][4], frames[i][5])));
    }
    return ori;
}

// uniform resampling
Eigen::Vector3d resample_pos(const Motion& motion, double frame){
    vector<double> d1 = motion[(int)frame];
    vector<double> d2 = motion[(int)frame + 1];
    Eigen::Vector3d b1 = Eigen::Vector3d(d1[0], d1[1], d1[2]);
    Eigen::Vector3d b2 = Eigen::Vector3d(d2[0], d2[1], d2[2]);
    double t = frame - (int)frame;

    return b1 * (1-t) + b2 * t;
}

Eigen::Quaterniond resample_ori(const Motion& motion, double frame){
    vector<double> d1 = motion[(int)frame];
    vector<double> d2 = motion[(int)frame + 1];
    Eigen::Quaterniond b1 = bvh_to_quaternion(d1[3], d1[4], d1[5]);
    Eigen::Quaterniond b2 = bvh_to_quaternion(d2[3], d2[4], d2[5]);
    
    return b1.slerp(frame - (int)frame, b2);
}

// old_motion and next_motion must have at least 10 frames
Motion interpolate_motion(Motion old_motion, Motion next_motion, bool time_shift){
    using namespace Eigen;

    if(old_motion.size() > 10){
        old_motion = Motion(old_motion.begin(), old_motion.begin() + 10);
    }

    Motion last_frames = old_motion;
    Motion next_frames = Motion(next_motion.begin(), next_motion.begin() + 10);

    Vector3d last_pos(last_frames[0][0], last_frames[0][1], last_frames[0][2]);
    Vector3d next_pos(next_frames[0][0], next_frames[0][1], next_frames[0][2]);

    Quaterniond last_ori = slerp_frames_orientation(last_frames);
    Quaterniond next_ori = slerp_frames_orientation(next_frames);

    // uniform resampling
    Quaterniond rot(AngleAxisd(get_y_rotation(next_ori, last_ori), Vector3d::UnitY()));

    Vector3d trans = last_pos - rot * next_pos;
    // Remove y component of translation
    trans[1] = 0;

    for (auto& frame : next_motion){
        Vector3d pos(frame[0], frame[1], frame[2]);
        Quaterniond ori = bvh_to_quaternion(Vector3d(frame[3], frame[4], frame[5]));
        pos = rot * pos + trans;
        ori *= rot;

        Vector3d new_ori = quaternion_to_bvh(ori);
        frame[0] = pos[0]; frame[1] = pos[1]; frame[2] = pos[2];
        frame[3] = new_ori[0]; frame[4] = new_ori[1]; frame[5] = new_ori[2];
    }

    next_frames = Motion(next_motion.begin(), next_motion.begin() + 10);

    // interpolate old motion with new motion
    if (!time_shift){
        double old_delta = 0.0;
        double new_delta = 0.0;

        for (int i = 0; i < 9; i++){
            old_delta += Vector3d(last_frames[i+1][0] - last_frames[i][0], last_frames[i+1][1] - last_frames[i][1], last_frames[i+1][2] - last_frames[i][2]).norm();
            new_delta += Vector3d(next_frames[i+1][0] - next_frames[i][0], next_frames[i+1][1] - next_frames[i][1], next_frames[i+1][2] - next_frames[i][2]).norm();
        }

        for (int i = 0; i < 10; i++){
            vector<double> frame;
            frame.push_back((last_frames[i][0] * i + next_frames[i][0] * (10-i)) / 10);
            frame.push_back((last_frames[i][1] * i + next_frames[i][1] * (10-i)) / 10);
            frame.push_back((last_frames[i][2] * i + next_frames[i][2] * (10-i)) / 10);

            int var_count = last_frames[0].size();
            for (int j = 3; j < var_count; j += 3){
                auto q1 = bvh_to_quaternion(last_frames[i][j], last_frames[i][j+1], last_frames[i][j+2]);
                auto q2 = bvh_to_quaternion(next_frames[i][j], next_frames[i][j+1], next_frames[i][j+2]);
                auto new_ori = quaternion_to_bvh(q1.slerp(0.1 * i, q2));
                frame.push_back(new_ori[0]);
                frame.push_back(new_ori[1]);
                frame.push_back(new_ori[2]);
            }
            old_motion[i] = frame;
        }
    }
    else {
        
    }

    old_motion.insert(old_motion.end(), next_motion.begin() + 10, next_motion.end());
    
    return old_motion;
}

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

vector<double> resample_ori(const Motion& motion, double frame){
    vector<double> d1 = motion[(int)frame];
    vector<double> d2 = motion[(int)frame + 1];

    vector<double> res;
    for (int idx = 3; idx < motion.size(); idx += 3){
        Eigen::Quaterniond b1 = bvh_to_quaternion(d1[idx], d1[idx+1], d1[idx+2]);
        Eigen::Quaterniond b2 = bvh_to_quaternion(d2[idx], d2[idx+1], d2[idx+2]);
        auto b_new = quaternion_to_bvh(b1.slerp(frame - (int)frame, b2));
        res.push_back(b_new[0]);
        res.push_back(b_new[1]);
        res.push_back(b_new[2]);
    }
    
    return res;
}

vector<double> resample_frame(const Motion& motion, double frame){
    Eigen::Vector3d res_pos = resample_pos(motion, frame);
    vector<double> res_vec(res_pos.data(), res_pos.data() + 3);
    vector<double> res_ori = resample_ori(motion, frame);
    res_vec.insert(res_vec.end(), res_ori.begin(), res_ori.end());

    return res_vec;
}

vector<double> interpolate_frame(const vector<double> frame1, const vector<double> frame2, double rate){
    vector<double> frame;

    frame.push_back(frame1[0] * (1.0 - rate) + frame2[0] * rate);
    frame.push_back(frame1[1] * (1.0 - rate) + frame2[1] * rate);
    frame.push_back(frame1[2] * (1.0 - rate) + frame2[2] * rate);

    int var_count = frame1.size();
    for (int j = 3; j < var_count; j += 3){
        auto q1 = bvh_to_quaternion(frame1[j], frame1[j+1], frame1[j+2]);
        auto q2 = bvh_to_quaternion(frame2[j], frame2[j+1], frame2[j+2]);
        auto new_ori = quaternion_to_bvh(q1.slerp(rate, q2));
        frame.push_back(new_ori[0]);
        frame.push_back(new_ori[1]);
        frame.push_back(new_ori[2]);
    }
    
    return frame;
}

// old_motion and next_motion must have at least 10 frames
Motion interpolate_motion(Motion old_motion, Motion next_motion, bool time_shift){
    using namespace Eigen;

    int sample_frames = 10;
    if(old_motion.size() < sample_frames) sample_frames = old_motion.size();

    if(old_motion.size() > sample_frames){
        old_motion = Motion(old_motion.begin(), old_motion.begin() + sample_frames);
    }

    Vector3d last_pos(old_motion[0][0], old_motion[0][1], old_motion[0][2]);
    Vector3d next_pos(next_motion[0][0], next_motion[0][1], next_motion[0][2]);

    // preprocess motion data instead so the beginning and the ending would have the same orientation
    auto old_frame = old_motion[sample_frames-1];
    auto next_frame = next_motion[0];
    Quaterniond last_ori = bvh_to_quaternion(Eigen::Vector3d(old_frame[3], old_frame[4], old_frame[5]));
    Quaterniond next_ori = bvh_to_quaternion(Eigen::Vector3d(next_frame[3], next_frame[4], next_frame[5]));

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

    double old_delta = 0.0;
    double new_delta = 0.0;

    Motion last_frames = old_motion;
    Motion next_frames = Motion(next_motion.begin(), next_motion.begin() + sample_frames);

    for (int i = 0; i < sample_frames - 1; i++){
        old_delta += Vector3d(last_frames[i+1][0] - last_frames[i][0], last_frames[i+1][1] - last_frames[i][1], last_frames[i+1][2] - last_frames[i][2]).norm();
        new_delta += Vector3d(next_frames[i+1][0] - next_frames[i][0], next_frames[i+1][1] - next_frames[i][1], next_frames[i+1][2] - next_frames[i][2]).norm();
    }

    // interpolate old motion with new motion
    if (!time_shift){
        for (int i = 0; i < sample_frames; i++)
            old_motion[i] = interpolate_frame(last_frames[i], next_frames[i], (double)i / sample_frames);

        old_motion.insert(old_motion.end(), next_motion.begin() + sample_frames, next_motion.end());
        return old_motion;
    }
    else {
        int next_sample_frames = old_delta * sample_frames / new_delta;
        int next_frames = next_motion.size();

        bool is_expanded = false;

        // ensure that next motion has enough length
        while (next_sample_frames > next_motion.size()){
            is_expanded = true;
            next_motion = interpolate_motion(next_motion, next_motion, false);
        }

        int interpolated_frames = (next_sample_frames + sample_frames) / 2;

        Motion resample_old;
        Motion resample_next;

        double old_rate = (double) sample_frames / interpolated_frames;
        double new_rate = (double) next_sample_frames / interpolated_frames;

        Motion interpolated_motion;
        for (int i = 0; i < interpolated_frames; i++){
            double t = (double) i / interpolated_frames;
            double resampled_t = (- 2 * pow(t, 3) + 3 * pow (t, 2)) + old_rate * (pow(t, 3) - 2 * pow(t, 2) + t) + new_rate * (pow(t, 3) - pow(t, 2));
            auto resampled_old_frame = resample_frame(old_motion, old_rate * resampled_t * sample_frames);
            auto resampled_next_frame = resample_frame(next_motion, new_rate * resampled_t * next_sample_frames);

            interpolated_motion.push_back(interpolate_frame(resampled_old_frame, resampled_next_frame, (double) i / interpolated_frames));
        }

        if (!is_expanded){
            interpolated_motion.insert(interpolated_motion.end(), next_motion.begin() + next_sample_frames, next_motion.end());
        }
    }
}

#include "Controller.hpp"

vector<vector<Motion>> Controller::walk_data = vector<vector<Motion>>();
vector<vector<Motion>> Controller::fastwalk_data = vector<vector<Motion>>();
vector<vector<Motion>> Controller::jog_data = vector<vector<Motion>>();
vector<vector<Motion>> Controller::run_data = vector<vector<Motion>>();

vector<Motion> Controller::stop_data = vector<Motion>();
vector<vector<Motion>> Controller::jump_data = vector<vector<Motion>>();

bool Controller::is_motion_loaded = false;

Motion extractMotion(const string& filename){
    BVHReader reader(filename);
    if (reader.loadFile()){
        return reader.getMotion();
    }
    else {
        cout << "error\n";
        return vector<vector<double>>();
    }
}

void Controller::draw(){
    for(int i = 0; i < root.size(); i++){
        root[i]->draw();
    }
}

Eigen::Vector3d Controller::load_frame(){
    vector<double> pose = this->getPose();

    int dataPtr = 0;

    stack<Segment *> segQ;
    for(auto iter = this->root.begin(); iter < this->root.end(); iter++){
        segQ.push((*iter).get());
    }

    while(segQ.size() > 0){
        Segment *curr = segQ.top();
        segQ.pop();

        int channels = curr->numChannels();
        for(int i = 0; i < channels; i++){
            curr->applyChannel(pose[dataPtr], i);
            dataPtr++;
        }

        int segs = curr->numSub();
        for(int i = segs - 1; i >= 0; i--){
            segQ.push(curr->getSeg(i));
        }
    }

    return Eigen::Vector3d(pose[0], pose[1], pose[2]);
}

void Controller::accelerate(){
    if (goal_speed < 4) goal_speed += 1;
    input_flag = true;
}

void Controller::brake(){
    if (goal_speed > 1) goal_speed -= 1;
    input_flag = true;
}

void Controller::mode_change(){
    to_move = !to_move;
    input_flag = true;
}

void Controller::turn_left(){
    if (goal_direction > -2) goal_direction -= 1;
    input_flag = true;
}

void Controller::turn_right(){
    if (goal_direction < 2) goal_direction += 1;
    input_flag = true;
}

void Controller::jump(int jumptype){
    jump_flag = jumptype;
    if (!is_moving && !to_move) input_flag = true;
}

Eigen::Vector3d Controller::rotation(){
    return this->root.front()->getRot();
}

Eigen::Vector3d Controller::translation(){
    return this->root.front()->getTrans();
}

void match_orientation (Motion& motion){
    auto start_ori = bvh_to_quaternion(Eigen::Vector3d(motion[0][3], motion[0][4], motion[0][5]));
    auto end_ori = bvh_to_quaternion(Eigen::Vector3d(motion[motion.size()-1][3], motion[motion.size()-1][4], motion[motion.size()-1][5]));

    double angle = get_y_rotation(end_ori, start_ori);

    for (int i = 0; i < 5; i++){
        int idx = motion.size() - 5 + i;
        auto frame = motion[idx];
        auto ori = bvh_to_quaternion(Eigen::Vector3d(frame[3], frame[4], frame[5]));
        Eigen::Quaterniond rot(Eigen::AngleAxisd(angle * (0.2 * (i + 1)), Eigen::Vector3d::UnitY()));
        ori = rot * ori;

        Eigen::Vector3d new_ori = quaternion_to_bvh(ori);
        motion[idx][3] = new_ori[0]; motion[idx][4] = new_ori[1]; motion[idx][5] = new_ori[2];
    }
}

void Controller::load_motion(){
    string folder = "./MotionData2/cmu/";

    // list filename here for each motion
    vector<string> jump = {"01", "02"};
    vector<string> highjump = {"03", "04"};
    vector<string> forwardjump = {"05", "06", "07", "09", "10"};

    // List motion for each gesture.
    vector<vector<string>> walk ;
    walk.emplace_back(initializer_list<string>{"walk, 90-degree left turn", "17", "18"});
    walk.emplace_back(initializer_list<string>{"walk, veer left", "11", "12"});
    walk.emplace_back(initializer_list<string>{"walk", "15", "16"});
    walk.emplace_back(initializer_list<string>{"walk, veer right", "13", "14"});
    walk.emplace_back(initializer_list<string>{"walk, 90-degree right turn", "19", "20"});
    walk.emplace_back(initializer_list<string>{"walk", "31", "32"});
    walk.emplace_back(initializer_list<string>{"slow walk, stop", "33", "34"});

    vector<vector<string>> fastwalk;
    fastwalk.emplace_back(initializer_list<string>{"walk, 90-degree left turn", "27", "28"});
    fastwalk.emplace_back(initializer_list<string>{"walk, veer left", "23", "24"});
    fastwalk.emplace_back(initializer_list<string>{"walk", "21", "22"});
    fastwalk.emplace_back(initializer_list<string>{"walk, veer right", "25", "26"});
    fastwalk.emplace_back(initializer_list<string>{"walk, 90-degree right turn", "29", "30"});
    fastwalk.emplace_back(initializer_list<string>{"walk", "31", "32"}); //
    fastwalk.emplace_back(initializer_list<string>{"slow walk, stop", "33", "34"}); //
    
    vector<vector<string>> jog;
    jog.emplace_back(initializer_list<string>{"run&jog, 90-degree left turn", "41", "42"});
    jog.emplace_back(initializer_list<string>{"run&jog, veer left", "37", "38"});
    jog.emplace_back(initializer_list<string>{"run&jog", "35", "36"});
    jog.emplace_back(initializer_list<string>{"run&jog, veer right", "39", "40"});
    jog.emplace_back(initializer_list<string>{"run&jog, 90-degree right turn", "43", "44"});
    jog.emplace_back(initializer_list<string>{"run&jog", "56"});
    jog.emplace_back(initializer_list<string>{"run&jog, sudden stop", "08", "57"});

    vector<vector<string>> run;
    run.emplace_back(initializer_list<string>{"run, 90-degree left turn", "51", "52"});
    run.emplace_back(initializer_list<string>{"run, veer left", "48"});
    run.emplace_back(initializer_list<string>{"run&jog", "45", "46"});
    run.emplace_back(initializer_list<string>{"run, veer right", "49", "50"});
    run.emplace_back(initializer_list<string>{"run, 90-degree right turn", "53", "54"});
    run.emplace_back(initializer_list<string>{"run", "55"});
    run.emplace_back(initializer_list<string>{"run&jog, sudden stop", "08", "57"});

    for(string data : jump){
        jump_data.emplace_back(vector<Motion>());
        string filename = "16_" + data + "_jump.bvh";
        jump_data[0].push_back(extractMotion(folder + filename));
    }

    for(string data : highjump){
        jump_data.emplace_back(vector<Motion>());
        string filename = "16_" + data + "_high jump.bvh";
        jump_data[1].push_back(extractMotion(folder + filename));
    }

    for(string data : forwardjump){
        jump_data.emplace_back(vector<Motion>());
        string filename = "16_" + data + "_forward jump.bvh";
        jump_data[2].push_back(extractMotion(folder + filename));
    }

    for(vector<string> pose : walk){
        walk_data.emplace_back(vector<Motion>());
        for(int i = 1; i < pose.size(); i++){
            string filename = "16_" + pose[i] + "_" + pose[0] + ".bvh";
            walk_data[walk_data.size() - 1].push_back(extractMotion(folder + filename));
        }
    }

    for(vector<string> pose : fastwalk){
        fastwalk_data.emplace_back(vector<Motion>());
        for(int i = 1; i < pose.size(); i++){
            string filename = "16_" + pose[i] + "_" + pose[0] + ".bvh";
            fastwalk_data[fastwalk_data.size() - 1].push_back(extractMotion(folder + filename));
        }
    }

    for(vector<string> pose : jog){
        jog_data.emplace_back(vector<Motion>());
        for(int i = 1; i < pose.size(); i++){
            string filename = "16_" + pose[i] + "_" + pose[0] + ".bvh";
            jog_data[jog_data.size() - 1].push_back(extractMotion(folder + filename));
        }
    }

    for(vector<string> pose : run){
        run_data.emplace_back(vector<Motion>());
        for(int i = 1; i < pose.size(); i++){
            string filename = "16_" + pose[i] + "_" + pose[0] + ".bvh";
            run_data[run_data.size() - 1].push_back(extractMotion(folder + filename));
        }
    }

    // Extract stop motion from the end of stopping walk motion.
    stop_data = walk_data[6];
    for (auto &motion : stop_data){
        motion = Motion(motion.end() - 5, motion.end());
        Motion motion_copy = Motion(motion);
        motion.insert(motion.end(), motion_copy.begin(), motion_copy.end());
        motion.insert(motion.end(), motion_copy.begin(), motion_copy.end());
        motion.insert(motion.end(), motion_copy.begin(), motion_copy.end());
        motion.insert(motion.end(), motion_copy.begin(), motion_copy.end());
        motion.insert(motion.end(), motion_copy.begin(), motion_copy.end());
        motion.insert(motion.end(), motion_copy.begin(), motion_copy.end());
        motion.insert(motion.end(), motion_copy.begin(), motion_copy.end());
    }

    /*
    
    process and interpolate straight motion so the beginning and the ending would have the same orientation here
    also make stop motion begin and end at the same position
    
    */
    for (auto& motion : walk_data[2]){
        match_orientation(motion);
    }

    for (auto& motion : fastwalk_data[2]){
        match_orientation(motion);
    }

    for (auto& motion : jog_data[2]){
        match_orientation(motion);
    }

    for (auto& motion : run_data[2]){
        match_orientation(motion);
    }

    for (auto& motion : stop_data){
        auto start_ori = bvh_to_quaternion(Eigen::Vector3d(motion[0][3], motion[0][4], motion[0][5]));
        auto end_ori = bvh_to_quaternion(Eigen::Vector3d(motion[motion.size()-1][3], motion[motion.size()-1][4], motion[motion.size()-1][5]));
        auto start_pos = Eigen::Vector3d(motion[0][0], motion[0][1], motion[0][2]);
        auto end_pos = Eigen::Vector3d(motion[motion.size()-1][0], motion[motion.size()-1][1], motion[motion.size()-1][2]);

        Eigen::Quaterniond rot = end_ori.inverse() * start_ori;
        Eigen::Quaterniond zero_rot(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));
        for (int i = 0; i < 10; i++){
            int idx = motion.size() - 10 + i;
            auto frame = motion[idx];
            auto ori = bvh_to_quaternion(Eigen::Vector3d(frame[3], frame[4], frame[5]));
            auto pos = Eigen::Vector3d(frame[0], frame[1], frame[2]);
            auto rot_inter = zero_rot.slerp((double)(i + 1) * 0.1, rot);
            auto trans = (start_pos - end_pos) * ((double)(i + 1) * 0.1);
            ori *= rot_inter;
            pos += trans;

            Eigen::Vector3d new_ori = quaternion_to_bvh(ori);
            motion[idx][0] = pos[0]; motion[idx][1] = pos[1]; motion[idx][2] = pos[2];
            motion[idx][3] = new_ori[0]; motion[idx][4] = new_ori[1]; motion[idx][5] = new_ori[2];
        }
    }

    is_motion_loaded = true;
}

Controller::Controller(){
    if(!is_motion_loaded) load_motion();

    string model_file = "./MotionData2/new/Trial001.bvh";
    BVHReader model_bvh(model_file);
    
    if(!model_bvh.loadFile()){
        cout << "Motion data file is not found." << endl;
        exit(0);
    }

    root = move(model_bvh.getRoots());

    predicted_motion = walk_data[2][0];
    input_flag = true;

    random_device rd;
    this->gen = mt19937(rd());
    this->dis = uniform_int_distribution<int>(INT_MIN, INT_MAX);

    curr_frame = 0;

    is_moving = true;
    to_move = true;
}

vector<double> Controller::getPose(){
    curr_frame += 1;
    if (curr_frame >= predicted_motion.size() - 10 || input_flag) {
        this->updateState();
        this->predictMotion();
    }

    return predicted_motion[curr_frame];
}

void Controller::updateState(){
    if (curr_frame >= predicted_motion.size() - 10){
        if (jump_flag && !is_moving) {
            jump_flag = 0;
            dont_disturb = false;
        }
        else if (is_moving && !to_move) {
            is_moving = false;
        }
        else if (!is_moving && to_move) {
            is_moving = true;
        }
        else {
            goal_direction = 0; 
        }
    }
}

void Controller::predictMotion(){
/*
    Choose motion that should be followed, and interpolate motion data.
*/
    if (dont_disturb && curr_frame > 15) {
        return;
    }

    vector<Motion>* next_motion = {};
    if (jump_flag && !is_moving) {
        next_motion = &(jump_data[jump_flag - 1]);
        dont_disturb = true;
    }
    else if (!is_moving && !to_move) next_motion = &stop_data;
    else {
        vector<vector<Motion>>* speed = nullptr;
        switch (goal_speed) {
            case 2:
                speed = &fastwalk_data; break;
            case 3:
                speed = &jog_data; break;
            case 4:
                speed = &run_data; break;
            default:
                speed = &walk_data; break;
        }

        if (is_moving && !to_move) {
            next_motion = &(speed->at(6));
        }
        else if (!is_moving && to_move) {
            next_motion = &(speed->at(5));
        }
        else next_motion = &(speed->at(goal_direction + 2));
    }

    Motion next_data = next_motion->at(this->dis(gen)%(next_motion->size()));

    // deleted part of already displayed motion.
    predicted_motion = Motion(predicted_motion.begin() + curr_frame, predicted_motion.end());
    curr_frame = 0;

    // if current motion is not finished, reinterpolate motion with the next motion
    predicted_motion = interpolate_motion(predicted_motion, next_data, true);

    input_flag = false;
}

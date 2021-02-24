#include "Controller.hpp"

Motion extractMotion(string filename){
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

void Controller::load_frame(){
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

void Controller::jump(){
    if (jump_flag < 3){
        jump_flag += 1;
        to_move = false;
    }
    input_flag = true;
}

Controller::Controller(){
    string model_file = "./MotionData2/new/Trial001.bvh";
    BVHReader model_bvh(model_file);
    
    if(!model_bvh.loadFile()){
        cout << "Motion data file is not found." << endl;
        exit(0);
    }

    root = move(model_bvh.getRoots());

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
    run.emplace_back(initializer_list<string>{"run&jog, sudden stop", "08", "57"}); //

    for(string data : jump){
        this->jump_data.emplace_back(vector<Motion>());
        string filename = "16_" + data + "_jump.bvh";
        this->jump_data[0].push_back(extractMotion(folder + filename));
    }

    for(string data : highjump){
        this->jump_data.emplace_back(vector<Motion>());
        string filename = "16_" + data + "_high jump.bvh";
        this->jump_data[1].push_back(extractMotion(folder + filename));
    }

    for(string data : forwardjump){
        this->jump_data.emplace_back(vector<Motion>());
        string filename = "16_" + data + "_forward jump.bvh";
        this->jump_data[2].push_back(extractMotion(folder + filename));
    }

    for(vector<string> pose : walk){
        this->walk.emplace_back(vector<Motion>());
        for(int i = 1; i < pose.size(); i++){
            string filename = "16_" + pose[i] + "_" + pose[0] + ".bvh";
            this->walk[this->walk.size() - 1].push_back(extractMotion(folder + filename));
        }
    }

    for(vector<string> pose : fastwalk){
        this->fastwalk.emplace_back(vector<Motion>());
        for(int i = 1; i < pose.size(); i++){
            string filename = "16_" + pose[i] + "_" + pose[0] + ".bvh";
            this->fastwalk[this->fastwalk.size() - 1].push_back(extractMotion(folder + filename));
        }
    }

    for(vector<string> pose : jog){
        this->jog.emplace_back(vector<Motion>());
        for(int i = 1; i < pose.size(); i++){
            string filename = "16_" + pose[i] + "_" + pose[0] + ".bvh";
            this->jog[this->jog.size() - 1].push_back(extractMotion(folder + filename));
        }
    }

    for(vector<string> pose : run){
        this->run.emplace_back(vector<Motion>());
        for(int i = 1; i < pose.size(); i++){
            string filename = "16_" + pose[i] + "_" + pose[0] + ".bvh";
            this->run[this->run.size() - 1].push_back(extractMotion(folder + filename));
        }
    }

    // Extract stop motion from the end of stopping walk motion.
    this->stop_data = this->walk[6];
    for (auto &motion : this->stop_data){
        motion = Motion(motion.end() - 5, motion.end());
    }

    predicted_motion = this->walk[2][0];
    input_flag = true;

    random_device rd;
    this->gen = mt19937(rd());
    this->dis = uniform_int_distribution<int>(INT_MIN, INT_MAX);

    curr_frame = 0;
    next_motion_frame = 0;
    interpolated_frame = 0;

    is_moving = true;
    to_move = true;
}

vector<double> Controller::getPose(){
    if (curr_frame < next_motion_frame && !input_flag) {
        curr_frame += 1;
    }
    else {
        this->updateState();
        this->predictMotion();
        curr_frame = 0;
    }

    return predicted_motion[curr_frame];
}

void Controller::updateState(){
    if (curr_frame >= next_motion_frame){
        if (jump_flag && !is_moving) jump_flag = 0;
        else if (is_moving && !to_move) is_moving = false;
        else if (!is_moving && to_move) is_moving = true;
        else {
            curr_direction = goal_direction;
            curr_speed = goal_speed;
        }
        predicted_motion = Motion(predicted_motion.begin() + next_motion_frame, predicted_motion.end());
        curr_frame = 0;
        next_motion_frame = predicted_motion.size();
        interpolated_frame = predicted_motion.size();
    }
}

void Controller::predictMotion(){
/*
    Choose motion that should be followed, and interpolate motion data.
*/
    vector<Motion>* next_motion = {};
    if (jump_flag && !is_moving) next_motion = &(jump_data[jump_flag - 1]);
    else if (!is_moving && !to_move) next_motion = &stop_data;
    else {
        vector<vector<Motion>>* speed = nullptr;
        switch (goal_speed) {
            case 2:
                speed = &fastwalk; break;
            case 3:
                speed = &jog; break;
            case 4:
                speed = &run; break;
            default:
                speed = &walk; break;
        }

        if (is_moving && !to_move) next_motion = &(speed->at(6));
        else if (!is_moving && to_move) next_motion = &(speed->at(5));
        else next_motion = &(speed->at(goal_direction + 2));
    }

    Motion next_data = next_motion->at(this->dis(gen)%(next_motion->size()));

    // deleted part of already displayed motion.
    predicted_motion = Motion(predicted_motion.begin() + curr_frame, predicted_motion.end());
    interpolated_frame -= curr_frame;
    next_motion_frame -= curr_frame;
    curr_frame = 0;

    int old_size = predicted_motion.size();

    // if current motion is not finished, reinterpolate motion with the next motion
    if (curr_frame < interpolated_frame) {
        predicted_motion = Motion(predicted_motion.begin(), predicted_motion.begin() + interpolated_frame);
        old_size = interpolated_frame;
        predicted_motion.insert(predicted_motion.end(), next_data.begin(), next_data.end());
    }
    else {
        // else just interpolate from current frame
        predicted_motion.insert(predicted_motion.end(), next_data.begin(), next_data.end());
    }
    next_motion_frame = old_size;
    interpolated_frame = old_size;

    input_flag = false;
}
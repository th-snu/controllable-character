#include "Controller.hpp"

Motion extractMotion(string filename){
    BVHReader reader(filename);
    if (reader.loadFile()){
        return reader.getMotion();
    }
    else return vector<vector<double>>();
}

void Controller::draw(){
    for(int i = 0; i < root.size(); i++){
        root[i]->draw();
    }
}

void Controller::accelerate(){
    if (!to_move) to_move = true;
    else if (goal_speed < 4) goal_speed += 1;
}

void Controller::brake(){
    if (goal_speed == 1 && to_move) to_move = false;
    else if (goal_speed > 1) goal_speed -= 1;
}

void Controller::stop(){
    if (to_move) to_move = false;
}

void Controller::turn_left(){
    if (goal_direction > -2) goal_direction -= 1;
}

void Controller::turn_right(){
    if (goal_direction < 2) goal_direction += 1;
}

void Controller::jump(){
    if (jump_flag < 3){
        jump_flag += 1;
        to_move = false;
    }
}

vector<double> Controller::getMotion(){
    if (!to_move && is_moving){
        // character should stop moving nhy

    }
    else if (curr_direction != goal_direction){
        
    }
    else if (curr_speed != goal_speed){

    }
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
    walk.emplace_back(initializer_list<string>{"walk, veer left", "11", "12"});
    walk.emplace_back(initializer_list<string>{"walk, veer right", "13", "14"});
    walk.emplace_back(initializer_list<string>{"walk", "15", "16"});
    walk.emplace_back(initializer_list<string>{"walk, 90-degree left turn", "17", "18"});
    walk.emplace_back(initializer_list<string>{"walk, 90-degree right turn", "19", "20"});
    walk.emplace_back(initializer_list<string>{"walk", "31", "32"});
    walk.emplace_back(initializer_list<string>{"slow walk, stop", "33", "34"});

    vector<vector<string>> fastwalk;
    fastwalk.emplace_back(initializer_list<string>{"walk, veer left", "23", "24"});
    fastwalk.emplace_back(initializer_list<string>{"walk, veer right", "25", "26"});
    fastwalk.emplace_back(initializer_list<string>{"walk", "21", "22"});
    fastwalk.emplace_back(initializer_list<string>{"walk, 90-degree left turn", "27", "28"});
    fastwalk.emplace_back(initializer_list<string>{"walk, 90-degree right turn", "29", "30"});
    fastwalk.emplace_back();
    fastwalk.emplace_back();
    
    vector<vector<string>> jog;
    jog.emplace_back(initializer_list<string>{"run&jog, veer left", "37", "38"});
    jog.emplace_back(initializer_list<string>{"run&jog, veer right", "39", "40"});
    jog.emplace_back(initializer_list<string>{"run&jog", "35", "36"});
    jog.emplace_back(initializer_list<string>{"run&jog, 90-degree left turn", "41", "42"});
    jog.emplace_back(initializer_list<string>{"run&jog, 90-degree right turn", "43", "44"});
    jog.emplace_back(initializer_list<string>{"run&jog", "56"});
    jog.emplace_back(initializer_list<string>{"run&jog, sudden stop", "08", "57"});

    vector<vector<string>> run;
    run.emplace_back(initializer_list<string>{"run, veer left", "48"});
    run.emplace_back(initializer_list<string>{"run, veer right", "49", "50"});
    run.emplace_back(initializer_list<string>{"run&jog", "45", "46"});
    run.emplace_back(initializer_list<string>{"run, 90-degree left turn", "51", "52"});
    run.emplace_back(initializer_list<string>{"run, 90-degree right turn", "53", "54"});
    run.emplace_back(initializer_list<string>{"run", "55"});
    run.emplace_back();

    for(string data : jump){
        string filename = "16_" + data + "_jump.bvh";
        this->jump_data.push_back(extractMotion(filename));
    }

    for(string data : highjump){
        string filename = "16_" + data + "_high jump.bvh";
        this->highjump_data.push_back(extractMotion(filename));
    }

    for(string data : forwardjump){
        string filename = "16_" + data + "_forward jump.bvh";
        this->forwardjump_data.push_back(extractMotion(filename));
    }

    for(vector<string> pose : walk){
        this->walk.emplace_back(vector<Motion>());
        for(int i = 1; i < pose.size(); i++){
            string filename = "16_" + pose[i] + "_" + pose[0] + ".bvh";
            this->walk[this->walk.size() - 1].push_back(extractMotion(filename));
        }
    }

    for(vector<string> pose : fastwalk){
        this->fastwalk.emplace_back(vector<Motion>());
        for(int i = 1; i < pose.size(); i++){
            string filename = "16_" + pose[i] + "_" + pose[0] + ".bvh";
            this->fastwalk[this->fastwalk.size() - 1].push_back(extractMotion(filename));
        }
    }

    for(vector<string> pose : jog){
        this->jog.emplace_back(vector<Motion>());
        for(int i = 1; i < pose.size(); i++){
            string filename = "16_" + pose[i] + "_" + pose[0] + ".bvh";
            this->jog[this->jog.size() - 1].push_back(extractMotion(filename));
        }
    }

    for(vector<string> pose : run){
        this->run.emplace_back(vector<Motion>());
        for(int i = 1; i < pose.size(); i++){
            string filename = "16_" + pose[i] + "_" + pose[0] + ".bvh";
            this->run[this->run.size() - 1].push_back(extractMotion(filename));
        }
    }
}
#include "Controller.hpp"
#include "bvh-loader/BVHReader.h"

Motion extractMotion(string filename){
    BVHReader reader(filename);
    if (reader.loadFile()){
        return reader.getMotion();
    }
    else return vector<vector<double>>();
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

    vector<string> beginWalk = {"walk", "31", "32"};
    vector<string> walkL45 = {"walk, veer left", "11", "12"};
    vector<string> walkR45 = {"walk, veer right", "13", "14"};
    vector<string> walk = {"walk", "15", "16"};
    vector<string> walkL90 = {"walk, 90-degree left turn", "17", "18"};
    vector<string> walkR90 = {"walk, 90-degree right turn", "19", "20"};
    vector<string> walkStop = {"slow walk, stop", "33", "34"};

    vector<string> fastwalkL45 = {"walk, veer left", "23", "24"};
    vector<string> fastwalkR45 = {"walk, veer right", "25", "26"};
    vector<string> fastwalk = {"walk", "21", "22"};
    vector<string> fastwalkL90 = {"walk, 90-degree left turn", "27", "28"};
    vector<string> fastwalkR90 = {"walk, 90-degree right turn", "29", "30"};
    
    vector<string> beginJog = {"run&jog", "56"};
    vector<string> jogStop = {"run&jog, sudden stop", "08", "57"};
    vector<string> jogL45 = {"run&jog, veer left", "37", "38"};
    vector<string> jogR45 = {"run&jog, veer right", "39", "40"};
    vector<string> jog = {"run&jog", "35", "36"};
    vector<string> jogL90 = {"run&jog, 90-degree left turn", "41", "42"};
    vector<string> jogR90 = {"run&jog, 90-degree right turn", "43", "44"};

    vector<string> beginRun = {"run", "55"};
    vector<string> runL45 = {"run, veer left", "48"};
    vector<string> runR45 = {"run, veer right", "49", "50"};
    vector<string> run = {"run&jog", "45", "46"}; 
    vector<string> runL90 = {"run, 90-degree left turn", "51", "52"};
    vector<string> runR90 = {"run, 90-degree right turn", "53", "54"};

    for(string data : jump){
        string filename = "16_" + data + "_jump.bvh";
        this->jump.push_back(extractMotion(filename));
    }

    for(string data : highjump){
        string filename = "16_" + data + "_high jump.bvh";
        this->highjump.push_back(extractMotion(filename));
    }

    for(string data : forwardjump){
        string filename = "16_" + data + "_forward jump.bvh";
        this->forwardjump.push_back(extractMotion(filename));
    }
}
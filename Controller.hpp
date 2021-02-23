#pragma once
#include "bvh-loader/GlHelper/DrawHelper.h"
#include "bvh-loader/BVHReader.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>
#include <queue>

using namespace std;

enum Pose {
    L45, L90, Move, R45, R90, Begin, Stop
};

class Controller {
/*
    Motion loader for character control based on cmu motion dataset
*/

public:
    Controller();
    void draw();
    void accelerate();
    void brake();
    void stop();
    void turn_left();
    void turn_right();
    void jump();

private:
/*
    From left 90 turn to right 90 turn and begin/stop motion.
    Each pointer points array of corresponding motion datas.
*/
    vector<vector<Motion>> walk;
    vector<vector<Motion>> fastwalk;
    vector<vector<Motion>> jog;
    vector<vector<Motion>> run;

/*
    Jump will make the character to stop first, then jump, and then resume.
    Pressing key multiple times will change jump mode. It applies to turning motion as well.
*/
    vector<Motion> jump_data;
    vector<Motion> highjump_data;
    vector<Motion> forwardjump_data;

    int curr_speed;
    int curr_direction;

    int goal_speed;
    int goal_direction;

    int jump_flag;
    
    bool to_move;
    bool is_moving;

    vector<unique_ptr<Segment>> root;

    double frame_pos;
    queue<vector<double>> predicted_motion;

/*
    When character stops, it repeats last few frames of the last motion with interpolation.
    If motion for current movement does not have stopping motion, character will slow down and then stop.

    When user decides to jump, character automatically stops, and Jump motion must be completed before moving onto next motion.
    Start/Stop motion should be completed at least partly.
*/
    vector<double> getMotion();
};

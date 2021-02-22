#pragma once
#include "bvh-loader/GlHelper/DrawHelper.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>
#include <queue>

using namespace std;

class Motion {
    vector<vector<double>> motion;
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
    From left 90 turn to right 90 turn and stop motion.
    each pointer points array of corresponding motion datas

    When character stops, it repeats last few frames of the last motion with interpolation
*/
    vector<Motion> walk[6];
    vector<Motion> jog[6];
    vector<Motion> run[6];

/*
    Jump will make the character to stop first, then jump, and then resume.
    Pressing key multiple times will change jump mode. It applies to turning motion as well.
*/
    vector<Motion> jump;
    vector<Motion> highjump;
    vector<Motion> forwardjump;

    Motion* motion;
    Motion* next_motion;
    int goal_speed;
    int goal_direction;

    vector<unique_ptr<Segment>> root;

    double frame_pos;
    bool input_flag;
    queue<vector<double>> predicted_motion;

    vector<double> getMotion();
};

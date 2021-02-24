#pragma once
#include "bvh-loader/GlHelper/DrawHelper.h"
#include "bvh-loader/BVHReader.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>
#include <queue>
#include <random>
#include <bits/stdc++.h>

using namespace std;

class Controller {
/*
    Motion loader for character control based on cmu motion dataset
*/

public:
    Controller();
    void draw();
    void load_frame();

    void accelerate();
    void brake();
    void mode_change();
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

    vector<Motion> stop_data;

    std::mt19937 gen;
    uniform_int_distribution<int> dis;

/*
    Jump will make the character to stop first, then jump, and then resume.
    Pressing key multiple times will change jump mode. It applies to turning motion as well.
*/
    vector<vector<Motion>> jump_data;

    int curr_speed = 1;
    int curr_direction = 0;

    int goal_speed = 1;
    int goal_direction = 0;

    int jump_flag;
    
    bool to_move;
    bool is_moving;

    vector<unique_ptr<Segment>> root;

    int curr_frame;
    int interpolated_frame;
    int next_motion_frame;
    Motion predicted_motion;

    bool input_flag = false;

/*
    When character stops, it repeats last few frames of the last motion with interpolation.
    If motion for current movement does not have stopping motion, character will slow down and then stop.

    When user decides to jump, character automatically stops, and Jump motion must be completed before moving onto next motion.
    Start/Stop motion should be completed at least partly.
*/
    vector<double> getPose();
    void predictMotion();
    void updateState();
};

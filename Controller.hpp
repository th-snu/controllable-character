#pragma once
#include "bvh-loader/GlHelper/DrawHelper.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>
#include <queue>
#include <random>
#include <bits/stdc++.h>
#include "Interpolation.hpp"

using namespace std;

class Controller {
/*
    Motion loader for character control based on cmu motion dataset
*/

public:
    Controller();
    void draw();
    Eigen::Vector3d load_frame();

    void accelerate();
    void brake();
    void mode_change();
    void turn_left();
    void turn_right();
    void jump(int jumptype);

    Eigen::Vector3d rotation();
    Eigen::Vector3d translation();

private:
/*
    From left 90 turn to right 90 turn and begin/stop motion.
    Each pointer points array of corresponding motion datas.
*/
    static void load_motion();
    static bool is_motion_loaded;
    static vector<vector<Motion>> walk_data;
    static vector<vector<Motion>> fastwalk_data;
    static vector<vector<Motion>> jog_data;
    static vector<vector<Motion>> run_data;

    static vector<Motion> stop_data;

/*
    Jump will make the character to stop first, then jump, and then resume.
    Pressing key multiple times will change jump mode. It applies to turning motion as well.
*/
    static vector<vector<Motion>> jump_data;

    std::mt19937 gen;
    uniform_int_distribution<int> dis;

    int goal_speed = 1;
    int goal_direction = 0;

    int jump_flag = 0;
    
    bool to_move;
    bool is_moving;

    vector<unique_ptr<Segment>> root;

    int curr_frame;
    int interpolated_frame;
    int next_motion_frame;
    Motion predicted_motion;

    bool input_flag = false;
    bool dont_disturb = false;

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

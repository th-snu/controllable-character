#pragma once
#include "bvh-loader/BVHReader.h"
#include "bvh-loader/GlHelper/DrawHelper.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>

using namespace std;

class MotionLoader {
public:
    void draw();
    BVHReader jump();
    
};

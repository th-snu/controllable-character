#include <random>
#include <map>
#include "bvh-loader/BVHReader.h"
#include "Camera.hpp"
#include "bvh-loader/GlHelper/DrawHelper.h"
#include "Controller.hpp"

using namespace std;

class UserInterface {
    public:
        static vector<string> splitString(string str);
        static UserInterface *current();
        static void setCurrent(UserInterface *window);
        void initialize();

        void loadGlobalCoord();
        void drawGridPlane();

        static void DisplayEvent();
        static void KeyboardEvent(unsigned char key,int x,int y);
        static void SpecialEvent(int key, int x, int y);
        static void MouseEvent(int button, int state, int x, int y);
        static void MotionEvent(int x, int y);
        static void ReshapeEvent(int w, int h);
        static void TimerEvent(int value);
        static UserInterface *curr;

    private:
        std::unique_ptr<Controller> motionLoader = nullptr;

        GLfloat mousePosX, mousePosY;
        Camera *currView;

        void nextFrame();
        void reload();
        void helpText();

        unsigned timeStep = 30;

        int width, height;

        bool leftButton = false;

        const Eigen::Vector3f initPos = Eigen::Vector3f(-300.0f, 200.0f, -300.0f);
        Camera mainCamera = Camera(initPos, Eigen::Vector3f(1.0f, 0.0f, 1.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f));

        void motion(int x, int y);
        void mouse(int button, int state, int x, int y);
        void display();
        void reshape(int w, int h);
        void timer(int unused);
        void keyboard(unsigned char key, int x, int y);
        void special(int key, int x, int y);
};

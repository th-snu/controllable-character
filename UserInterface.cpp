#include <string>
#include <iostream>
#include "UserInterface.hpp"

using namespace std;

void glutBitmapString(void* font, const char* string){
	glutBitmapString(font, reinterpret_cast<const unsigned char *>(string));
}

UserInterface *UserInterface::curr = nullptr;

void UserInterface::initialize(){
	currView = &mainCamera;
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(50, 0);
	glutCreateWindow("Controllable Character");
	glutReshapeFunc(UserInterface::ReshapeEvent);
	glutDisplayFunc(UserInterface::DisplayEvent);
	glutTimerFunc(timeStep, UserInterface::TimerEvent, 0);
	glutKeyboardFunc(UserInterface::KeyboardEvent);
	glutSpecialFunc(UserInterface::SpecialEvent);
	glutMouseFunc(UserInterface::MouseEvent);
	glutMotionFunc(UserInterface::MotionEvent);
}

void UserInterface::loadGlobalCoord()
{
	glLoadIdentity();
	currView->lookAt();
}

UserInterface *UserInterface::current(){
	return UserInterface::curr;
}

void UserInterface::setCurrent(UserInterface *window){
	UserInterface::curr = window;
}

void UserInterface::DisplayEvent(){ current()->display(); }
void UserInterface::KeyboardEvent(unsigned char key,int x,int y ){ current()->keyboard(key,x,y); }
void UserInterface::MouseEvent(int button, int state, int x, int y) { current()->mouse(button,state,x,y); }
void UserInterface::SpecialEvent(int key, int x, int y) { current()->special(key, x, y); }
void UserInterface::MotionEvent(int x, int y) { current()->motion(x,y); }
void UserInterface::ReshapeEvent(int w, int h) { current()->reshape(w,h); }
void UserInterface::TimerEvent(int value) { current()->timer(value); }

void UserInterface::motion(int x, int y)
{
	if (leftButton) {
		float dx = x - mousePosX;
		float dy = y - mousePosY;

		mousePosX = x;
		mousePosY = y;

        currView->rotate(dx / width, dy / height);
		loadGlobalCoord();
	}
	return;
}

void UserInterface::nextFrame(){
	// Get motion of the next frame
}

void UserInterface::keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 'w':
		currView->move(Camera::Forward);
		break;
	case 's':
		currView->move(Camera::Backward);
		break;
	case 'a':
		currView->move(Camera::Left);
		break;
	case 'd':
		currView->move(Camera::Right);
		break;
	case ' ':
		motionLoader->jump();
	case 27:
		exit(0);
		break;
	default:
		break;
	}
}

void UserInterface::special(int key, int x, int y) {
	switch(key){
		case GLUT_KEY_UP:
		motionLoader->accelerate();
		break;
		case GLUT_KEY_DOWN:
		motionLoader->brake();
		break;
		case GLUT_KEY_LEFT:
		motionLoader->turn_left();
		break;
		case GLUT_KEY_RIGHT:
		motionLoader->turn_right();
		break;
	}
}

void UserInterface::mouse(int button, int state, int x, int y) {
	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN)
		{
			mousePosX = x;
			mousePosY = y;
			leftButton = true;
		}
		else if (state == GLUT_UP)
		{
			leftButton = false;
		}
		break;
	case 3:break;
	default:break;
	}
	return;
}

void UserInterface::drawGridPlane() {
	glBegin(GL_LINES);
	glColor3f(0, 1, 0);
	for (int i = -1000; i < 1001; i+=100){
		glVertex3f(i, 0, -1000);
		glVertex3f(i, 0, 1000);
		glVertex3f(1000, 0, i);
		glVertex3f(-1000, 0, i);
	}
	glEnd();
}

void UserInterface::helpText() {
	glRasterPos2i(100, 120);
	glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, "W A S D: Camera Move\nArrow Keys: Accelerate/Turn Character\nQ: Exit The Program");
}

void UserInterface::display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	loadGlobalCoord();

	glPushMatrix();
	drawGridPlane();

	motionLoader->draw();
	
	glPopMatrix();

	glutSwapBuffers();
}

void UserInterface::reshape(int w, int h) {
	width = w;
	height = h;
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, (GLfloat)w / (GLfloat)h, .1f, 2500.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void UserInterface::timer(int unused)
{
	/* call the display callback and forces the current window to be displayed */
	glutPostRedisplay();
	glutTimerFunc(timeStep, UserInterface::TimerEvent, 0);
}
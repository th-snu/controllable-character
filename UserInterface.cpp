#include <string>
#include <iostream>
#include "UserInterface.hpp"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>

using namespace std;

UserInterface *UserInterface::curr = nullptr;

void UserInterface::initialize(){
	endSites[currEndSite].back()->setColor(Eigen::Vector3d(0.0, 1.0, 1.0));
	currView = &mainCamera;
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(50, 0);
	glutCreateWindow("Controllable Character");
	glutReshapeFunc(UserInterface::ReshapeEvent);
	glutDisplayFunc(UserInterface::DisplayEvent);
	glutTimerFunc(timeStep, UserInterface::TimerEvent, 0);
	glutKeyboardFunc(UserInterface::KeyboardEvent);
	glutMouseFunc(UserInterface::MouseEvent);
	glutMotionFunc(UserInterface::MotionEvent);
}

vector<string> UserInterface::splitString(string str){
    vector<string> result; 
    istringstream iss(str); 
    for(string s; iss >> s; ) 
        result.push_back(s);
    return result;
}

void UserInterface::getJoints(){
	/*
		Get the list of endsites with the list of corresponding joints connected to it.
		BVHReader object will have control of Segment objects, ignore memory usage
	*/
	Segment * root = bvh->getRoot();
	flexibility = map<string, double>();
	endSites = vector<vector<Segment *>>();

    vector<pair<Segment *, int>> segQ;
	segQ.push_back(pair<Segment *, int>(root, 0));

    while(segQ.size() > 0){
        auto pr = segQ.back();
		Segment *curr = pr.first;
		int segCount = pr.second;
        segQ.pop_back();

		/* End Site case */
		if (curr->numSub() == 0){
			vector<Segment *> endPath;
			for (auto joint : segQ){
				endPath.push_back(joint.first);
			}
			endPath.push_back(curr);
			endSites.push_back(endPath);

			flexibility.insert(make_pair(curr->getName(), 1.0));

			continue;
		}

        int segs = curr->numSub();
        if (segCount < segs){
            segQ.push_back(pair<Segment *, int>(curr, segCount + 1));
			segQ.push_back(pair<Segment *, int>(curr->getSeg(segCount), 0));
        }
		else{
			flexibility.insert(make_pair(curr->getName(), 1.0));
		}
    }

	return;
}

void UserInterface::jointOptions(string line){
	int spacepos = line.find(' ');
	string cmd = line.substr(0, spacepos);
	string args = line.substr(spacepos + 1, line.length() - spacepos - 1);

	if (cmd == "list"){
		if (args == "curr"){
			for (int i = 0; i < endSites[currEndSite].size(); i++){
				string name = endSites[currEndSite][i]->getName();
				cout << name << ", " << flexibility[name] << endl;
			}
		}
		else{
			for (pair<string, double> flex : flexibility){
				cout << flex.first << ", " << flex.second << endl;
			}
		}
	}
	else if (cmd == "flex"){
		int spacepos = args.find(' ');
		string jointName = args.substr(0, spacepos);
		if (flexibility.count(jointName)){
			try {
				flexibility[jointName] = stod(args.substr(spacepos));
			}
			catch (exception &e){
				cout << "Could not parse flexibility value." << endl;
			}
		}
		else {
			cout << "Joint " << jointName << " not found." << endl;
		}
	}
	else {
		cout << "joint list: " << "List selected joints." << endl;
		cout << "joint flex (j) (v): " << "Set flexibility of joint (j) to (v)." << endl;
	}

	return;
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
	case 'q':
        prevSite();
		break;
	case 'e':
		nextSite();
		break;
	case '1':
		nextFrame();
		break;
	case 'r':
		reload();
		break;
	case 27:
		exit(0);
		break;
	default:
		break;
	}
}

void UserInterface::reload(){
    vector<Segment *> segQ;
	segQ.push_back(this->bvh->getRoot());

    while(segQ.size() > 0){
        auto pr = segQ.back();
		Segment *curr = pr;
        segQ.pop_back();

		curr->rotate(Eigen::Vector3d(0.0, 0.0, 0.0));
		curr->translate(Eigen::Vector3d(0.0, 0.0, 0.0));
		
        for (int i = 0; i < pr->numSub(); i++){
            segQ.push_back(pr->getSeg(i));
        }
    }

	return;
}

void UserInterface::prevSite(){
	endSites[currEndSite].back()->setColor(Eigen::Vector3d(1.0, 0.0, 0.0));
	if (currEndSite > 0){
		currEndSite -= 1;
	}
	else currEndSite = endSites.size() - 1;
	endSites[currEndSite].back()->setColor(Eigen::Vector3d(0.0, 1.0, 1.0));
}

void UserInterface::nextSite(){
	endSites[currEndSite].back()->setColor(Eigen::Vector3d(1.0, 0.0, 0.0));
	if (currEndSite >= endSites.size() - 1){
		currEndSite = 0;
	}
	else currEndSite += 1;
	endSites[currEndSite].back()->setColor(Eigen::Vector3d(0.0, 1.0, 1.0));
}

void UserInterface::mouse(int button, int state, int x, int y)
{
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
	case GLUT_RIGHT_BUTTON:
		if (state == GLUT_DOWN){
			moveObject = !moveObject;
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
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, "W A S D : Accelerate/Turn");
}

void UserInterface::display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	loadGlobalCoord();

	//glRotatef(45, -1, 0, 0);
	glPushMatrix();
	drawGridPlane();

	bvh->draw();
	
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
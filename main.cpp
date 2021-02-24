#include "UserInterface.hpp"

int main(int argc, char** argv) {
	UserInterface *sim = new UserInterface();
	UserInterface::setCurrent(sim);

	glutInit(&argc, argv);
	sim->initialize();
	glutMainLoop();

	free(sim);

	return 0;
}
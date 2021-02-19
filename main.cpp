#include "UserInterface.hpp"

int main(int argc, char** argv) {
	UserInterface *sim = new UserInterface();

	glutInit(&argc, argv);
	sim->initialize();

	glutMainLoop();

	free(sim);

	return 0;
}
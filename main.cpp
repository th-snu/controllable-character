#include "UserInterface.hpp"

int main(int argc, char** argv) {
	/* Can be enhanced to have UI to choose file */
	string filename = "";
	UserInterface *sim = new UserInterface();

	glutInit(&argc, argv);
	sim->initialize();

	glutMainLoop();

	free(sim);

	return 0;
}
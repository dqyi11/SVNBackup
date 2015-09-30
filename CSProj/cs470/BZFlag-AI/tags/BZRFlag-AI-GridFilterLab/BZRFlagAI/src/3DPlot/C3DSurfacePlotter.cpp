/*
 * C3DSurfacePlotter.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: walter
 */

#include "C3DSurfacePlotter.h"

#include <iostream>
#include <string.h>
using namespace std;

static C3DSurfacePlotter * pPlotInstance = NULL;

void drawGridCallback()
{
	if(pPlotInstance)
	{
		pPlotInstance->Draw();
	}
}

C3DSurfacePlotter::C3DSurfacePlotter(int width, int height)
{
	// TODO Auto-generated constructor stub
	mSurfWidth = width;
	mSurfHeight = height;
	mpSurfaceData = 0;

}

C3DSurfacePlotter::~C3DSurfacePlotter() {
	// TODO Auto-generated destructor stub
}
void C3DSurfacePlotter::Initialize() {
	glClearColor(0, 0, 0, 0);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);
}

using namespace std;

bool C3DSurfacePlotter::init(double ** pSurfaceData)
{
	if(!pSurfaceData)
	{
		return false;
	}

	mpSurfaceData = pSurfaceData;

	// GLUT Window Initialization:
	int myargc = 0;
	char * myargv[1] = {NULL};


	glutInit(&myargc, myargv);
	glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);
	glutInitWindowSize (600, 600);
	glutInitWindowPosition(0,0);
	glutCreateWindow ("Grid Filter Visualization");
/*
	glutInit(&myargc, myargv);
	glutInitWindowSize (mSurfWidth, mSurfHeight);
	glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE | GLUT_DEPTH);
	glutInitWindowPosition(mSurfWidth,mSurfHeight);
	glutCreateWindow ("Grid Filter Visualization");

	::pPlotInstance = this;
	*/
	glutDisplayFunc(::drawGridCallback);

	Initialize();
	//glutDisplayFunc(Draw);


	//glutMainLoop();

	return true;
}

bool C3DSurfacePlotter::deinit()
{
	mpSurfaceData = 0;

	return true;
}

void C3DSurfacePlotter::Draw()
{

	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(.5, 1.0, 1.0);
	glBegin(GL_POINTS);
		for(int i = 0; i < mSurfWidth; i++){
			for(int j = 0; j < mSurfHeight; j++){
				float x = (float)i/800;
				float y = (float)j/800;

				float value = (float)mpSurfaceData[i][j];
				glColor3f(1-value, 1.0-value, 1.0-value);
				glVertex3f(x, y, 0);
			}
		}
	glEnd();
	glFlush();
	glutSwapBuffers();
/*
	glRasterPos2f(-1, -1);
	glDrawPixels(mSurfWidth, mSurfHeight, GL_LUMINANCE, GL_FLOAT, mpSurfaceData[0]);
	glFlush();
	glutSwapBuffers();
*/

}

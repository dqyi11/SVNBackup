/*
 * C3DSurfacePlotter.h
 *
 *  Created on: Oct 30, 2012
 *      Author: walter
 */

#ifndef C3DSURFACEPLOTTER_H_
#define C3DSURFACEPLOTTER_H_

#include <GL/gl.h>
#include <GL/glx.h>
#include <GL/glu.h>
#include <GL/glut.h>

class C3DSurfacePlotter {
public:
	C3DSurfacePlotter(int width, int height);
	virtual ~C3DSurfacePlotter();

	bool init(double ** pSurfaceData);
	bool deinit();
	void Initialize();

	void Draw();

private:
	int mSurfWidth;
	int mSurfHeight;

	double ** mpSurfaceData;
};

#endif /* C3DSURFACEPLOTTER_H_ */

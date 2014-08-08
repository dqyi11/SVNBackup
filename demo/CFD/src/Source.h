/*
 * Source.h
 *
 *  Created on: Jun 25, 2014
 *      Author: dhruvmsaxena
 */

#ifndef SOURCE_H_
#define SOURCE_H_

#include <math.h>
#include <iostream>
using namespace std;

#define IX(i,j) ((i)+(N+2)*(j))

class Source {
	int type; // 0 - unidirectional, 1 - sweep
	int o_x, o_y; // source origin coords
	float param1, param2; // (width, angle) for 0; (start angle, end angle) for 1; angles in radians
	float strength;
	float slope; // for 0 only

public:
	Source();
	Source(float []);

	void setSource(float *, float *, int, int);

	virtual ~Source();
};

#endif /* SOURCE_H_ */

/*
 * KalmanFilterConst.h
 *
 *  Created on: Dec 2, 2012
 *      Author: walter
 */

#ifndef KALMANFILTERCONST_H_
#define KALMANFILTERCONST_H_

#include <math.h>

const double initialState[] = {0.0,
							   0.0,
							   0.0,
							   0.0,
							   0.0,
							   0.0};

const double initialErrEstCov[] = {40.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					                  0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
					                  0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
					                  0.0, 0.0, 0.0, 40.0, 0.0, 0.0,
					                  0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
						              0.0, 0.0, 0.0, 0.0, 0.0, 0.1};

const double TransNoiseCov[] = {0.1, 0, 0, 0, 0, 0,
                                     0, 0.1, 0, 0, 0, 0,
                                     0, 0, 100, 0, 0, 0,
                                     0, 0, 0, 0.1, 0, 0,
                                     0, 0, 0, 0, 0.1, 0,
                                     0, 0, 0, 0, 0, 100};

const double ObsNoiseCov[] = {25, 0,
		                      0, 25};

const double ObsModel [] = {1,0,0,0,0,0,
							0,0,0,1,0,0};

/*
 *H = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 1 & 0 & 0 \end{bmatrix}
 * */


const double TransitionModel[] = {1, CTRL_STEP_TIME, pow(CTRL_STEP_TIME,2)/2, 0, 0, 0,
		                        0, 1, CTRL_STEP_TIME, 0, 0, 0,
		                        0, - FRICTION, 1, 0, 0, 0,
		                        0, 0, 0, 1, CTRL_STEP_TIME, pow(CTRL_STEP_TIME,2)/2,
		                        0, 0, 0, 0, 1, CTRL_STEP_TIME,
		                        0, 0, 0, 0, -FRICTION, 1};



#endif /* KALMANFILTERCONST_H_ */

/*
 * ControlParams.h
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#ifndef CONTROLPARAMS_H_
#define CONTROLPARAMS_H_

const double ATTRACTIVE_ALPHA = 150.0;
const double REPULSIVE_BETA = 80;
const double TANGENTIAL_GAMMA = 90;

const double BASE_ATR_SPREADING_RANGE = 10.0;
const double OBSTACLE_REP_SPREADING_RANGE = 10.0;
const double OBSTACLE_TAN_SPREADING_RANGE = 10.0;

const double PD_PROPORTIONAL = 1.5;
const double PD_DERIVATIVE = 0.2;
const double CONSTANT_LINEAR_SPEED = 10.0;

const double PI = 3.1415926;

const int PLOT_STEP = 10;

const double LIMIT_MAX = 100000;

const double CTRL_STEP_TIME = 0.02;  // unit sec

#endif /* CONTROLPARAMS_H_ */

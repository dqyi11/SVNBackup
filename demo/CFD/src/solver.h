/*
 * solver.h
 *
 *  Created on: Jun 24, 2014
 *      Author: dhruvmsaxena
 */

#ifndef SOLVER_H_
#define SOLVER_H_

void add_source ( int N, float * x, float * s, float dt );
void set_bnd ( int N, int b, float * x, int o, int * object );
void lin_solve ( int N, int b, float * x, float * x0, float a, float c, int o, int * object );
void diffuse ( int N, int b, float * x, float * x0, float diff, float dt, int o, int * object );
void advect ( int N, int b, float * d, float * d0, float * u, float * v, float dt, int o, int * object );
void project ( int N, float * u, float * v, float * p, float * div, int o, int * object );
void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt, int o, int * object );



#endif /* SOLVER_H_ */

#ifndef __GLOBAL_H_
#define __GLOBAL_H_

#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <memory.h>
#include <vector>
#include <windows.h>

using namespace std;

char    strTestInstance[256];
char    strFunctionType[256];


// bounds of variables
double  lowBound = 0,  uppBound = 1;  

// demensionality of variables and objectives
int     numVariables;
int     numObjectives;

// distribution indexes in SBX and polynomial mutation
int     id_cx = 20;    // crossover
int     id_mu = 20;    // for mutation

// ideal point used in decomposition methods
double  *idealpoint;

// parameters for random number generation
int     seed = 237;
long    rnd_uni_init;


#include "random.h"

#endif

#ifndef PROBLEM_FACTORY__
#define PROBLEM_FACTORY__

#include "problem_base.h"
#include <fstream>

BProblem *GenerateProblem(std::ifstream &ifile);


#endif
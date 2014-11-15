
#ifndef EXPERIMENT__
#define EXPERIMENT__

#include <fstream>

class CNSGAIII;
class BProblem;

void SetupExperiment(CNSGAIII &algo, BProblem **prob, std::ifstream &ifile);

#endif
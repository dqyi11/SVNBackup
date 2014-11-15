
#include "exp_experiment.h"
#include "problem_factory.h"
#include "alg_nsgaiii.h"

void SetupExperiment(CNSGAIII &algo, BProblem **prob, std::ifstream &ifile)
{
	algo.Setup(ifile);
	*prob = GenerateProblem(ifile);
}
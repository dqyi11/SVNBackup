#include "problem_ZDT.h"
#include "alg_individual.h"
#include <cmath>

using namespace std;


CProblemZDT1::CProblemZDT1(std::size_t m):
	BProblem("ZDT1"), 
	num_variables_(m), 
	num_objectives_(2) 
{
	lbs_.resize(num_variables_, 0.0); // lower bound 0.0
	ubs_.resize(num_variables_, 1.0); // upper bound 1.0
}
// ----------------------------------------------------------------------
bool CProblemZDT1::Evaluate(CIndividual *indv) const
{
	CIndividual::TDecVec &x = indv->vars();
	CIndividual::TObjVec &f = indv->objs();

	if (x.size() != num_variables_) return false; // #variables does not match

	f.resize(num_objectives_, 0);

	f[0] = x[0];

	double g = 0;
	for (size_t i=1; i<x.size(); i+=1)
	{
		g += x[i]/(num_variables_ - 1);
	}
	g = 1 + 9*g;

	double h = 1 - sqrt(f[0]/g);

	f[1] = g*h;

	return true;

}// bool CProblemDTLZ1::Evaluate(CIndividual *indv) const
#include "problem_self.h"
#include "alg_individual.h"

using namespace std;

CProblemSelf::CProblemSelf(std::size_t num_vars, std::size_t num_objs):
	BProblem("Self"),
	num_vars_(num_vars),
	num_objs_(num_objs)
{
	// define the domain of variables here
	lbs_.resize(num_vars_, 0.0);
	ubs_.resize(num_vars_, 1.0);
}
// -----------------------------------------------------------
bool CProblemSelf::Evaluate(CIndividual *indv) const
{
	CIndividual::TDecVec &x = indv->vars();
	CIndividual::TObjVec &f = indv->objs();

	if (x.size() != num_vars_) return false;

	// You can define your own problem here.
	f[0] = x[0];
	f[1] = 1-x[0];
	f[2] = x[1]+x[2];

	return true;
}

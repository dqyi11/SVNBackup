#ifndef PROBLEM_SELF__
#define PROBLEM_SELF__

#include "problem_base.h"

// You can add your own problem here.

class CProblemSelf : public BProblem
{
public:
	CProblemSelf(std::size_t num_vars, std::size_t num_objs);

	virtual std::size_t num_variables() const { return num_vars_; }
	virtual std::size_t num_objectives() const { return num_objs_; }
	virtual bool Evaluate(CIndividual *indv) const;

private:

	std::size_t num_vars_, num_objs_;
};

#endif
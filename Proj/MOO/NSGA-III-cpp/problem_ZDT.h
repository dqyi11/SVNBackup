#ifndef PROBLEM_ZDT__
#define PROBLEM_ZDT__

#include "problem_base.h"
#include <cstddef>

// ----------------------------------------------------------------------
//		CProblemZDT1
// ----------------------------------------------------------------------
class CProblemZDT1 : public BProblem
{
public:
	explicit CProblemZDT1(std::size_t m = 30); // default setting in ZDT

	virtual std::size_t num_variables() const { return num_variables_; }
	virtual std::size_t num_objectives() const { return num_objectives_; }

	virtual bool Evaluate(CIndividual *indv) const;

	CProblemZDT1 & operator= (const CProblemZDT1 &); // prohibit copy assignment (VS2012 does not support 'delete')

private:
	size_t num_variables_;
	const size_t num_objectives_;
};

#endif
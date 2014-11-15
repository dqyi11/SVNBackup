#ifndef INITIALIZATION__
#define INITIALIZATION__


// ----------------------------------------------------------------------
//		CRandomInitialization
// ----------------------------------------------------------------------

class CIndividual;
class CPopulation;
class BProblem;

class CRandomInitialization
{
public:
	void operator()(CPopulation *pop, const BProblem &prob) const;
	void operator()(CIndividual *indv, const BProblem &prob) const;
};

extern CRandomInitialization RandomInitialization;

#endif
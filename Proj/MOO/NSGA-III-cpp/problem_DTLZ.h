
#ifndef PROBLEM_DTLZ__
#define PROBLEM_DTLZ__

#include "problem_base.h"
#include <cstddef>

// ----------------------------------------------------------------------
//		CProblemDTLZ
//
// Deb, Thiele, Laumanns, and Zitzler, "Scalable Test Problems for 
// Evolutionary Multi-Objective Optimization," Evolutionary 
// Multiobjective Optimizatoin: Theoretical Advances and Applications,
// Chapter 6.
//
// http://link.springer.com/content/pdf/10.1007%2F1-84628-137-7.pdf
// ----------------------------------------------------------------------
class CProblemDTLZ : public BProblem
{
public:
	CProblemDTLZ(std::size_t M, std::size_t k, const std::string &name);

	virtual std::size_t num_variables() const { return M_+k_-1; }
	virtual std::size_t num_objectives() const { return M_; }

	virtual bool Evaluate(CIndividual *indv) const = 0;

protected:
	std::size_t M_; // number of objectives
	std::size_t k_; // number of variables in g(xM)
};


// ----------------------------------------------------------------------
//		CProblemDTLZ1
// ----------------------------------------------------------------------

class CProblemDTLZ1 : public CProblemDTLZ
{
public:
	explicit CProblemDTLZ1(std::size_t M, std::size_t k = 5):CProblemDTLZ(M, k, "DTLZ1") {}
	virtual bool Evaluate(CIndividual *indv) const;
};


// ----------------------------------------------------------------------
//		CProblemDTLZ2
// ----------------------------------------------------------------------

class CProblemDTLZ2 : public CProblemDTLZ
{
public:
	explicit CProblemDTLZ2(std::size_t M, std::size_t k = 10):CProblemDTLZ(M, k, "DTLZ2") {}
	virtual bool Evaluate(CIndividual *indv) const;
};


// ----------------------------------------------------------------------
//		CProblemDTLZ3
// ----------------------------------------------------------------------

class CProblemDTLZ3 : public CProblemDTLZ
{
public:
	explicit CProblemDTLZ3(std::size_t M, std::size_t k = 10):CProblemDTLZ(M, k, "DTLZ3") {}
	virtual bool Evaluate(CIndividual *indv) const;
};


// ----------------------------------------------------------------------
//		CProblemDTLZ4
// ----------------------------------------------------------------------

class CProblemDTLZ4 : public CProblemDTLZ
{
public:
	explicit CProblemDTLZ4(std::size_t M, std::size_t k = 10, double alpha = 100):CProblemDTLZ(M, k, "DTLZ4"), alpha_(alpha) {}
	virtual bool Evaluate(CIndividual *indv) const;

private:
	double alpha_;
};



// ----------------------------------------------------------------------
//		CProblemDTLZ5
// ----------------------------------------------------------------------

class CProblemDTLZ5 : public CProblemDTLZ
{
public:
	explicit CProblemDTLZ5(std::size_t M, std::size_t k = 10):CProblemDTLZ(M, k, "DTLZ5") {}
	virtual bool Evaluate(CIndividual *indv) const;
};


// ----------------------------------------------------------------------
//		CProblemDTLZ6
// ----------------------------------------------------------------------

class CProblemDTLZ6 : public CProblemDTLZ
{
public:
	explicit CProblemDTLZ6(std::size_t M, std::size_t k = 10):CProblemDTLZ(M, k, "DTLZ6") {}
	virtual bool Evaluate(CIndividual *indv) const;
};


// ----------------------------------------------------------------------
//		CProblemDTLZ7
// ----------------------------------------------------------------------

class CProblemDTLZ7 : public CProblemDTLZ
{
public:
	explicit CProblemDTLZ7(std::size_t M, std::size_t k = 20):CProblemDTLZ(M, k, "DTLZ7") {}
	virtual bool Evaluate(CIndividual *indv) const;
};


#endif
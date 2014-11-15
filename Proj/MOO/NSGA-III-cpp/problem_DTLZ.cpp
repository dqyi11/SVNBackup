
#include "problem_DTLZ.h"
#include "alg_individual.h"
#include "aux_math.h"
#include "aux_misc.h"

#include <cmath>
#include <vector>

using std::size_t;
using std::cos;

// ----------------------------------------------------------------------
//		CProblemDTLZ
// ----------------------------------------------------------------------

CProblemDTLZ::CProblemDTLZ(size_t M, size_t k, const std::string &name):
	BProblem(name + "(" + IntToStr(M) + ")"), 
	M_(M), 
	k_(k)
{
	lbs_.resize(M_+k_-1, 0.0); // lower bound 0.0
	ubs_.resize(M_+k_-1, 1.0); // upper bound 1.0
}


// ----------------------------------------------------------------------
//		CProblemDTLZ1
// ----------------------------------------------------------------------
bool CProblemDTLZ1::Evaluate(CIndividual *indv) const
{
	CIndividual::TDecVec &x = indv->vars();
	CIndividual::TObjVec &f = indv->objs();

	if (x.size() != M_ + k_ - 1) return false; // #variables does not match

	double g = 0;
	for (size_t i = M_-1; i < x.size(); i += 1)
	{
		g += MathAux::square(x[i]-0.5) - cos(20*MathAux::PI*(x[i]-0.5));
	}
	g = (k_ + g)*100;


	f.resize(M_, 0);
	for (size_t m = 0; m < M_; m += 1)
	{
		double product = 0.5*(1+g);
		size_t i = 0;
		for (; M_ >= 2+m && i <= M_-2-m; i += 1)
		{
			product *= x[i];
		}
		if (m > 0)
		{
			product *= (1 - x[i]);
		}
		f[m] = product;
	}

	return true;

}// bool CProblemDTLZ1::Evaluate(CIndividual *indv) const




// ----------------------------------------------------------------------
//		CProblemDTLZ2
// ----------------------------------------------------------------------
bool CProblemDTLZ2::Evaluate(CIndividual *indv) const
{
	CIndividual::TDecVec &x = indv->vars();
	CIndividual::TObjVec &f = indv->objs();

	if (x.size() != M_ + k_ - 1) return false; // #variables does not match

	double g = 0;
	for (size_t i = M_-1; i < x.size(); i += 1)
	{
		g += MathAux::square(x[i]-0.5);
	}

	f.resize(M_, 0);
	for (size_t m = 0; m < M_; m += 1)
	{
		double product = (1+g);
		size_t i=0;
		for (; i+m<=M_-2; i+=1)
		{
			product *= cos(x[i]*MathAux::PI/2);
		}
		if (m > 0)
			product *= sin(x[i]*MathAux::PI/2);
		
		f[m] = product;
	}

	return true;

}// bool CProblemDTLZ2::Evaluate(CIndividual *indv) const





// ----------------------------------------------------------------------
//		CProblemDTLZ3
// ----------------------------------------------------------------------
bool CProblemDTLZ3::Evaluate(CIndividual *indv) const
{
	CIndividual::TDecVec &x = indv->vars();
	CIndividual::TObjVec &f = indv->objs();

	if (x.size() != M_ + k_ - 1) return false; // #variables does not match

	double g = 0;
	for (size_t i = M_-1; i < x.size(); i += 1)
	{
		g += MathAux::square(x[i]-0.5) - cos(20*MathAux::PI*(x[i]-0.5));
	}
	g = (k_ + g)*100;

	f.resize(M_, 0);
	for (size_t m = 0; m < M_; m += 1)
	{
		double product = (1+g);
		size_t i=0;
		for (; i+m<=M_-2; i+=1)
		{
			product *= cos(x[i]*MathAux::PI/2);
		}
		if (m > 0)
			product *= sin(x[i]*MathAux::PI/2);
		
		f[m] = product;
	}

	return true;

}// bool CProblemDTLZ3::Evaluate(CIndividual *indv) const




// ----------------------------------------------------------------------
//		CProblemDTLZ4
// ----------------------------------------------------------------------
bool CProblemDTLZ4::Evaluate(CIndividual *indv) const
{
	CIndividual::TDecVec &x = indv->vars();
	CIndividual::TObjVec &f = indv->objs();

	if (x.size() != M_ + k_ - 1) return false; // #variables does not match

	double g = 0;
	for (size_t i = M_-1; i < x.size(); i += 1)
	{
		g += MathAux::square(x[i]-0.5);
	}

	f.resize(M_, 0);
	for (size_t m = 0; m < M_; m += 1)
	{
		double product = (1+g);
		size_t i=0;
		for (; i+m<=M_-2; i+=1)
		{
			product *= cos( pow(x[i], alpha_)*MathAux::PI/2 );
		}
		if (m > 0)
			product *= sin( pow(x[i], alpha_)*MathAux::PI/2 );
		
		f[m] = product;
	}

	return true;

}// bool CProblemDTLZ4::Evaluate(CIndividual *indv) const





// ----------------------------------------------------------------------
//		CProblemDTLZ5
// ----------------------------------------------------------------------
bool CProblemDTLZ5::Evaluate(CIndividual *indv) const
{
	CIndividual::TDecVec &x = indv->vars();
	CIndividual::TObjVec &f = indv->objs();

	if (x.size() != M_ + k_ - 1) return false; // #variables does not match

	double g = 0;
	for (size_t i = M_-1; i < x.size(); i += 1)
	{
		g += MathAux::square(x[i]-0.5);
	}

	std::vector<double> theta(x.size());
	theta[0] = x[0]*MathAux::PI/2.0; 
	for (size_t i=1; i<theta.size(); i+=1)
	{
		theta[i] = MathAux::PI/(4*(1+g))*(1+2*g*x[i]);
	}

	f.resize(M_, 0);
	for (size_t m = 0; m < M_; m += 1)
	{
		double product = (1+g);
		size_t i=0;
		for (; i+m<=M_-2; i+=1)
		{
			product *= cos(theta[i]);
		}
		if (m > 0)
			product *= sin(theta[i]);
		
		f[m] = product;
	}

	return true;

}// bool CProblemDTLZ5::Evaluate(CIndividual *indv) const




// ----------------------------------------------------------------------
//		CProblemDTLZ6
// ----------------------------------------------------------------------
bool CProblemDTLZ6::Evaluate(CIndividual *indv) const
{
	CIndividual::TDecVec &x = indv->vars();
	CIndividual::TObjVec &f = indv->objs();

	if (x.size() != M_ + k_ - 1) return false; // #variables does not match

	double g = 0;
	for (size_t i = M_-1; i < x.size(); i += 1)
	{
		g += pow(x[i], 0.1);
	}

	// the following is the same as DTLZ5
	std::vector<double> theta(x.size());
	theta[0] = x[0]*MathAux::PI/2.0; 
	for (size_t i=1; i<theta.size(); i+=1)
	{
		theta[i] = MathAux::PI/(4*(1+g))*(1+2*g*x[i]);
	}

	f.resize(M_, 0);
	for (size_t m = 0; m < M_; m += 1)
	{
		double product = (1+g);
		size_t i=0;
		for (; i+m<=M_-2; i+=1)
		{
			product *= cos(theta[i]);
		}
		if (m > 0)
			product *= sin(theta[i]);
		
		f[m] = product;
	}

	return true;

}// bool CProblemDTLZ6::Evaluate(CIndividual *indv) const





// ----------------------------------------------------------------------
//		CProblemDTLZ7
// ----------------------------------------------------------------------
bool CProblemDTLZ7::Evaluate(CIndividual *indv) const
{
	CIndividual::TDecVec &x = indv->vars();
	CIndividual::TObjVec &f = indv->objs();

	if (x.size() != M_ + k_ - 1) return false; // #variables does not match

	for (size_t m = 0; m < M_-1; m += 1)
	{
		f[m] = x[m];
	}

	double g = 0;
	for (size_t i = M_-1; i < x.size(); i += 1)
	{
		g += x[i];
	}
	g = 1 + 9*g/k_;

	double h = M_;
	for (size_t i = 0; i < M_-1; i += 1)
	{
		h -= f[i]/(1+g)*(1+sin(3*MathAux::PI*f[i]));
	}

	f[M_-1] = (1+g)*h;

	return true;

}// bool CProblemDTLZ7::Evaluate(CIndividual *indv) const
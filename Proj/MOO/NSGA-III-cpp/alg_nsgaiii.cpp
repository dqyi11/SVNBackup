#include "alg_nsgaiii.h"
#include "problem_base.h"
#include "alg_individual.h"
#include "alg_reference_point.h"
#include "alg_population.h"

#include "alg_initialization.h"
#include "alg_crossover.h"
#include "alg_mutation.h"
#include "alg_environmental_selection.h"

#include "gnuplot_interface.h"
#include "log.h"
#include "windows.h" // for Sleep()

#include <vector>
#include <fstream>

using namespace std;

CNSGAIII::CNSGAIII(): 
	name_("NSGAIII"),
	gen_num_(1),
	pc_(1.0), // default setting in NSGA-III (IEEE tEC 2014)
	eta_c_(30), // default setting
	eta_m_(20) // default setting
{
}
// ----------------------------------------------------------------------
void CNSGAIII::Setup(ifstream &ifile)
{
	if (!ifile) return;

	string dummy;
	ifile >> dummy >> dummy >> name_;

	size_t p1 = 0, p2 = 0;
	ifile >> dummy >> dummy >> p1 >> p2;

	obj_division_p_.push_back(p1);

	if (!ifile) ifile.clear();
	else obj_division_p_.push_back(p2);
	
	ifile >> dummy >> dummy >> gen_num_;
	ifile >> dummy >> dummy >> pc_;
	ifile >> dummy >> dummy >> eta_c_;
	ifile >> dummy >> dummy >> eta_m_;
}
// ----------------------------------------------------------------------
void CNSGAIII::Solve(CPopulation *solutions, const BProblem &problem)
{
	CIndividual::SetTargetProblem(problem);
	
	vector<CReferencePoint> rps;
	GenerateReferencePoints(&rps, problem.num_objectives(), obj_division_p_); 
	size_t PopSize = rps.size();
	while (PopSize%4) PopSize += 1;

	CPopulation pop[2]={CPopulation(PopSize)};
	CSimulatedBinaryCrossover SBX(pc_, eta_c_);
	CPolynomialMutation PolyMut(1.0/problem.num_variables(), eta_m_);

	Gnuplot gplot;

	int cur = 0, next = 1;
	RandomInitialization(&pop[cur], problem);
	for (size_t i=0; i<PopSize; i+=1)
	{
		problem.Evaluate(&pop[cur][i]);
	}

	for (size_t t=0; t<gen_num_; t+=1)
	{
		pop[cur].resize(PopSize*2);

		for (size_t i=0; i<PopSize; i+=2)
		{
			int father = rand()%PopSize,
				mother = rand()%PopSize;

			SBX(&pop[cur][PopSize+i], &pop[cur][PopSize+i+1], pop[cur][father], pop[cur][mother]);

			PolyMut(&pop[cur][PopSize+i]);
			PolyMut(&pop[cur][PopSize+i+1]);

			problem.Evaluate(&pop[cur][PopSize+i]);
			problem.Evaluate(&pop[cur][PopSize+i+1]);
		}

		EnvironmentalSelection(&pop[next], &pop[cur], rps, PopSize);

		ShowPopulation(gplot, pop[next], "pop"); Sleep(50);

		std::swap(cur, next);
	}

	*solutions = pop[cur];
}
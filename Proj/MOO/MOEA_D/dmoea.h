#ifndef __MOEAD_H_
#define __MOEAD_H_

#include "global.h"
#include "common.h"
#include "individual.h"
#include "scalarfunc.h"
#include "recombination.h"

class TMOEAD    
{
public:

	TMOEAD();
	virtual ~TMOEAD();

	void init_uniformweight(int sd);    // initialize the weights for subproblems
	void init_neighbourhood();          // calculate the neighbourhood of each subproblem
	void init_population();             // initialize the population


	void update_reference(TIndividual &ind);           // update the approximation of ideal point
	void update_problem(TIndividual &child, int id);   // compare and update the neighboring solutions
	void evolution();                                  // mating restriction, recombination, mutation, update
	void run(int sd, int nc, int mg, int rn);          // execute MOEAD
	void save_front(char savefilename[1024]);          // save the pareto front into files

    vector <TSOP>  population;  // current population      	   
	TIndividual *indivpoint;    // reference point
	int  niche;                 // neighborhood size
	int  pops;                  // population   size	

	void operator=(const TMOEAD &emo);
};

TMOEAD::TMOEAD()
{

	idealpoint = new double[numObjectives];
	indivpoint = new TIndividual[numObjectives];    
	// initialize ideal point	
    for(int n=0; n<numObjectives; n++) 
	{
		idealpoint[n] = 1.0e+30;  
		indivpoint[n].rnd_init();
		indivpoint[n].obj_eval();
	}
}

TMOEAD::~TMOEAD()
{
	delete [] idealpoint;    
	delete [] indivpoint;
}


void TMOEAD::init_population()
{
    for(int i=0; i<pops; i++)
	{
		population[i].indiv.rnd_init();
		population[i].indiv.obj_eval();
		update_reference(population[i].indiv);
	}
}


// initialize a set of evely-distributed weight vectors
void TMOEAD::init_uniformweight(int sd)
{   
    for(int i=0; i<=sd; i++)
	{
		if(numObjectives==2)
		{
            TSOP sop;		   
			sop.array.push_back(i);
			sop.array.push_back(sd-i);
		    for(int j=0; j<sop.array.size(); j++)
				sop.namda.push_back(1.0*sop.array[j]/sd);
			population.push_back(sop); 
		}
		else
		{
			for(int j=0;j<=sd;j++)
			{
				if(i+j<=sd)
				{
				    TSOP sop;		   
					sop.array.push_back(i);
					sop.array.push_back(j);
					sop.array.push_back(sd-i-j);
		            for(int k=0; k<sop.array.size(); k++)
						sop.namda.push_back(1.0*sop.array[k]/sd);
					population.push_back(sop); 
				}
			}
		}
	}
	pops = population.size();
}

// initialize the neighborhood of subproblems based on the distances of weight vectors
void TMOEAD::init_neighbourhood()
{
    double *x   = new double[pops];
	int    *idx = new int[pops];
	for(int i=0; i<pops; i++)
	{	
		for(int j=0; j<pops; j++)
		{
		    x[j]    = distanceVector(population[i].namda,population[j].namda);
			idx[j]  = j;			
		}
		minfastsort(x,idx,pops,niche);   
		for(int k=0; k<niche; k++)   
			population[i].table.push_back(idx[k]);

	}
    delete [] x;
	delete [] idx;
}

// update the best solutions of neighboring subproblems
void TMOEAD::update_problem(TIndividual &indiv, int id)
{
    for(int i=0; i<niche; i++)
	{
		int    k  = population[id].table[i];
		double f1, f2;
		f1 = scalar_func(population[k].indiv.y_obj, population[k].namda, indivpoint);
		f2 = scalar_func(indiv.y_obj, population[k].namda, indivpoint);
		if(f2<f1) population[k].indiv = indiv;
	}
}


// update the reference point
void TMOEAD::update_reference(TIndividual &ind)
{
	for(int n=0; n<numObjectives; n++)    
	{
		if(ind.y_obj[n]<idealpoint[n])
		{
			idealpoint[n]  = ind.y_obj[n];
			indivpoint[n]  = ind;
		}		
	}
}


// recombination, mutation, update in MOEA/D
void TMOEAD::evolution()
{
    for(int i=0; i<population.size(); i++)
	{
		int   n  =  i; 
		int   s  = population[n].table.size();
		int   r1 = int(s*rnd_uni(&rnd_uni_init));
		int   r2 = int(s*rnd_uni(&rnd_uni_init));
		int   p1 = population[n].table[r1];
		int   p2 = population[n].table[r2];
		TIndividual child, child2;
		realbinarycrossover(population[p1].indiv,population[p2].indiv,child, child2);  
		realmutation(child, 1.0/numVariables);
		child.obj_eval();
		update_reference(child);
		update_problem(child, n);
	}
}



void TMOEAD::run(int sd, int nc, int mg, int rn)
{
    // sd: integer number for generating weight vectors
	// nc: size of neighborhood
	// mg: maximal number of generations 
	niche = nc;	
	init_uniformweight(sd);
    init_neighbourhood();
	init_population();	
	for(int gen=2; gen<=mg; gen++)   evolution();      
	char savefilename[1024];
    sprintf(savefilename,"ParetoFront/DMOEA_%s_R%d.dat",strTestInstance,rn);
	save_front(savefilename);
	population.clear();
}


void TMOEAD::save_front(char saveFilename[1024])
{
    std::fstream fout;
	fout.open(saveFilename,std::ios::out);
	for(int n=0; n<population.size(); n++)
	{
		for(int k=0;k<numObjectives;k++)
			fout<<population[n].indiv.y_obj[k]<<"  ";
		fout<<"\n";
	}
	fout.close();
}


void TMOEAD::operator=(const TMOEAD &emo)
{
    pops        = emo.pops;
	population  = emo.population;
	indivpoint  = emo.indivpoint;
	niche       = emo.niche;
} 


#endif
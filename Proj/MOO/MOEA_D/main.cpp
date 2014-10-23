/*==========================================================================
//  Implementation of Multiobjective Evolutionary Algorithm Based on
//  Decomposition (MOEA/D) For Continuous Multiobjective Optimization Problems (2006)
//
//  See the details of MOEA/D in the following paper
//  Q. Zhang and H. Li, MOEA/D: A Multi-objective Evolutionary Algorithm Based on Decomposition,
//  IEEE Trans. on Evolutionary Computation, in press, 2007
//
//  The source code of MOEA/D was implemented by Hui Li and Qingfu Zhang
//
//  If you have any questions about the codes, please contact
//  Qingfu Zhang at qzhang@essex.ac.uk  or Hui Li at hzl@cs.nott.ac.uk
===========================================================================*/


#include "global.h"
#include "dmoea.h"

void main()
{

    // set the type of decomposition method
    // "_TCH1": Tchebycheff, "_TCH2": normalized Tchebycheff, "_PBI": Penalty-based BI
    strcpy(strFunctionType,"_TCH1");

    int  total_run       = 1;         // totoal number of runs
    int  max_gen         = 250;       // maximal number of generations
    int  niche           = 20;        // neighborhood size

    char *instances[]  = {"ZDT1","ZDT2","ZDT3","ZDT4","ZDT6","DTLZ1","DTLZ2"}; // names of test instances
    int  nvars[]       = {30, 30, 30, 10, 10, 10, 10};                         // number of variables
    int  nobjs[]       = {2, 2, 2, 2, 2, 3, 3};                                // number of objectives

    for(int n=0; n<7; n++)
    {
        strcpy(strTestInstance,instances[n]);
        numVariables  = nvars[n];
        numObjectives = nobjs[n];

        for(int run=1; run<=total_run; run++)
        {
            seed = (seed + 111)%1235;
            rnd_uni_init = -(long)seed;
            TMOEAD  MOEAD;

            if(numObjectives==3)
            {
                MOEAD.run(23, niche, max_gen, run);  //23 -3  popsize 300
            }

            if(numObjectives==2)
            {
                MOEAD.run(99, niche, max_gen, run);  //99 -2  popsize 100
            }
        }
    }
    return;
}

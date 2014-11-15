
#include "log.h"
#include "alg_population.h"
#include "gnuplot_interface.h"

#include <fstream>
using namespace std;

//#define OUTPUT_DECISION_VECTOR

bool SaveToFile(const std::string &fname, const CPopulation &pop, ios_base::openmode mode)
{
	ofstream ofile(fname.c_str(), mode);
	if (!ofile) return false;

	for (size_t i=0; i<pop.size(); i+=1)
	{
#ifdef OUTPUT_DECISION_VECTOR
		for (size_t j=0; j<pop[i].vars().size(); j+=1)
		{
			ofile << pop[i].vars()[j] << ' ';
		}
#endif

		for (size_t f=0; f<pop[i].objs().size(); f+=1)
		{
			ofile << pop[i].objs()[f] << ' ';
		}
		ofile << endl;
	}
	ofile << endl;
	return true;
}
// ----------------------------------------------------------------------
bool ShowPopulation(Gnuplot &gplot, const CPopulation &pop, const std::string &legend)
{
	if (!SaveToFile(legend, pop, ios_base::out)) return false;

	size_t n = 0;
#ifdef OUTPUT_DECISION_VECTOR
	n = pop[0].vars().size();
#endif
	
	if (pop[0].objs().size() == 2)
	{
		gplot.plot(legend, n+1, n+2);
		return true;
	}
	else if (pop[0].objs().size() == 3)
	{
		gplot.splot(legend, n+1, n+2, n+3);
		return true;
	}
	else // only plot 2-D or 3-D figures
		return false;
}
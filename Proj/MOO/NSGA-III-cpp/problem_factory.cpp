
#include "problem_factory.h"
#include "problem_DTLZ.h"
#include "problem_self.h"
#include <string>
using namespace std;

BProblem *GenerateProblem(std::ifstream &ifile)
{
	string pname, dummy;

	ifile >> dummy >> dummy >> pname;
	if (pname == "Self")
	{
		// define your own file format, read it, and generate an instance here

		size_t num_vars, num_objs;

		ifile >> dummy >> dummy >> num_vars;
		ifile >> dummy >> dummy >> num_objs;

		return new CProblemSelf(num_vars, num_objs);
	}
	else // assuming that the remaining problems are DTLZ and follow the same format
	{
		size_t num_objs;

		ifile >> dummy >> dummy >> num_objs;

		if (pname == "DTLZ1") return new CProblemDTLZ1(num_objs);
		else if (pname == "DTLZ2") return new CProblemDTLZ2(num_objs);
		else if (pname == "DTLZ3") return new CProblemDTLZ3(num_objs);
		else if (pname == "DTLZ4") return new CProblemDTLZ4(num_objs);
		else return 0;
	}
	
	return 0;
}

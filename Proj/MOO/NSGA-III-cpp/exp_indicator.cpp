
#include "exp_indicator.h"
#include <sstream>
#include <fstream>
#include <cmath>
#include <limits>
#include <algorithm>
using namespace std;

ostream & operator << (ostream &os, const CObjectiveVector &objvec)
{
	for (size_t i=0; i<objvec.size(); i+=1)
	{
		os << objvec[i] << ' ';
	}
	return os;
}
// ---------------------------------------------------------------------
istream & operator >> (istream &is, CObjectiveVector &objvec)
{
	char str[1000]={};
	is.getline(str, 1000);

	istringstream iss(str);

	CObjectiveVector::ValueType val;
	objvec.clear();
	while (iss >> val)
	{
		objvec.push_back(val);
	}
	return is;
}
// ---------------------------------------------------------------------
TFront & LoadFront(TFront &front, const std::string &infname) // return the number of vectors loaded successfully
{
	front.clear();

	ifstream ifile(infname);
	if (!ifile) return front;

	CObjectiveVector objvec;

	while (ifile >> objvec && objvec.size()>0)
	{
		front.push_back(objvec);
	}
	
	return front;
}
// ---------------------------------------------------------------------
double EuclideanDistance(const CObjectiveVector &l, const CObjectiveVector &r)
{
    double sum = 0;
    for (size_t i=0; i<l.size(); i+=1)
    {
        sum += pow(l[i]-r[i],2);
    }
    return sqrt(sum);
}
// ---------------------------------------------------------------------
double IGD(const TFront &PF, const TFront &approximation)
{
    double sum = 0;
    for (size_t p=0; p<PF.size(); p+=1)
    {
        double min_dist = numeric_limits<double>::max();
        for (size_t a=0; a<approximation.size(); a+=1)
        {
			min_dist = min(min_dist, EuclideanDistance(PF[p], approximation[a]));
        }
        sum += min_dist;
    }
    return PF.size()>0?sum/PF.size():-1;
}
// ---------------------------------------------------------------------


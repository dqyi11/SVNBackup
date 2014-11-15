#ifndef INDICATOR__
#define INDICATOR__

#include <iostream>
#include <vector>

class CObjectiveVector
{
public:
	typedef double ValueType;

	const ValueType & operator[](size_t i) const { return v_[i]; }
	ValueType & operator[](size_t i) { 
		return const_cast<ValueType &>( static_cast<const CObjectiveVector &>(*this)[i] );
	}
	size_t size() const { return v_.size(); }

	void clear() { v_.clear(); }
	void push_back(const ValueType &v) { v_.push_back(v); }
private:
	std::vector<ValueType> v_;
};

std::ostream & operator << (std::ostream &os, const CObjectiveVector &objvec);
std::istream & operator >> (std::istream &is, CObjectiveVector &objvec);
// ---------------------------------------------------------------------
typedef std::vector<CObjectiveVector> TFront;


TFront & LoadFront(TFront &front, const std::string &infname); // return the number of vectors loaded successfully
double IGD(const TFront &PF, const TFront &approximation);


#endif
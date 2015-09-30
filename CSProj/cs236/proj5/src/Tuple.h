/*
 * Tuple.h
 *
 *  Created on: Feb 19, 2013
 *      Author: walter
 */

#ifndef TUPLE_H_
#define TUPLE_H_

#include <map>
#include <string>

class Tuple {
public:
	Tuple();
	virtual ~Tuple();

	std::string toString();

	bool compare(Tuple & tuple);

	int compareTo(Tuple & tuple);

	std::map<std::string, std::string> mTuples;
};

#endif /* TUPLE_H_ */

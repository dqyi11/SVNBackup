/*
 * Domain.h
 *
 *  Created on: Feb 6, 2013
 *      Author: kyle
 */

#ifndef DOMAIN_H_
#define DOMAIN_H_

#include <sstream>
#include <string>
#include <set>

namespace std {

class Domain {
public:
	Domain();
	void addElement(string member);
	string toString();
private:
	set<string> elements;
};

} /* namespace std */
#endif /* DOMAIN_H_ */

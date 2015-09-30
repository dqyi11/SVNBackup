/*
 * Domain.cpp
 *
 *  Created on: Feb 6, 2013
 *      Author: kyle
 */

#include "Domain.h"

namespace std {

Domain::Domain() {}

void Domain::addElement(string member) {
	elements.insert(member);
}

string Domain::toString() {
	stringstream rep;
	rep << "Domain(" << elements.size() << "):" << endl;
	for(set<string>::iterator it = elements.begin(); it != elements.end(); ++it)
		rep << "  " << (*it) << endl;

	return rep.str();
}

} /* namespace std */

/*
 * DatalogElement.h
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#ifndef DATALOGELEMENT_H_
#define DATALOGELEMENT_H_

#include <string>

class DatalogElement {
public:
	DatalogElement() {};
	virtual ~DatalogElement() {};

	virtual std::string toString() = 0;
};

#endif /* DATALOGELEMENT_H_ */

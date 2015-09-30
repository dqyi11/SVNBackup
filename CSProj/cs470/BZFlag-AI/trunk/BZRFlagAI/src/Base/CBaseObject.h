/*
 * CBaseObject.h
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#ifndef CBASEOBJECT_H_
#define CBASEOBJECT_H_

class CBaseObject {
public:
	CBaseObject();
	virtual ~CBaseObject();

	virtual void print() = 0;
};

#endif /* CBASEOBJECT_H_ */

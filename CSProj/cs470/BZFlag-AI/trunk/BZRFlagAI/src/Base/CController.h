/*
 * CController.h
 *
 *  Created on: Sep 30, 2012
 *      Author: walter
 */

#ifndef CCONTROLLER_H_
#define CCONTROLLER_H_

typedef struct ControlParams_t
{
	double speed;
	double angleVelocity;

}ControlParams;

class CController {
public:
	CController();
	virtual ~CController();
};

#endif /* CCONTROLLER_H_ */

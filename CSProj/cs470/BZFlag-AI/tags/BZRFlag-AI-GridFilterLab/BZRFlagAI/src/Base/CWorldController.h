/*
 * CWorldController.h
 *
 *  Created on: Sep 26, 2012
 *      Author: walter
 */

#ifndef CWORLDCONTROLLER_H_
#define CWORLDCONTROLLER_H_

#include "CWorld.h"
#include "CCommunicator.h"

#define GET_WORLDCTRL()    CWorldController::getInstance()
#define GET_WORLD()        GET_WORLDCTRL()->getWorld()
#define GET_COMMUNICATOR() GET_WORLDCTRL()->getCommunicator();


class CWorldController {
public:
	virtual ~CWorldController();

	bool init(CCommunicator * communicator);

public:

	static CWorldController * getInstance();
	CCommunicator * getCommunicator() { return mpCommunicator; };
	CWorld * getWorld() { return mpWorld; };

	bool updateWorld(void);

private:
	CWorldController();

	bool updateMyAgents(void);
	bool updateFlags(void);

	bool initConstants();
	bool initTeams();
	bool initBases();
    bool initObstacles();
	bool initMyAgents();
	bool initOtherAgents();
	bool initFlags();

	CCommunicator * mpCommunicator;
	CWorld * mpWorld;

	static CWorldController * mpInstance;
};

#endif /* CWORLDCONTROLLER_H_ */

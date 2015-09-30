/*
 * CPotentialFieldManager.h
 *
 *  Created on: Sep 30, 2012
 *      Author: walter
 */

#ifndef CPOTENTIALFIELDMANAGER_H_
#define CPOTENTIALFIELDMANAGER_H_


#include "CFlagBaseAttractivePotentialField.h"
#include "CObstacleRepulsivePotentialField.h"
#include "CObstacleTangentialPotentialField.h"
#include <vector>

class Gnuplot;

class CPotentialFieldManager {
	friend class CPotentialFieldAgentController;
public:
	CPotentialFieldManager();
	virtual ~CPotentialFieldManager();

	void update();
	void init();

	bool addAttractiveFlagBase(CFlag * pFlag, CBase * pBase);
	void clearAttractiveFlagBase();
	bool addRepulsiveObstacle(CObstacle * pObstacle);
	bool addTangentialObstacle(CObstacle * pObstacle);

	bool removeAttractiveFlagBase(CFlag * pFlag, CBase * pBase);

	void drawPotentialMap(void);

	CVector getVector(double x, double y);

protected:
	vector<CFlagBaseAttractivePotentialField> mBasesAttractive;
	vector<CObstacleRepulsivePotentialField> mObstaclesRepulsive;
	vector<CObstacleTangentialPotentialField> mObstaclesTangential;

};

#endif /* CPOTENTIALFIELDMANAGER_H_ */

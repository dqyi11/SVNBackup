/*
 * CPathPlanningGraphCreator.h
 *
 *  Created on: Jan 11, 2013
 *      Author: walter
 */

#ifndef CPATHPLANNINGGRAPHCREATOR_H_
#define CPATHPLANNINGGRAPHCREATOR_H_

#include "../Base/CHexaGridGraph.h"
#include "../Base/CPathPlanningGraph.h"
#include "../Visualizer/CVisualHexagonDiscretizedMap.h"
#include "../Visualizer/CPathPlanningGraphVisualizer.h"
#include "../Base/CHuman.h"

class CPathPlanningGraphCreator {
public:
	CPathPlanningGraphCreator();
	virtual ~CPathPlanningGraphCreator();

	void setGridGraph(CHexaGridGraph * gridGraph) { mpGridGraph = gridGraph; };
	CHexaGridGraph * getGridGraph() { return mpGridGraph; };

	void setMap(CVisualHexagonDiscretizedMap * map) { mpMap = map; };
	CVisualHexagonDiscretizedMap * getMap() { return mpMap; };

	void setHuman(CHuman * human) { mpHuman = human; };
	CHuman * getHuman() { return mpHuman; };

	CPathPlanningGraph * getPlanningGraph() { return mpPathPlanningGraph; };

	bool init();

	void visualizeGraph(const char * filename);

private:
	CVisualHexagonDiscretizedMap * mpMap;
	CHexaGridGraph * mpGridGraph;
	CPathPlanningGraph * mpPathPlanningGraph;
	CHuman * mpHuman;

	CPathPlanningGraphVisualizer * mpVisualizer;

	int mPlanningTimeLength;
};

#endif /* CPATHPLANNINGGRAPHCREATOR_H_ */

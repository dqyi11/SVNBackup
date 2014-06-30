/*
 * CPathPlanningGraphPruner.h
 *
 *  Created on: Jan 15, 2013
 *      Author: walter
 */

#ifndef CPATHPLANNINGGRAPHPRUNER_H_
#define CPATHPLANNINGGRAPHPRUNER_H_

#include "../Base/CPathPlanningGraph.h"
#include "../Visualizer/CPathPlanningGraphVisualizer.h"
#include "../Visualizer/CVisualHexagonDiscretizedMap.h"

class CPathPlanningGraphPruner {
public:
	CPathPlanningGraphPruner(CPathPlanningGraph * graph);
	virtual ~CPathPlanningGraphPruner();

	void setStartVertex(string name) { mStartVertexName = name; };

	void setDiscretizedMap(CVisualHexagonDiscretizedMap * map) { mpMap = map; };

	bool pruneGraph();

	bool pruneByStartVertex();
	bool pruneByObstacle();
	bool pruneForward();
	bool pruneBackward();

	CPathPlanningGraph * getGraph() { return mpPathPlanningGraph; };

	void visualizeGraph(const char * filename);

private:
	CPathPlanningGraph * mpPathPlanningGraph;
	string mStartVertexName;

	CPathPlanningGraphVisualizer * mpVisualizer;
	CVisualHexagonDiscretizedMap * mpMap;
};

#endif /* CPATHPLANNINGGRAPHPRUNER_H_ */

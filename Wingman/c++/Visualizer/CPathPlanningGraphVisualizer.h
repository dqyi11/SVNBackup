/*
 * CPathPlanningGraphVisualizer.h
 *
 *  Created on: Jan 11, 2013
 *      Author: walter
 */

#ifndef CPATHPLANNINGGRAPHVISUALIZER_H_
#define CPATHPLANNINGGRAPHVISUALIZER_H_

#include <gvc.h>
#include "../Base/CPathPlanningGraph.h"

class CVisualLevelVertex
{
public:
	CVisualLevelVertex();
	virtual ~CVisualLevelVertex();

	CLevelVertex * mpVertex;
	Agnode_t     * mpVisualHandle;
};

class CVisualDirectedEdge
{
public:
	CVisualDirectedEdge();
	virtual ~CVisualDirectedEdge();

	CDirectedEdge * mpEdge;
    Agedge_t      * mpVisualHandle;
};


class CPathPlanningGraphVisualizer {
public:
	CPathPlanningGraphVisualizer(CPathPlanningGraph * graph);
	virtual ~CPathPlanningGraphVisualizer();

	CVisualLevelVertex * findVisualLevelVertex(CLevelVertex * vertex);

	void draw(const char * filename);

	CPathPlanningGraph * mpGraph;

	list<CVisualLevelVertex*> mVisualVertexList;
	list<CVisualDirectedEdge*>   mVisualEdgeList;
};

#endif /* CPATHPLANNINGGRAPHVISUALIZER_H_ */

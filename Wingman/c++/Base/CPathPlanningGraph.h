/*
 * CPathPlanningGraph.h
 *
 *  Created on: Jan 11, 2013
 *      Author: walter
 */

#ifndef CPATHPLANNINGGRAPH_H_
#define CPATHPLANNINGGRAPH_H_

#include "CDirectedGraph.h"
#include "CHexaGridGraph.h"

class CLevelVertex : public CVertex
{
public:
	CLevelVertex();
	virtual ~CLevelVertex();

	int mLevel;
	CHexaVertex * mpHexaVertex;
};

class CLevelVertexSet
{
public:
	CLevelVertexSet();
	virtual ~CLevelVertexSet();

	void addLevelVertex(CLevelVertex * levelVertex);
	bool hasLevelVertex(CLevelVertex * levelVertex);

	void setLevel(int level) { mLevel = level; };

	int mLevel;
	list<CLevelVertex*> mSet;
};

class CPathPlanningGraph: public CDirectedGraph {
public:
	CPathPlanningGraph(int planningLength);
	virtual ~CPathPlanningGraph();

	bool addVertex(CLevelVertex * vertex, int level);
	CLevelVertex * createVertex(string name, int level);

	CLevelVertex * findVertex(string name, int level);

	CDirectedEdge * addEdge(CLevelVertex * vertexStart, CLevelVertex * vertexEnd);

	bool removeVertex(string name, int level);
	bool removeAllOtherVertex(string name, int level);

	CEdgeSet findEdgeByStart(CLevelVertex * vertexStart);
	CEdgeSet findEdgeByEnd(CLevelVertex * vertexEnd);

	int getPlanningLength() { return mPlanningLength; };

	void printLevelVertex();

	CLevelVertexSet * mpLevelSets;
	int mPlanningLength;
};

#endif /* CPATHPLANNINGGRAPH_H_ */

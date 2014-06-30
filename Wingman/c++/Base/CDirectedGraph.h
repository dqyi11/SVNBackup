/*
 * CDirectedGraph.h
 *
 *  Created on: Jan 7, 2013
 *      Author: walter
 */

#ifndef CDIRECTEDGRAPH_H_
#define CDIRECTEDGRAPH_H_

#include "CGraph.h"

class CDirectedEdge : public CEdge {
public:
	CDirectedEdge();
	virtual ~CDirectedEdge();

    bool mFromAToB;
};

class CDirectedGraph : public CGraph {
public:
	CDirectedGraph();
	virtual ~CDirectedGraph();


	CDirectedEdge * findEdge(CVertex * vertexStart, CVertex * vertexEnd);

	CDirectedEdge * addEdge(string idStart, string idEnd);
	CDirectedEdge * addEdge(CVertex * vertexStart, CVertex * vertexEnd);

	bool isConnected(CVertex * vertexStart, CVertex * vertexEnd);

	bool removeVertex(string id);
};

#endif /* CDIRECTEDGRAPH_H_ */

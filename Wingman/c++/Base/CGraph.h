/*
 * CGraph.h
 *
 *  Created on: Jan 7, 2013
 *      Author: walter
 */

#ifndef CGRAPH_H_
#define CGRAPH_H_

#include <string>
#include <list>

using namespace std;

class CVertex
{
public:
	CVertex();
	virtual ~CVertex();

	string mName;
	string mId;

};

class CEdge
{
public:
	CEdge();
	virtual ~CEdge();

	CVertex * mpVertexA;
	CVertex * mpVertexB;
};

class CEdgeSet
{
public:
	CEdgeSet();
	virtual ~CEdgeSet();

	void addEdge(CEdge * pEdge);
	bool hasEdge(CEdge * pEdge);

	list<CEdge*> mSet;
};

class CGraph {
public:
	CGraph();
	virtual ~CGraph();

	CVertex * createVertex(string name);
	CVertex * findVertex(string id);

	CEdge * findEdge(CVertex * vertexA, CVertex * vertexB);

	CEdge * addEdge(string idA, string idB);
	CEdge * addEdge(CVertex * vertexA, CVertex * vertexB);

	void addVertex(CVertex * vertex)   { mVertexList.push_back(vertex); };
	void addEdge(CEdge * edge)   { mEdgeList.push_back(edge); };

	void printVertex();
	void printEdge();

	list<CVertex*> mVertexList;
	list<CEdge*>   mEdgeList;
};

#endif /* CGRAPH_H_ */

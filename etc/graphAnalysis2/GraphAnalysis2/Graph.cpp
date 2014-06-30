#include "StdAfx.h"
#include "Graph.h"
#include <iostream>

using namespace std;

CVertex::CVertex(string name)
{
	mName = name;
}

CVertex::~CVertex()
{
}

void CVertex::print()
{
	cout << "Vertex: " << mName << " @ " << mIndex << endl;
}

CEdge::CEdge(CVertex * a, CVertex * b)
{
	mpA = a;
	mpB = b;
}

CEdge::~CEdge()
{
	mpA = NULL;
	mpB = NULL;
}

void CEdge::print()
{
	cout << "Edge: " << mpA->mName << " - " << mpB->mName << endl;
}

CPartite::CPartite(string name)
{
	mName = name;
}

CPartite::~CPartite()
{
	mVertices.clear();
}

void CPartite::add(CVertex * pVertex)
{
	mVertices.push_back(pVertex);
}

void CPartite::print()
{
	vector<CVertex*>::iterator it;
	cout << "Partite: " << mName << endl;
	for(it=mVertices.begin();it!=mVertices.end();it++)
	{
		(*it)->print();
	}
}

CConnector::CConnector(CPartite * a, CPartite * b)
{
	mpA = a;
	mpB = b;
}

CConnector::~CConnector()
{
	mEdges.clear();
}

void CConnector::add(CEdge* pEdge)
{
	mEdges.push_back(pEdge);
}

void CConnector::print()
{
	vector<CEdge*>::iterator it;
	cout << "Connector: " << mpA->mName << " - " << mpB->mName << endl;
	for(it=mEdges.begin();it!=mEdges.end();it++)
	{
		(*it)->print();
	}
}

CGraph::CGraph(void)
{
}

CGraph::~CGraph(void)
{
	mVertices.clear();
	mEdges.clear();
	mPartites.clear();
	mConnectors.clear();
}

CVertex * CGraph::createVertex(string name)
{
	CVertex * pVertex = new CVertex(name);
	pVertex->mIndex = mVertices.size();
	mVertices.push_back(pVertex);
	return pVertex;
}

CPartite * CGraph::createPartite(string name)
{
	CPartite * pPartite = new CPartite(name);
	mPartites.push_back(pPartite);
	return pPartite;
}

CEdge * CGraph::connect(CVertex * a, CVertex * b)
{
	CEdge * pEdge = new CEdge(a, b);
	pEdge->mIndex = mEdges.size();
	mEdges.push_back(pEdge);
	return pEdge;
}

CConnector * CGraph::connect(CPartite * a, CPartite * b)
{
	CConnector * pConnector = new CConnector(a, b);
	mConnectors.push_back(pConnector);
	return pConnector;
}

void CGraph::printVertex()
{
	vector<CPartite*>::iterator it;
	for(it=mPartites.begin();it!=mPartites.end();it++)
	{
		(*it)->print();
	}	
}

void CGraph::printEdge()
{
	vector<CConnector*>::iterator it;
	for(it=mConnectors.begin();it!=mConnectors.end();it++)
	{
		(*it)->print();
	}
}

CPartite * CGraph::getPartite(string name)
{
	vector<CPartite*>::iterator it;
	for(it=mPartites.begin();it!=mPartites.end();it++)
	{
		if(0==(*it)->mName.compare(name))
		{
			return (*it);
		}
	}

	return NULL;
}

CVertex * CGraph::getVertex(string name)
{
	vector<CVertex*>::iterator it;
	for(it=mVertices.begin();it!=mVertices.end();it++)
	{
		if(0==(*it)->mName.compare(name))
		{
			return (*it);
		}
	}

	return NULL;
}
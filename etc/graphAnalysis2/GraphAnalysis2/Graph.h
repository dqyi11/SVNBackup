#pragma once

#include <string>
#include <vector>

using namespace std;

class CVertex
{
public:
	CVertex(string name);
	~CVertex();

	void print();

	string mName;
	int mIndex;
};

class CEdge
{
public:
	CEdge(CVertex * a, CVertex * b);
	~CEdge();

	void print();

	CVertex * mpA;
	CVertex * mpB;
	int mIndex;
};

class CPartite
{
public:
	CPartite(string name);
	~CPartite();

	void add(CVertex * pVertex);

	void print();

	vector<CVertex*> mVertices;
	string mName;
};

class CConnector
{
public:
	CConnector(CPartite * a, CPartite * b);
	~CConnector();

	void add(CEdge* pEdge);

	void print();

	CPartite * mpA;
	CPartite * mpB;

	vector<CEdge*> mEdges;
};

class CGraph
{
public:
	CGraph(void);
	~CGraph(void);

	CVertex * createVertex(string name);
	CPartite * createPartite(string name);
	CEdge * connect(CVertex * a, CVertex * b);
	CConnector * connect(CPartite * a, CPartite * b);

	CPartite * getPartite(string name);
	CVertex * getVertex(string name);

	void printVertex();
	void printEdge();

	vector<CVertex*> mVertices;
	vector<CEdge*> mEdges;
	vector<CPartite*> mPartites;
	vector<CConnector*> mConnectors;
};

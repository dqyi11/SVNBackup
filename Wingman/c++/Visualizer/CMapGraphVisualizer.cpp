/*
 * CMapGraphVisualizer.cpp
 *
 *  Created on: Jan 8, 2013
 *      Author: walter
 */

#include "CMapGraphVisualizer.h"

#include <iostream>

using namespace std;

CVisualVertex::CVisualVertex()
{
	mpVertex = NULL;
	mpVisualHandle = NULL;
}

CVisualVertex::~CVisualVertex()
{
	mpVertex = NULL;
	mpVisualHandle = NULL;
}

CVisualEdge::CVisualEdge()
{
	mpEdge = NULL;
	mpVisualHandle = NULL;
}

CVisualEdge::~CVisualEdge()
{
	mpEdge = NULL;
	mpVisualHandle = NULL;
}

CMapGraphVisualizer::CMapGraphVisualizer(CHexaGridGraph * graph) {
	// TODO Auto-generated constructor stub
	mpGraph = graph;

	mVisualVertexList.clear();
	mVisualEdgeList.clear();
}

CMapGraphVisualizer::~CMapGraphVisualizer() {
	// TODO Auto-generated destructor stub
	list<CVisualVertex*>::iterator itV;
	for(itV=mVisualVertexList.begin();itV!=mVisualVertexList.end();itV++)
	{
		CVisualVertex * pV = (*itV);
		delete pV;
		(*itV)=NULL;
	}

	list<CVisualEdge*>::iterator itE;
	for(itE=mVisualEdgeList.begin();itE!=mVisualEdgeList.end();itE++)
	{
		CVisualEdge * pE = (*itE);
		delete pE;
		(*itE)=NULL;
	}

	mVisualVertexList.clear();
	mVisualEdgeList.clear();

	igraph_destroy(mpIGraph);
}

void CMapGraphVisualizer::draw()
{
	//dataPreparation();

	if(mpGraph)
	{
		//igraph_matrix_t res;
		//igraph_layout_grid(mpIGraph, &res, 0);

		cout << " igraph layout grid " <<endl;

		GVC_t * gvc = NULL;
		Agraph_t * g = NULL;
		Agnode_t * n = NULL, * m = NULL;
		Agedge_t * e = NULL;

		gvc = gvContext();

		if(gvc==NULL)
		{
			cout << " failed " << endl;
		}


		cout << "gvContext "<<endl;

		g = agopen("g", AGRAPH);

		list<CVertex *>::iterator itV;
		for(itV=mpGraph->mVertexList.begin();itV!=mpGraph->mVertexList.end();itV++)
		{
			CVisualVertex * visualVertex = new CVisualVertex();
			visualVertex->mpVertex = (CHexaVertex *)(*itV);
			char name[256];
			memset(name,0x0,256);
			strcpy(name, (*itV)->mName.c_str());
			//printf(name, "%d %d", visualVertex->mpVertex->mPosX, visualVertex->mpVertex->mPosY);
			// cout << " make " << name << endl;
			visualVertex->mpVisualHandle = agnode(g, name);

			int ret = -1;

			char rank[25];
			memset(rank,0x0,25);
			sprintf(rank, "%d", visualVertex->mpVertex->mPosY);
			agsafeset(visualVertex->mpVisualHandle, (char*)"shape", (char*)"polygon", "");
			agsafeset(visualVertex->mpVisualHandle, "sides", "6", "");
			agsafeset(visualVertex->mpVisualHandle, "group", rank, "");
			agsafeset(visualVertex->mpVisualHandle, "height", "0.5", "");
			agsafeset(visualVertex->mpVisualHandle, "width", "0.5", "");
			agsafeset(visualVertex->mpVisualHandle, "fixedsize", "true", "");

			mVisualVertexList.push_back(visualVertex);
		}

		list<CEdge *>::iterator itE;
		for(itE=mpGraph->mEdgeList.begin();itE!=mpGraph->mEdgeList.end();itE++)
		{
			CVisualEdge * visualEdge = new CVisualEdge();
			visualEdge->mpEdge = (*itE);
			CVisualVertex * m = findVisualVertex((CHexaVertex*)visualEdge->mpEdge->mpVertexA);
		    CVisualVertex * n = findVisualVertex((CHexaVertex*)visualEdge->mpEdge->mpVertexB);
		    if(m->mpVisualHandle!=NULL && n->mpVisualHandle!=NULL)
		    {
				visualEdge->mpVisualHandle = agedge(g, m->mpVisualHandle, n->mpVisualHandle);
				mVisualEdgeList.push_back(visualEdge);
		    }
		}


		gvLayout(gvc, g, "dot");

		cout << "out put ... " << endl;
		gvRenderFilename(gvc, g, "png", "graph.png");
		gvFreeLayout(gvc, g);
		agclose(g);

		gvFreeContext(gvc);
	}

}

void CMapGraphVisualizer::dataPreparation()
{
	if(mpGraph)
	{
		igraph_integer_t vertexNum = mpGraph->mVertexList.size();
		igraph_empty(mpIGraph, vertexNum, IGRAPH_UNDIRECTED);

		list<CEdge *>::iterator it;
		for(it=mpGraph->mEdgeList.begin();it!=mpGraph->mEdgeList.end();it++)
		{
			CEdge * edge = (*it);
			CHexaVertex * vertexA = (CHexaVertex *)(edge->mpVertexA);
			CHexaVertex * vertexB = (CHexaVertex *)(edge->mpVertexB);

			igraph_integer_t vertexAId = vertexA->mIndex;
			igraph_integer_t vertexBId = vertexB->mIndex;

			igraph_add_edge(mpIGraph, vertexAId, vertexBId);
		}

		//cout << " edge " << igraph_ecount(mpIGraph) << endl;
		//cout << " vertex " << igraph_vcount(mpIGraph) << endl;

	}

}

CVisualVertex * CMapGraphVisualizer::findVisualVertex(CHexaVertex * vertex)
{
	list<CVisualVertex*>::iterator it;
	for(it=mVisualVertexList.begin();it!=mVisualVertexList.end();it++)
	{

		if((*it)->mpVertex==vertex)
		{
			return (*it);
		}
	}

	return NULL;
}

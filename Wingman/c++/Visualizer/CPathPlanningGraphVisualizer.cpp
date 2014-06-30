/*
 * CPathPlanningGraphVisualizer.cpp
 *
 *  Created on: Jan 11, 2013
 *      Author: walter
 */

#include "CPathPlanningGraphVisualizer.h"
#include <iostream>

using namespace std;

CVisualLevelVertex::CVisualLevelVertex()
{
	mpVertex = NULL;
	mpVisualHandle = NULL;
}

CVisualLevelVertex::~CVisualLevelVertex()
{
	mpVertex = NULL;
	mpVisualHandle = NULL;
}

CVisualDirectedEdge::CVisualDirectedEdge()
{
	mpEdge = NULL;
	mpVisualHandle = NULL;
}

CVisualDirectedEdge::~CVisualDirectedEdge()
{
	mpEdge = NULL;
	mpVisualHandle = NULL;
}

CPathPlanningGraphVisualizer::CPathPlanningGraphVisualizer(CPathPlanningGraph * graph) {
	// TODO Auto-generated constructor stub
	mpGraph = graph;
	mVisualVertexList.clear();
	mVisualEdgeList.clear();

}

CPathPlanningGraphVisualizer::~CPathPlanningGraphVisualizer() {
	// TODO Auto-generated destructor stub
	mpGraph = NULL;
	mVisualVertexList.clear();
	mVisualEdgeList.clear();
}

CVisualLevelVertex * CPathPlanningGraphVisualizer::findVisualLevelVertex(CLevelVertex * vertex)
{
	if(vertex)
	{
		list<CVisualLevelVertex*>::iterator it;
		for(it=mVisualVertexList.begin();it!=mVisualVertexList.end();it++)
		{
			if((*it)->mpVertex==vertex)
			{
				return (*it);
			}
		}
	}

	return NULL;
}

void CPathPlanningGraphVisualizer::draw(const char * filename)
{
	if(mpGraph)
	{
		GVC_t * gvc = NULL;
		Agraph_t * g = NULL;
		Agnode_t * n = NULL, * m = NULL;
		Agedge_t * e = NULL;

		gvc = gvContext();

		if(gvc==NULL)
		{
			cout << " failed " << endl;
		}

		g = agopen("g", AGDIGRAPH);

		int levelLength = mpGraph->getPlanningLength();

		for(int i=0;i<levelLength;i++)
		{
			CLevelVertexSet set = mpGraph->mpLevelSets[i];

			//cout << " plotting level " << i << ", size " << set.mSet.size() << endl;

			list<CLevelVertex*>::iterator it;
			for(it=set.mSet.begin(); it!=set.mSet.end();it++)
			{
				//cout << " on level " << i << " at " << (*it)->mName << endl;
				CVisualLevelVertex * visualVertex = new CVisualLevelVertex();
				visualVertex->mpVertex = (*it);
				char name[256];
				memset(name,0x0,256);
				sprintf(name,"%s@%d",(*it)->mName.c_str(), (*it)->mLevel);

				visualVertex->mpVisualHandle = agnode(g, name);

				char rank[25];
				memset(rank,0x0,25);
				sprintf(rank, "%d", visualVertex->mpVertex->mLevel);

				//cout << " plot " << name << " - " << rank << endl;

				agsafeset(visualVertex->mpVisualHandle, (char*)"shape", (char*)"polygon", "");
				agsafeset(visualVertex->mpVisualHandle, "sides", "6", "");
				agsafeset(visualVertex->mpVisualHandle, "group", rank, "");
				agsafeset(visualVertex->mpVisualHandle, "height", "1", "");
				agsafeset(visualVertex->mpVisualHandle, "width", "1", "");
				agsafeset(visualVertex->mpVisualHandle, "fixedsize", "true", "");

				mVisualVertexList.push_back(visualVertex);

			}

		}

		list<CEdge *>::iterator itE;
		for(itE=mpGraph->mEdgeList.begin();itE!=mpGraph->mEdgeList.end();itE++)
		{
			//cout << " find edge " << endl;
			CVisualDirectedEdge * visualEdge = new CVisualDirectedEdge();
			visualEdge->mpEdge = (CDirectedEdge*)(*itE);

			CVisualLevelVertex * m = findVisualLevelVertex((CLevelVertex*)visualEdge->mpEdge->mpVertexA);
			CVisualLevelVertex * n = findVisualLevelVertex((CLevelVertex*)visualEdge->mpEdge->mpVertexB);

		    //cout << " edge for " << m->mpVertex->mName << " and " << n->mpVertex->mName << endl;

			if(m->mpVisualHandle!=NULL && n->mpVisualHandle!=NULL)
			{
				visualEdge->mpVisualHandle = agedge(g, m->mpVisualHandle, n->mpVisualHandle);
				mVisualEdgeList.push_back(visualEdge);
			}
		}

		gvLayout(gvc, g, "dot");

		// cout << "out put ... " << endl;
		gvRenderFilename(gvc, g, "png", filename);
		gvFreeLayout(gvc, g);
		agclose(g);

		gvFreeContext(gvc);
	}

}


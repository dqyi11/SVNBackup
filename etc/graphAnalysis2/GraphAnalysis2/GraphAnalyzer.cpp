#include "StdAfx.h"
#include "GraphAnalyzer.h"
#include <iostream>

using namespace std;

CGraphAnalyzer::CGraphAnalyzer(CGraph * graph)
{
	mpGraph = graph;
}

CGraphAnalyzer::~CGraphAnalyzer(void)
{
	mpGraph = NULL;
	igraph_destroy(&mGraph);
}

void CGraphAnalyzer::init()
{
	if(mpGraph)
	{
		igraph_integer_t v_num = mpGraph->mVertices.size();
		cout << " edge number " << mpGraph->mEdges.size() << endl;
		igraph_empty(&mGraph, v_num, IGRAPH_UNDIRECTED);

		vector<CVertex*>::iterator itV ;
		for(itV=mpGraph->mVertices.begin();itV!=mpGraph->mVertices.end();itV++)
		{
			igraph_integer_t v = (*itV)->mIndex;
			// cout << "igraph_add_vertex: " << igraph_add_vertex(&mGraph, v, NULL) << endl;

		}

		vector<CEdge*>::iterator itE;
		for(itE=mpGraph->mEdges.begin();itE!=mpGraph->mEdges.end();itE++)
		{
			igraph_integer_t from = (*itE)->mpA->mIndex;
			igraph_integer_t to = (*itE)->mpB->mIndex;
			cout << "igraph_add_edge: " << igraph_add_edge(&mGraph, from, to) << endl;
			

			// cout << "igraph_edge: " << igraph_edge(&mGraph,(*it)->mIndex,&((*it)->mpA->mIndex),&((*it)->mpB->mIndex)) << endl;

		}

		igraph_integer_t eCnt = igraph_ecount(&mGraph);
		igraph_integer_t vCnt = igraph_vcount(&mGraph);


		cout << "eCnt " << eCnt << ", vCnt " << vCnt << endl;

		igraph_bool_t res;
		igraph_is_bipartite(&mGraph, &res, NULL);
		cout << "igraph_is_bipartite " << res << endl;

		igraph_integer_t diameter;
		igraph_bool_t unconn = true;
		igraph_diameter(&mGraph, &diameter, NULL, NULL, NULL,IGRAPH_UNDIRECTED, unconn);

		cout << "igraph_diameter: " << diameter << endl;

	}

}
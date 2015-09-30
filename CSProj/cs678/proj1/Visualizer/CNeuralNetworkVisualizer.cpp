/*
 * CNeuralNetworkVisualizer.cpp
 *
 *  Created on: Feb 4, 2013
 *      Author: walter
 */

#include "CNeuralNetworkVisualizer.h"
#include <iostream>

using namespace std;

CVisualVertex::CVisualVertex()
{
	mpVertex = NULL;
	mpLayer = NULL;
	mpVisualHandle = NULL;
}

CVisualVertex::~CVisualVertex()
{
	mpVertex = NULL;
	mpLayer = NULL;
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

CNeuralNetworkVisualizer::CNeuralNetworkVisualizer(CNeuralNetwork * network) {
	// TODO Auto-generated constructor stub
	mpNetwork = network;
	mVertices.clear();
	mEdges.clear();
}

CNeuralNetworkVisualizer::~CNeuralNetworkVisualizer() {
	// TODO Auto-generated destructor stub
	mpNetwork = NULL;
	mVertices.clear();
	mEdges.clear();
}

void CNeuralNetworkVisualizer::draw(char * filename) {

	if(mpNetwork)
	{
		cout << "CNeuralNetworkVisualizer::draw" << endl;
		GVC_t * gvc = NULL;
		Agraph_t * g = NULL;
		Agnode_t * n = NULL, * m = NULL;
		Agedge_t * e = NULL;

		gvc = gvContext();

		if(gvc==NULL)
		{
			cout << " gvnContext init failed " << endl;
		}

		g = agopen("g", AGDIGRAPH);

		for(int i=0;i<mpNetwork->getLayerNum();i++)
		{
			CLayer * pLayer = mpNetwork->getLayer(i);

			cout << "in Layer " << pLayer->getName() << endl;

			for(int j=0;j<pLayer->getNeuronNum();j++)
			{
				CNeuron * pNeuron = pLayer->getNeuron(j);

				cout << "in Neuron " << pNeuron->getName() << endl;
				CVisualVertex * pVisualVertex = new CVisualVertex();
				pVisualVertex->mpLayer = pLayer;
				pVisualVertex->mpVertex = pNeuron;

				char name[256];
				memset(name,0x0,256);
				sprintf(name,"%s",pNeuron->getName().c_str());

				pVisualVertex->mpVisualHandle = agnode(g, name);

				char rank[25];
				memset(rank,0x0,25);
				sprintf(rank, "%s", pLayer->getName().c_str());

				//cout << " plot " << name << " - " << rank << endl;

				switch(pLayer->getType())
				{
				case INPUT:
				default:
					agsafeset(pVisualVertex->mpVisualHandle, "shape", "square", "");
					break;
				case HIDDEN:
					agsafeset(pVisualVertex->mpVisualHandle, "shape", "circle", "");
					break;
				case OUTPUT:
					agsafeset(pVisualVertex->mpVisualHandle, "shape", "polygon", "");
					break;
				}

				agsafeset(pVisualVertex->mpVisualHandle, "sides", "6", "");
				agsafeset(pVisualVertex->mpVisualHandle, "group", rank, "");
				agsafeset(pVisualVertex->mpVisualHandle, "height", "1", "");
				agsafeset(pVisualVertex->mpVisualHandle, "width", "1", "");
				agsafeset(pVisualVertex->mpVisualHandle, "fixedsize", "true", "");

				mVertices.push_back(*pVisualVertex);

			}

		}

		for(int k=0;k<mpNetwork->getLinkNum();k++)
		{
			CNeuronLink * pLink = mpNetwork->getLink(k);

			CVisualEdge * pVisualEdge = new CVisualEdge();
			pVisualEdge->mpEdge = pLink;

			CVisualVertex * pFrom = findVisualVertex(pLink->mpFromNeuron);
			CVisualVertex * pTo = findVisualVertex(pLink->mpToNeuron);

			if(pFrom!=NULL && pTo!=NULL)
			{

				if(pFrom->mpVisualHandle!=NULL && pFrom->mpVisualHandle!=NULL)
				{
					pVisualEdge->mpVisualHandle = agedge(g, pFrom->mpVisualHandle, pTo->mpVisualHandle);
					mEdges.push_back(*pVisualEdge);
				}

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

CVisualVertex * CNeuralNetworkVisualizer::findVisualVertex(CNeuron * pNeuron)
{
	vector<CVisualVertex>::iterator it;
	for(it=mVertices.begin();it!=mVertices.end();it++)
	{
		if((*it).mpVertex==pNeuron)
		{
			return &(*it);
		}
	}

	return NULL;
}


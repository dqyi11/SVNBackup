/*
 * CMapGraphManager.cpp
 *
 *  Created on: Jan 7, 2013
 *      Author: walter
 */

#include "CMapGraphConvertor.h"
#include <sstream>
#include <iostream>

using namespace std;

CMapGraphConvertor::CMapGraphConvertor() {
	// TODO Auto-generated constructor stub
	mpMap = NULL;
	mpGraph = NULL;
	mpVisualizer = NULL;
}

CMapGraphConvertor::~CMapGraphConvertor() {
	// TODO Auto-generated destructor stub
	if(mpVisualizer)
	{
		delete mpVisualizer;
		mpVisualizer = NULL;
	}

	if(mpGraph)
	{
		delete mpGraph;
		mpGraph = NULL;
	}

	mpMap = NULL;
}

bool CMapGraphConvertor::init()
{
	if(mpMap==NULL)
	{
		return false;
	}

	if(mpVisualizer)
	{
		delete mpVisualizer;
		mpVisualizer = NULL;
	}

	if(mpGraph)
	{
		delete mpGraph;
		mpGraph = NULL;
	}

	mpGraph = new CHexaGridGraph();

	int width = 2 * mpMap->getWidth();
	int height = mpMap->getHeight();
	stringstream sstream;

	//cout << " MAP " << width << "X" << height << endl;

    for(int j=1;j<=height;j+=2)
    {
    	for(int i=1;i<=width;i+=2)
    	{
    		string name1 = "";
			string name2 = "";

    		CVisualHexagon * pGrid = &(mpMap->mpVisualHexagons[i-1][j-1]);
    		sstream << pGrid->getIndex();

    		sstream >> name1;
    		sstream.clear();



            CHexaVertex * pVertex1 = mpGraph->createVertex(name1);
            pVertex1->mPosX = pGrid->getGrid()->mX;
            pVertex1->mPosY = pGrid->getGrid()->mY;
            pVertex1->mIndex = pGrid->getIndex();

            // cout << "createing " << name1 << " + " << pVertex1->mName << endl;


            pGrid = &(mpMap->mpVisualHexagons[i][j]);

            sstream << pGrid->getIndex();
            sstream >> name2;

            sstream.clear();

    		CHexaVertex * pVertex2 = mpGraph->createVertex(name2);
    		pVertex2->mPosX = pGrid->getGrid()->mX;
    		pVertex2->mPosY = pGrid->getGrid()->mY;
    		pVertex2->mIndex = pGrid->getIndex();

    		// cout << "creating " << name2 << " + " << pVertex2->mName << endl;

    	}
    }

    //mpGraph->printVertex();

    // build edge by map connection for each vertex
    list<CVertex *>::iterator itV;
    for(itV=mpGraph->mVertexList.begin();itV!=mpGraph->mVertexList.end();itV++)
    {
    	CHexaVertex * hexaVex = (CHexaVertex *)(*itV);

    	//cout << " checking " << hexaVex->mName << " " << hexaVex->mPosX << " " << hexaVex->mPosY << endl;
    	CGridSet set = mpMap->getGridSet(hexaVex->mPosX, hexaVex->mPosY, 1);
    	list<CGrid*>::iterator itG;

    	//cout << " find neighbor " << " ";
    	for(itG=set.mSet.begin();itG!=set.mSet.end();itG++)
    	{
    		// find corresponding vertex
    		CHexaVertex * neiHexaVex = mpGraph->findVertex((*itG)->mX, (*itG)->mY);

    		//if(neiHexaVex)
    		//   cout << neiHexaVex->mName << " ";
    		// make the connection
    		if(hexaVex!=neiHexaVex)
    		{
    			mpGraph->addEdge((CVertex*)hexaVex, (CVertex*)neiHexaVex);
    		}
    	}
    	//cout << endl;

    }

    //mpGraph->printEdge();

    mpVisualizer = new CMapGraphVisualizer(mpGraph);

}

void CMapGraphConvertor::visualizeGraph()
{
	if(mpVisualizer)
	{
		mpVisualizer->draw();
	}
}

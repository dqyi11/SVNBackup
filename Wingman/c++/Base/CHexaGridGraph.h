/*
 * CHexaGridGraph.h
 *
 *  Created on: Jan 8, 2013
 *      Author: walter
 */

#ifndef CHEXAGRIDGRAPH_H_
#define CHEXAGRIDGRAPH_H_

#include "CGraph.h"

class CHexaVertex : public CVertex {
public:
	CHexaVertex();
	virtual ~CHexaVertex();

	int mPosX;
	int mPosY;
	int mIndex;
};

class CHexaGridGraph: public CGraph {
public:
	CHexaGridGraph();
	virtual ~CHexaGridGraph();

	CHexaVertex * createVertex(string name);

	CHexaVertex * findVertex(int posX, int posY);
	CHexaVertex * findVertex(string name);

	bool isConnected(CHexaVertex * a, CHexaVertex * b);

};

#endif /* CHEXAGRIDGRAPH_H_ */

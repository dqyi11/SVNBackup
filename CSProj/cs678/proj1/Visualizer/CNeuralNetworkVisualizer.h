/*
 * CNeuralNetworkVisualizer.h
 *
 *  Created on: Feb 4, 2013
 *      Author: walter
 */

#ifndef CNEURALNETWORKVISUALIZER_H_
#define CNEURALNETWORKVISUALIZER_H_

#include <gvc.h>
#include "../NeuralNetwork/CNeuralNetwork.h"
#include <vector>

class CVisualVertex
{
public:
	CVisualVertex();
	virtual ~CVisualVertex();

	CNeuron * mpVertex;
	CLayer  * mpLayer;
	Agnode_t * mpVisualHandle;
};

class CVisualEdge
{
public:
	CVisualEdge();
	virtual ~CVisualEdge();

	CNeuronLink * mpEdge;
	Agedge_t    * mpVisualHandle;
};

class CNeuralNetworkVisualizer {
public:
	CNeuralNetworkVisualizer(CNeuralNetwork * network);
	virtual ~CNeuralNetworkVisualizer();

	//oid init();
	void draw(char * filename);

	CVisualVertex * findVisualVertex(CNeuron * pNeuron);


private:
	CNeuralNetwork * mpNetwork;

	vector<CVisualVertex> mVertices;
	vector<CVisualEdge>   mEdges;

};

#endif /* CNEURALNETWORKVISUALIZER_H_ */

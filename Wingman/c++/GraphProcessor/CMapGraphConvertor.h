/*
 * CMapGraphManager.h
 *
 *  Created on: Jan 7, 2013
 *      Author: walter
 */

#ifndef CMAPGRAPHCONVERTOR_H_
#define CMAPGRAPHCONVERTOR_H_

#include "../Base/CHexaGridGraph.h"
#include "../Visualizer/CMapGraphVisualizer.h"
#include "../Visualizer/CVisualHexagonDiscretizedMap.h"

class CMapGraphConvertor {
public:
	CMapGraphConvertor();
	virtual ~CMapGraphConvertor();

	bool init();

	void setMap(CVisualHexagonDiscretizedMap * pMap) { mpMap = pMap; };

	CHexaGridGraph * getGraph() { return mpGraph; };

	void visualizeGraph();

private:
	CVisualHexagonDiscretizedMap * mpMap;
	CHexaGridGraph * mpGraph;

	CMapGraphVisualizer * mpVisualizer;

};

#endif /* CMAPGRAPHCONVERTOR_H_ */

#pragma once

#include <igraph.h>
#include "Graph.h"

class CGraphAnalyzer
{
public:
	CGraphAnalyzer(CGraph * graph);
	~CGraphAnalyzer(void);

	void init();

	CGraph * mpGraph;
	igraph_t mGraph;
};

#pragma once

#include "tinyxml.h"
#include "Graph.h"

class CGraphLoader
{
public:
	CGraphLoader(const char * filename);
	~CGraphLoader(void);

	void init();

	TiXmlDocument * mpDoc;
	CGraph * mpGraph;
};

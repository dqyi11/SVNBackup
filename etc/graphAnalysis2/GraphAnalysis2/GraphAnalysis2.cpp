// GraphAnalysis2.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "GraphLoader.h"
#include "GraphAnalyzer.h"

#include <Windows.h>

int _tmain(int argc, _TCHAR* argv[])
{
    char* pFilename = NULL; 
	DWORD dwNum = WideCharToMultiByte(CP_OEMCP,NULL,argv[1],-1,NULL,0,NULL,FALSE);
	pFilename = new char[dwNum];
    WideCharToMultiByte (CP_OEMCP,NULL,argv[1],-1,pFilename,dwNum,NULL,FALSE);

	CGraphLoader * graphLoader = new CGraphLoader(pFilename);
	graphLoader->init();

	CGraphAnalyzer * analyzer =  new CGraphAnalyzer(graphLoader->mpGraph);
	analyzer->init();

	return 0;
}


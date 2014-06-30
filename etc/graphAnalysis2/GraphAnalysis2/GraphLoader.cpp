#include "StdAfx.h"
#include "GraphLoader.h"
#include <iostream>


using namespace std;

CGraphLoader::CGraphLoader(const char * filename)
{
	mpDoc = new TiXmlDocument(filename);
	mpGraph = new CGraph();

	if(false == mpDoc->LoadFile())
	{
		cout << "load file failure = " << filename << endl;
	}
}

CGraphLoader::~CGraphLoader(void)
{

	if(mpGraph)
	{
		delete mpGraph;
		mpGraph = NULL;
	}

	if(mpDoc)
	{
		delete mpDoc;
		mpDoc = NULL;
	}
}

void CGraphLoader::init()
{
	if(mpDoc)
	{
		TiXmlElement* pRoot = mpDoc->FirstChildElement("Graph");
		
		if(pRoot)
		{
			TiXmlElement* pPartiteElement = NULL;
			
			do
			{
				if(NULL==pPartiteElement)
				{
					pPartiteElement = pRoot->FirstChildElement("Partite");
				}
				else
				{
					pPartiteElement = pPartiteElement->NextSiblingElement("Partite");
				}

				if(pPartiteElement)
				{
					string parAttrName(pPartiteElement->Attribute("name"));
					CPartite * partite = mpGraph->createPartite(parAttrName);

					TiXmlElement * pVertexElement = NULL;
					
					do
					{
						if(NULL==pVertexElement)
						{
							pVertexElement = pPartiteElement->FirstChildElement("Vertex");
						}
						else
						{
							pVertexElement = pVertexElement->NextSiblingElement("Vertex");
						}

						if(pVertexElement)
						{
							string vertexAttrName(pVertexElement->Attribute("name"));
							CVertex * vertex = mpGraph->createVertex(vertexAttrName);
							partite->add(vertex);						

						}

					}while(NULL!=pVertexElement);

				}

			}while(NULL!=pPartiteElement);

			TiXmlElement* pConnectorElement = NULL;

			do
			{
				if(NULL==pConnectorElement)
				{
					pConnectorElement = pRoot->FirstChildElement("Connector");
				}
				else
				{
					pConnectorElement = pConnectorElement->NextSiblingElement("Connector");
				}

				if(pConnectorElement)
				{
					string conAttrName1(pConnectorElement->Attribute("partite1"));
					string conAttrName2(pConnectorElement->Attribute("partite2"));

					CPartite * p1 = mpGraph->getPartite(conAttrName1);
					CPartite * p2 = mpGraph->getPartite(conAttrName2);
					CConnector * connector = mpGraph->connect(p1,p2);

					TiXmlElement * pEdgeElement = NULL;
					
					do
					{
						if(NULL==pEdgeElement)
						{
							pEdgeElement = pConnectorElement->FirstChildElement("Edge");
						}
						else
						{
							pEdgeElement = pEdgeElement->NextSiblingElement("Edge");
						}

						if(pEdgeElement)
						{
							string edgeAttrName1(pEdgeElement->Attribute("node1"));
							string edgeAttrName2(pEdgeElement->Attribute("node2"));

							CVertex * v1 = mpGraph->getVertex(edgeAttrName1);
							CVertex * v2 = mpGraph->getVertex(edgeAttrName2);
							CEdge * edge = mpGraph->connect(v1, v2);
							connector->add(edge);						

						}

					}while(NULL!=pEdgeElement);

				}

			}while(NULL!=pConnectorElement);


		}

		mpGraph->printVertex();
		mpGraph->printEdge();
		

	}

}

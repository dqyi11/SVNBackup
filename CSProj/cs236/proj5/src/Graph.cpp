/*
 * Graph.cpp
 *
 *  Created on: Apr 4, 2013
 *      Author: walter
 */

#include "Graph.h"

using namespace std;

Graph::Graph() {
}
Graph::~Graph() {
}

void Graph::addNode(Node * node)
{
	if(node)
	{
		if(NULL==getNode(node->getName()))
		{
			mNodeList.push_back(node);
		}
	}

}

Node * Graph::getNode(std::string name)
{
	vector<Node*>::iterator it;
	for(it=mNodeList.begin();it!=mNodeList.end();it++)
	{
		if((*it)->getName().compare(name)==0)
		{
			return (Node*)(*it);
		}
	}

	return NULL;
}

void Graph::addNode(string name)
{
	if(NULL==getNode(name))
	{
		Node * newNode = new Node(name);
		addNode(newNode);
	}
}

bool Graph::connect(string from, string to)
{
	Node * pFrom = getNode(from);
	Node * pTo = getNode(to);

	if(NULL==pFrom || NULL==pTo)
	{
		return false;
	}

	pFrom->addAdjacent(pTo);

	return true;
}

void Graph::print()
{
	vector<Node*>::iterator it;
	for(it=mNodeList.begin();it!=mNodeList.end();it++)
	{
		(*it)->print();
	}
}

Graph Graph::getGraphByDFS(std::string rootName) {
	Graph result;

	result.addTree(getNode(rootName), 0);
	return result;
}

Node* Graph::addTree(Node* localRoot, int level){
	Node* copy = this->getVisited(localRoot->getName());
	if(NULL == copy){
		copy = new Node(localRoot->getName());
		copy->setLevel(level);
		this->mVisitedNodes.push_back(copy);
		for(vector<Node*>::iterator it=localRoot->mAdjacentList.begin(); it!=localRoot->mAdjacentList.end(); ++it){
			Node* child = this->addTree(*it, level+1);
			copy->addAdjacent(child);
		}

		this->addNode(copy);
	}

	return copy;
}

Node* Graph::getVisited(string name){
	vector<Node*>::iterator it;
	for(it=mVisitedNodes.begin();it!=mVisitedNodes.end();it++)
		if((*it)->getName().compare(name)==0)
			return *it;

	return NULL;
}

bool Graph::isCyclic()
{
	vector<Node*>::iterator it;
	for(it=mNodeList.begin();it!=mNodeList.end();it++)
	{
		vector<Node*>::iterator itNeighbor;
		for(itNeighbor=(*it)->mAdjacentList.begin();
				itNeighbor!=(*it)->mAdjacentList.end();
				itNeighbor++)
		{
			if(getPostOrder((*it)->getName())
					<=
				getPostOrder((*itNeighbor)->getName())
			  )
			{
				return true;
			}

		}
	}

	return false;
}

vector<ExecutionItem> Graph::generateExecutionItems()
{
	vector<ExecutionItem> executionItems;

	std::vector<Node*>::iterator it;
	for(it=mNodeList.begin();it!=mNodeList.end();it++)
	{

	}

	return executionItems;
}

int Graph::getPostOrder(std::string name)
{
	int postOrderNum = 1;

	vector<Node*>::iterator it;
	for(it=mNodeList.begin();it!=mNodeList.end();it++)
	{
		if((*it)->getName().compare(name)==0)
		{
			return postOrderNum;
		}
		postOrderNum++;
	}

    return postOrderNum;
}

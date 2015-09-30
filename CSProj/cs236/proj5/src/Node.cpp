/*
 * Node.cpp
 *
 *  Created on: Apr 4, 2013
 *      Author: walter
 */

#include "Node.h"
#include <iostream>

using namespace std;

Node::Node(string name) {
	// TODO Auto-generated constructor stub
	mName = name;
	mLevel = -1;
}

Node::~Node() {
	// TODO Auto-generated destructor stub
}

void Node::addAdjacent(Node * pNode)
{
	if(pNode)
	{
		if(false==isAdjacent(pNode))
		{
			mAdjacentList.push_back(pNode);
		}
	}
}

bool Node::isAdjacent(string name)
{
	vector<Node*>::iterator it;
	for(it=mAdjacentList.begin();it!=mAdjacentList.end();it++)
	{
		if((*it)->getName().compare(name)==0)
		{
			return true;
		}
	}

	return false;
}

bool Node::isAdjacent(Node * pNode)
{
	if(NULL==pNode)
	{
		return false;
	}

	return isAdjacent(pNode->getName());
}

void Node::print()
{
	cout << mName <<":";
	vector<Node*>::iterator it;
	for(it=mAdjacentList.begin();it!=mAdjacentList.end();it++)
	{
		cout <<(*it)->getName() <<",";
	}
	cout << endl;
}

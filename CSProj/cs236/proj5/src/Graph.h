/*
 * Graph.h
 *
 *  Created on: Apr 4, 2013
 *      Author: walter
 */

#ifndef GRAPH_H_
#define GRAPH_H_

#include "Node.h"
#include <string.h>
#include "ExecutionItem.h"

class Node;

class Graph {
public:
	Graph();
	virtual ~Graph();

	void addNode(std::string name);
	bool connect(std::string from, std::string to);
	void addNode(Node * node);
	Node * getNode(std::string name);

	// methods only called for DFS graph
	Graph getGraphByDFS(std::string rootName);
	Node* addTree(Node* localRoot, int level);
	Node* getVisited(std::string name);
	bool isCyclic();
	std::vector<ExecutionItem> generateExecutionItems();
	int getPostOrder(std::string name);

	void print();

	std::vector<Node*> mNodeList;

private:
	std::vector<Node*> mVisitedNodes;
};

#endif /* GRAPH_H_ */

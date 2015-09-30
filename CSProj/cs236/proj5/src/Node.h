/*
 * Node.h
 *
 *  Created on: Apr 4, 2013
 *      Author: walter
 */

#ifndef NODE_H_
#define NODE_H_
#include <vector>
#include <string>

class Node {
public:
	Node(std::string name);
	virtual ~Node();

	std::vector<Node*> mAdjacentList;

	std::string getName() { return mName; };

	void setLevel(int level) { mLevel = level; };
	int getLevel() { return mLevel; };

	void addAdjacent(Node * pNode);

	bool isAdjacent(std::string name);
	bool isAdjacent(Node * pNode);

	void print();

private:
	std::string mName;
	int mLevel;
};

#endif /* NODE_H_ */

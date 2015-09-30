/*
 * ExecutionItem.h
 *
 *  Created on: Apr 6, 2013
 *      Author: walter
 */

#ifndef EXECUTIONITEM_H_
#define EXECUTIONITEM_H_

#include "RelationRule.h"
#include <vector>

class ExecutionItem {
public:

	ExecutionItem();
	virtual ~ExecutionItem();

	RelationRule * mpRule;
	std::vector<RelationRule*> mRules;





};

#endif /* EXECUTIONITEM_H_ */

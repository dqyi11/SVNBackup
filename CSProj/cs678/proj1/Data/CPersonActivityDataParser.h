/*
 * CPersonActivityDataParser.h
 *
 *  Created on: Jan 30, 2013
 *      Author: walter
 */

#ifndef CPERSONACTIVITYDATAPARSER_H_
#define CPERSONACTIVITYDATAPARSER_H_

#include "CParser.h"
#include "CPersonActivityData.h"
#include "CPersonActivityDataManager.h"

class CPersonActivityDataParser: public CParser {
public:
	CPersonActivityDataParser(const char* fileName);
	virtual ~CPersonActivityDataParser();

	virtual bool initData(vector<string> elementSet);

	CPersonActivitySequence * getSequence(string seqNum);

	void print();

	bool initDataManager(CPersonActivityDataManager * pManager);

	vector<CPersonActivitySequence*> mSeqArray;
};

#endif /* CPERSONACTIVITYDATAPARSER_H_ */

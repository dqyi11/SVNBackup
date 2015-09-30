/*
 * CRandomNumGenerator.h
 *
 *  Created on: Feb 4, 2013
 *      Author: walter
 */

#ifndef CRANDOMNUMGENERATOR_H_
#define CRANDOMNUMGENERATOR_H_

class CRandomNumGenerator {
public:
	CRandomNumGenerator();
	virtual ~CRandomNumGenerator();

	int getRndNum();

	void setMax(int max) { mMax = max; };
	void setMin(int min) { mMin = min; };

	int getMax() { return mMax; };
	int getMin() { return mMin; };

private:
	int mMax;
	int mMin;
};

#endif /* CRANDOMNUMGENERATOR_H_ */

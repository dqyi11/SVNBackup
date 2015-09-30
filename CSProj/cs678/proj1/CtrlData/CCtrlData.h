/*
 * CCtrlData.h
 *
 *  Created on: Feb 6, 2013
 *      Author: walter
 */

#ifndef CCTRLDATA_H_
#define CCTRLDATA_H_

class CCtrlData {
public:
	CCtrlData();
	virtual ~CCtrlData();

	double getData(int type, int row, int col);
};

#endif /* CCTRLDATA_H_ */

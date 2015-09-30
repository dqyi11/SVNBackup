/*
 * CObstacle.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: walter
 */

#include "CObstacle.h"

CObstacle::CObstacle() {
	// TODO Auto-generated constructor stub

}

CObstacle::~CObstacle() {
	// TODO Auto-generated destructor stub
}

void CObstacle::print()
{
	vector<double>::iterator it;

	cout<<"The obstacle state is : " << endl;
	cout<<"X pos is : ";
	for (it = xvals.begin(); it != xvals.end(); it ++)
	{
		cout << (*it) << " ";
	}

	cout<<"Y pos is : ";
	for (it = yvals.begin(); it != yvals.end(); it ++)
	{
		cout << (*it) << " ";
	}
	cout << endl;
}

double CObstacle::getMidPointX(){
	vector<double>::iterator it;
	double midX = 0.0;

	assert(xvals.size() > 0);

	for (it = xvals.begin(); it != xvals.end(); it ++){
		midX += *it;
	}

	return midX/xvals.size();

}

double CObstacle::getMidPointY(){
	vector<double>::iterator it;
	double midY = 0.0;

	assert(yvals.size() > 0);

	for (it = yvals.begin(); it != yvals.end(); it ++){
		midY += *it;
	}

	return midY/yvals.size();

}

double CObstacle::getRadius(){
	double midX = getMidPointX();
	double midY = getMidPointY();

	double cornerX = xvals[0];
	double cornerY = yvals[0];

	return sqrt(pow(cornerX - midX, 2)+pow(cornerY - midY,2));

}


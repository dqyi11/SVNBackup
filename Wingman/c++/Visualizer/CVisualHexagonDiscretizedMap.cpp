/*
 * CVisualHexagonDiscretizedMap.cpp
 *
 *  Created on: Dec 5, 2012
 *      Author: walter
 */

#include "CVisualHexagonDiscretizedMap.h"
#include "gnuplot_i.hpp"
#include <cmath>
#include <string.h>

//const int D = 2;
#define MAX_LEN 512

string convertToHex(int value)
{
	string hex = "";
	char temp[2];
	memset(temp,0x0,2);

	if(value<10 && value>=0)
	{
		sprintf(temp, "%d", value);
		hex.append(temp);
	}
	else if(value>=10 && value<=15)
	{
		switch(value)
		{
		case 10:
			hex = "a";
			break;
		case 11:
			hex = "b";
			break;
		case 12:
			hex = "c";
			break;
		case 13:
			hex = "d";
			break;
		case 14:
			hex = "e";
			break;
		case 15:
			hex = "f";
			break;
		}
	}

	return hex;
}

string convertToColorValue(int value)
{
	string hex = "";
	if(value>255 && value<0)
	{
		return hex;
	}

	int bit1 = value % 16;
	int bit2 = value / 16;

	hex = convertToHex(bit2);
	hex.append(convertToHex(bit1));

	return "#"+hex+hex+hex;
}

CVisualHexagon::CVisualHexagon()
{
	mIndexId = -1;
	mCenterX = 0.0;
	mCenterY = 0.0;
	mValue = 0.0;

	for(int i=0;i<7;i++)
	{
		mXVertex[i]=0.0;
		mYVertex[i]=0.0;
	}

}

CVisualHexagon::~CVisualHexagon()
{

}

void CVisualHexagon::init(int X, int Y, int d)
{
	double tX = (double)X;
	double tY = (double)Y;
	double td = (double)d;
	if(X%2 ==0 && Y%2==1)
	{
		cout << " generate hexagon error " << X << " and " << Y << endl;
	}

	double s = 2/sqrt(3)*td;
	//cout << " visual hexagon " << X << " " << Y;

	mCenterX = X*td;

	if(Y%2==0)
	{
		mCenterY = tY/2*td+tY*s;
	}
	else
	{
		mCenterY = tY*s+tY*d/2;
	}


	//cout << " " << mCenterX << " " << mCenterY << endl;

	mXVertex[0]=mCenterX-d;
	mYVertex[0]=mCenterY-s/2;
	mXVertex[1]=mCenterX-d;
	mYVertex[1]=mCenterY+s/2;
	mXVertex[2]=mCenterX;
	mYVertex[2]=mCenterY+s;
	mXVertex[3]=mCenterX+d;
	mYVertex[3]=mCenterY+s/2;
	mXVertex[4]=mCenterX+d;
	mYVertex[4]=mCenterY-s/2;
	mXVertex[5]=mCenterX;
	mYVertex[5]=mCenterY-s;
	mXVertex[6]=mCenterX-d;
	mYVertex[6]=mCenterY-s/2;

	/*
	cout << "0 " << mXVertex[0] << " " << mYVertex[0] <<endl;
	cout << "1 " << mXVertex[1] << " " << mYVertex[1] <<endl;
	cout << "2 " << mXVertex[2] << " " << mYVertex[2] <<endl;
	cout << "3 " << mXVertex[3] << " " << mYVertex[3] <<endl;
	cout << "4 " << mXVertex[4] << " " << mYVertex[4] <<endl;
	cout << "5 " << mXVertex[5] << " " << mYVertex[5] <<endl;
	cout << "6 " << mXVertex[6] << " " << mYVertex[6] <<endl;
	*/

}

string CVisualHexagon::getRGBColor()
{
	string rgb = "#ffffff";
	int scaledValue = 255 * (1-mValue);


	rgb = convertToColorValue(scaledValue);

	//cout << " get color " << rgb << " from " << scaledValue << endl;

	return rgb;
}

/*
CVisualHexagonDiscretizedMap::CVisualHexagonDiscretizedMap()
    : CHexagonDiscretizedMap(0, 0, 0)
{

}
*/


CVisualHexagonDiscretizedMap::CVisualHexagonDiscretizedMap(int width, int height, int hexSize)
    : CHexagonDiscretizedMap(width, height, hexSize)
{
	// TODO Auto-generated constructor stub
	mpVisualHexagons = new CVisualHexagon * [mWidth];
	for(int i=0; i<mWidth; i++){
		mpVisualHexagons[i] = new CVisualHexagon[mHeight];
	}

}

CVisualHexagonDiscretizedMap::~CVisualHexagonDiscretizedMap() {
	// TODO Auto-generated destructor stub

}

CVisualHexagonDiscretizedMap::CVisualHexagonDiscretizedMap(const CVisualHexagonDiscretizedMap & map)
{
	mHeight = map.mHeight;
	mWidth = map.mWidth;
	mHexagonSize = map.mHexagonSize;
	mAgents = map.mAgents;

	for(int j=0;j<mHeight;j++)
	{
		for(int i=0;i<mWidth;i++)
		{
			mpGridLattice[i][j] = map.mpGridLattice[i][j];
			mpVisualHexagons[i][j] = map.mpVisualHexagons[i][j];
			mpVisualHexagons[i][j].setGrid(&mpGridLattice[i][j]);
		}
	}
}

void CVisualHexagonDiscretizedMap::init()
{
	CHexagonDiscretizedMap::init();

	int count = 1;

    for(int j=1;j<=mHeight;j+=2)
    {
    	for(int i=1;i<=mWidth;i+=2)
    	{
    		//cout << " init pos " << i-1 << " " << j-1 << endl;
    		mpVisualHexagons[i-1][j-1].init(i-1,j-1,mHexagonSize);
    		mpVisualHexagons[i-1][j-1].setIndex(count);
    		mpVisualHexagons[i-1][j-1].setValue(getEntropy(mpGridLattice[i-1][j-1].mProbabilityValue));
    		mpVisualHexagons[i-1][j-1].setGrid(&mpGridLattice[i-1][j-1]);
    		count=count+1;
    		//cout << " Index " << mpVisualHexagons[i-1][j-1].getIndex() << endl;

    		//cout << " init pos " << i << " " << j << endl;
    		mpVisualHexagons[i][j].init(i,j,mHexagonSize);
    		mpVisualHexagons[i][j].setIndex(count);
    		mpVisualHexagons[i][j].setValue(getEntropy(mpGridLattice[i][j].mProbabilityValue));
    		mpVisualHexagons[i][j].setGrid(&mpGridLattice[i][j]);
    		count=count+1;

    		//cout << " Index " << mpVisualHexagons[i][j].getIndex() << endl;

    	}
    }


}

void CVisualHexagonDiscretizedMap::syncValue()
{
    for(int j=1;j<=mHeight;j+=2)
    {
    	for(int i=1;i<=mWidth;i+=2)
    	{
    		mpVisualHexagons[i-1][j-1].setValue(getEntropy(mpGridLattice[i-1][j-1].mProbabilityValue));

    		mpVisualHexagons[i][j].setValue(getEntropy(mpGridLattice[i][j].mProbabilityValue));

    	}
    }
}

double CVisualHexagonDiscretizedMap::getEntropy(double probability)
{
	double entropy = 0;
	if(0==probability)
	{
		entropy = 0;
	}
	else if(1==probability)
	{
		entropy = 0;
	}
	else
	{
		entropy = probability * log2(probability) + (1-probability) * log2(1-probability);
		entropy = - entropy;
	}

	return entropy;
}

void CVisualHexagonDiscretizedMap::plotMap()
{
	syncValue();

	Gnuplot mDiscretizedMap;

	int width = mWidth * 20;
	int height = 2 *mHeight * 20;

	char temp[MAX_LEN];
	memset(temp,0x0,MAX_LEN);
	sprintf(temp, "set term png size %d,%d crop\n", width, width);

    mDiscretizedMap << temp;
	mDiscretizedMap << "set term png font ',12' linewidth 1\n";
    mDiscretizedMap.set_xrange(-2*mHexagonSize ,(mWidth+2) * mHexagonSize);
    mDiscretizedMap.set_yrange(-2*mHexagonSize ,2*(mHeight) * mHexagonSize);
    mDiscretizedMap << "set output 'tessellation.png' \n";

    int number_count = 0;

    for(int j=1;j<=mHeight;j+=2)
	{
		for(int i=1;i<=mWidth;i+=2)
		{
			//cout << "plotting " << i-1 << " " << j-1 << endl;
			//cout << "Index " << mpVisualHexagons[i-1][j-1].getIndex() << endl;
			number_count ++;

			memset(temp,0x0,MAX_LEN);
			sprintf(temp, "set object %d polygon from %f,%f to %f,%f to %f,%f to %f,%f to %f,%f to %f,%f to %f,%f\n",
					mpVisualHexagons[i-1][j-1].getIndex(),
					mpVisualHexagons[i-1][j-1].getXVertex(0),
					mpVisualHexagons[i-1][j-1].getYVertex(0),
					mpVisualHexagons[i-1][j-1].getXVertex(1),
					mpVisualHexagons[i-1][j-1].getYVertex(1),
					mpVisualHexagons[i-1][j-1].getXVertex(2),
					mpVisualHexagons[i-1][j-1].getYVertex(2),
					mpVisualHexagons[i-1][j-1].getXVertex(3),
					mpVisualHexagons[i-1][j-1].getYVertex(3),
					mpVisualHexagons[i-1][j-1].getXVertex(4),
					mpVisualHexagons[i-1][j-1].getYVertex(4),
					mpVisualHexagons[i-1][j-1].getXVertex(5),
					mpVisualHexagons[i-1][j-1].getYVertex(5),
					mpVisualHexagons[i-1][j-1].getXVertex(6),
					mpVisualHexagons[i-1][j-1].getYVertex(6));
			mDiscretizedMap << temp;

			memset(temp,0x0,MAX_LEN);
			if(mpVisualHexagons[i-1][j-1].getGrid()->mType==OBSTACLE)
			{
				sprintf(temp, "set object %d fillcolor rgb '#FF0000' fillstyle solid 1.0 border lt 1\n",
										mpVisualHexagons[i-1][j-1].getIndex());
			}
			else
			{
				sprintf(temp, "set object %d fillcolor rgb '%s' fillstyle solid 1.0 border lt 1\n",
						mpVisualHexagons[i-1][j-1].getIndex(),
						mpVisualHexagons[i-1][j-1].getRGBColor().c_str());
			}
			mDiscretizedMap << temp;
			//cout << temp;

			memset(temp,0x0,MAX_LEN);
			sprintf(temp, "set label %d at %f,%f ' %d ' front center font ',5' textcolor rgb '#FFFFFF' \n",
					mpVisualHexagons[i-1][j-1].getIndex(),
					mpVisualHexagons[i-1][j-1].getCenterX(),
					mpVisualHexagons[i-1][j-1].getCenterY(),
					mpVisualHexagons[i-1][j-1].getIndex());
			mDiscretizedMap << temp;
			//cout << temp;

			/*
			cout << " Index " << mpVisualHexagons[i-1][j-1].mIndexId << endl;
			cout << " 0 " << mpVisualHexagons[i-1][j-1].mXVertex[0];
			cout << " " << mpVisualHexagons[i-1][j-1].mYVertex[0] <<endl;
			cout << " 1 " << mpVisualHexagons[i-1][j-1].mXVertex[1];
			cout << " " << mpVisualHexagons[i-1][j-1].mYVertex[1] <<endl;
			cout << " 2 " << mpVisualHexagons[i-1][j-1].mXVertex[2];
			cout << " " << mpVisualHexagons[i-1][j-1].mYVertex[2] <<endl;
			cout << " 3 " << mpVisualHexagons[i-1][j-1].mXVertex[3];
			cout << " " << mpVisualHexagons[i-1][j-1].mYVertex[3] <<endl;
			cout << " 4 " << mpVisualHexagons[i-1][j-1].mXVertex[4];
			cout << " " << mpVisualHexagons[i-1][j-1].mYVertex[4] <<endl;
			cout << " 5 " << mpVisualHexagons[i-1][j-1].mXVertex[5];
			cout << " " << mpVisualHexagons[i-1][j-1].mYVertex[5] <<endl;
			cout << " 6 " << mpVisualHexagons[i-1][j-1].mXVertex[6];
			cout << " " << mpVisualHexagons[i-1][j-1].mYVertex[6] <<endl;
			cout << endl;
			*/

			//cout << "plotting " << i << " " << j << endl;
			//cout << "Index " << mpVisualHexagons[i][j].getIndex() << endl;
			number_count++;

			memset(temp,0x0,MAX_LEN);
			sprintf(temp, "set object %d polygon from %f,%f to %f,%f to %f,%f to %f,%f to %f,%f to %f,%f to %f,%f\n",
					mpVisualHexagons[i][j].getIndex(),
					mpVisualHexagons[i][j].getXVertex(0),
					mpVisualHexagons[i][j].getYVertex(0),
					mpVisualHexagons[i][j].getXVertex(1),
					mpVisualHexagons[i][j].getYVertex(1),
					mpVisualHexagons[i][j].getXVertex(2),
					mpVisualHexagons[i][j].getYVertex(2),
					mpVisualHexagons[i][j].getXVertex(3),
					mpVisualHexagons[i][j].getYVertex(3),
					mpVisualHexagons[i][j].getXVertex(4),
					mpVisualHexagons[i][j].getYVertex(4),
					mpVisualHexagons[i][j].getXVertex(5),
					mpVisualHexagons[i][j].getYVertex(5),
					mpVisualHexagons[i][j].getXVertex(6),
					mpVisualHexagons[i][j].getYVertex(6));
			mDiscretizedMap << temp;
			//cout << temp;


			memset(temp,0x0,MAX_LEN);
			if(mpVisualHexagons[i-1][j-1].getGrid()->mType==OBSTACLE)
			{
				sprintf(temp, "set object %d fillcolor rgb '#FF0000' fillstyle solid 1.0 border lt 1\n",
										mpVisualHexagons[i-1][j-1].getIndex());
			}
			else
			{
				sprintf(temp, "set object %d fillcolor rgb '%s' fillstyle solid 1.0 border lt 1\n",
						mpVisualHexagons[i][j].getIndex(),
						mpVisualHexagons[i][j].getRGBColor().c_str());
		    }
			mDiscretizedMap << temp;
			//cout << temp;

			memset(temp,0x0,MAX_LEN);
			sprintf(temp, "set label %d at %f,%f ' %d ' front center font ',5' textcolor rgb '#FFFFFF' \n",
					mpVisualHexagons[i][j].getIndex(),
					mpVisualHexagons[i][j].getCenterX(),
					mpVisualHexagons[i][j].getCenterY(),
					mpVisualHexagons[i][j].getIndex());
			mDiscretizedMap << temp;
			//cout << temp;

			/*
			cout << " Index " << mpVisualHexagons[i][j].mIndexId << endl;
			cout << " 0 " << mpVisualHexagons[i][j].mXVertex[0];
			cout << " " << mpVisualHexagons[i][j].mYVertex[0] <<endl;
			cout << " 1 " << mpVisualHexagons[i][j].mXVertex[1];
			cout << " " << mpVisualHexagons[i][j].mYVertex[1] <<endl;
			cout << " 2 " << mpVisualHexagons[i][j].mXVertex[2];
			cout << " " << mpVisualHexagons[i][j].mYVertex[2] <<endl;
			cout << " 3 " << mpVisualHexagons[i][j].mXVertex[3];
			cout << " " << mpVisualHexagons[i][j].mYVertex[3] <<endl;
			cout << " 4 " << mpVisualHexagons[i][j].mXVertex[4];
			cout << " " << mpVisualHexagons[i][j].mYVertex[4] <<endl;
			cout << " 5 " << mpVisualHexagons[i][j].mXVertex[5];
			cout << " " << mpVisualHexagons[i][j].mYVertex[5] <<endl;
			cout << " 6 " << mpVisualHexagons[i][j].mXVertex[6];
			cout << " " << mpVisualHexagons[i][j].mYVertex[6] <<endl;
			cout << endl;
			*/
		}

	}

    memset(temp,0x0,MAX_LEN);
	sprintf(temp, "plot %d \n", number_count);

    mDiscretizedMap << temp;
    //mDiscretizedMap << "set output 'tessellation.png'\n";

}

CVisualHexagon * CVisualHexagonDiscretizedMap::getHexagon(int index)
{
    for(int j=1;j<=mHeight;j+=2)
    {
    	for(int i=1;i<=mWidth;i+=2)
    	{
    		if(mpVisualHexagons[i-1][j-1].getIndex()==index)
    		{
    			return &(mpVisualHexagons[i-1][j-1]);
    		}

    		if(mpVisualHexagons[i][j].getIndex()==index)
    		{
    			return &(mpVisualHexagons[i][j]);
    		}

    	}
    }

    return NULL;
}

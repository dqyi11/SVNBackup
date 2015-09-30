/*
 * CWorldController.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: walter
 */

#include <vector>
#include <iostream>
#include <stdlib.h>
#include <assert.h>
#include "CWorldController.h"
//#include "CWorld.h";
#include "../CommandConst.h"
#include "CCommunicator.h"
#include "CTeam.h"
#include "CMyAgent.h"

CWorldController * CWorldController::mpInstance = NULL;

CWorldController::CWorldController()
{
	// TODO Auto-generated constructor stub

	mpWorld = new CWorld();
	mpCommunicator = NULL;
}

CWorldController::~CWorldController()
{
	// TODO Auto-generated destructor stub
}

bool CWorldController::init(CCommunicator * communicator)
{
	if (communicator)
	{
	    mpCommunicator = communicator;
	}
	else
	{
		return false;
	}

	initConstants();
	initTeams();
	initBases();
	initMyAgents();
	initOtherAgents();
	initObstacles();
	initFlags();

	if (mpWorld)
	{
		mpWorld->print();
	}

	return true;
}

bool CWorldController::initMyAgents()
{
	vector<string> v;
	CMyAgent * pAgent = NULL;
	int i = 0;

	if (mpCommunicator)
	{
		mpCommunicator->sendLine(MYTANKS);
		mpCommunicator->readAck();
		v = mpCommunicator->readArr();
		if(0 != v.at(0).compare(BEGIN))
		{
			return false;
		}
		v.clear();
		v = mpCommunicator->readArr();
		while(0 == v.at(0).compare(MYTANK))
		{
			pAgent = new CMyAgent(atoi(v.at(1).c_str()), v.at(2));
            //cout <<"at init: "<<v[3]<<" "<<v[4]<<" "<<v[5];
            //cout <<" "<<v[6]<<" "<<v[7]<<" "<<v[8]<<" "<<v[9]<<endl;
			pAgent->setStatus(v.at(3));
            pAgent->setShotsAvailable(atoi(v.at(4).c_str()));
            pAgent->setTimeToReload(atoi(v.at(5).c_str()));
            pAgent->setFlag(v.at(6));
            pAgent->setPosX(atof(v.at(7).c_str()));
            pAgent->setPosY(atof(v.at(8).c_str()));
            pAgent->setOrientation(atof(v.at(9).c_str()));
            pAgent->setVelocityX(atof(v.at(10).c_str()));
            pAgent->setVelocityY(atof(v.at(11).c_str()));
            pAgent->setAngleVelocity(atof(v.at(12).c_str()));

            if(mpWorld->mpMyTeam)
            {
            	mpWorld->mpMyTeam->addAgent((CAgent*)pAgent);
            }

			v.clear();
			v = mpCommunicator->readArr();
			i++;

		}

		if(0 != v.at(0).compare(END))
		{
			return false;
		}
	}
	return true;

}

bool CWorldController::initConstants()
{
	vector<string> v;
	CTeam * pTeam = NULL;
	int i = 0;

	if (mpCommunicator)
	{
		mpCommunicator->sendLine(CONSTANTS);
		mpCommunicator->readAck();
		v = mpCommunicator->readArr();
		if(0 != v.at(0).compare(BEGIN))
		{
			return false;
		}

		v.clear();
		v = mpCommunicator->readArr();

		while(0 == v.at(0).compare(CONSTANT))
		{
			if(0 == v.at(1).compare(TEAM))
			{
				pTeam = new CTeam(v.at(2));
				mpWorld->mpMyTeam = pTeam;
			}else
		    if(0 == v.at(1).compare(WORLDSIZE))
			{
				mpWorld->mSize = atoi(v.at(2).c_str());
			}else
			if(v[1].compare(TANKANGVEL) == 0){
				mpWorld->tankangvel = atof(v[2].c_str());
			}else
			if(v[1].compare(TANKLENGTH) == 0){
				mpWorld->tanklength = atof(v[2].c_str());
			}if(v[1].compare(TANKRADIUS) == 0){
				mpWorld->tankradius = atof(v[2].c_str());
			}else
			if(v[1].compare(TANKSPEED) == 0){
				mpWorld->tankspeed = atoi(v[2].c_str());
			}else
			if(v[1].compare(TANKALIVE) == 0){
				mpWorld->tankalive = v[2].c_str();
			}else
			if(v[1].compare(TANKDEAD) == 0){
				mpWorld->tankdead = v[2].c_str();
			}else
			if(v[1].compare(LINEARACCEL) == 0){
				mpWorld->linearaccel = atof(v[2].c_str());
			}else
			if(v[1].compare(ANGULARACCEL) == 0){
				mpWorld->angularaccel = atof(v[2].c_str());
			}else
			if(v[1].compare(TANKWIDTH) == 0){
				mpWorld->tankwidth = atof(v[2].c_str());
			}if(v[1].compare(SHOTRADIUS) == 0){
				mpWorld->shotradius = atof(v[2].c_str());
			}else
			if(v[1].compare(SHOTRANGE) == 0){
				mpWorld->shotrange = atoi(v[2].c_str());
			}if(v[1].compare(SHOTSPEED) == 0){
				mpWorld->shotspeed = atoi(v[2].c_str());
			}else
			if(v[1].compare(FLAGRADIUS) == 0){
				mpWorld->flagradius = atof(v[2].c_str());
			}else
			if(v[1].compare(EXPLODETIME) == 0){
				mpWorld->explodetime = atoi(v[2].c_str());
			}else
			if(v[1].compare(TRUEPOSITIVE) == 0){
				mpWorld->truepositive = atoi(v[2].c_str());
				mpWorld->truepositive = .97;
			}else
			if(v[1].compare(TRUENEGATIVE) == 0){
				mpWorld->truenegative = atoi(v[2].c_str());
				mpWorld->truenegative = .9;
			}

			v.clear();
			v = mpCommunicator->readArr();
		}

		if(0 != v.at(0).compare(END))
		{
			return false;
		}
		return true;
	}
	return false;
}

bool CWorldController::initTeams()
{
	vector<string> v;
	CTeam * pTeam = NULL;
	int i = 0;

	if (mpCommunicator)
	{
		mpCommunicator->sendLine(TEAMS);
		mpCommunicator->readAck();
		v = mpCommunicator->readArr();
		if(0 != v.at(0).compare(BEGIN))
		{
			return false;
		}
		v.clear();
		v = mpCommunicator->readArr();
		while(0 == v.at(0).compare(TEAM))
		{
			if (0 == mpWorld->mpMyTeam->getColor().compare(v.at(1)))
			{
				mpWorld->mpMyTeam->setNumCount(atoi(v.at(2).c_str()));

			}
			else
			{

				pTeam = new CTeam(v.at(1));
				pTeam->setNumCount(atoi(v.at(2).c_str()));

				mpWorld->otherTeam.push_back(pTeam);

			}

			v.clear();
			v = mpCommunicator->readArr();
			i++;
		}
		if(0 != v.at(0).compare(END))
		{
			return false;
		}
	}

	return true;
}

bool CWorldController::initObstacles(){
//	cout<<"in here"<<endl;

	vector<string> v;
	CObstacle * obstacle;


	if(mpCommunicator){
		mpCommunicator->sendLine(OBSTACLES);
		mpCommunicator->readAck();

		v = mpCommunicator->readArr();

		if(v[0].compare(BEGIN) != 0){
			return false;
		}
		//cout << v[0]<<endl;

		v.clear();
		v = mpCommunicator->readArr();

		while(v[0].compare(OBSTACLE) == 0)
		{
			//create new obstacle object
			obstacle = new CObstacle();

			//add first x y coordinates
			obstacle->xvals.push_back(atof(v[1].c_str()));
			obstacle->yvals.push_back(atof(v[2].c_str()));

			//add second x y coordinates
			obstacle->xvals.push_back(atof(v[3].c_str()));
			obstacle->yvals.push_back(atof(v[4].c_str()));

			//add third x y coordinates
			obstacle->xvals.push_back(atof(v[5].c_str()));
			obstacle->yvals.push_back(atof(v[6].c_str()));

			//add fourth x y coordinates
			obstacle->xvals.push_back(atof(v[7].c_str()));
			obstacle->yvals.push_back(atof(v[8].c_str()));

			assert(obstacle->xvals.size() == obstacle->yvals.size());

			mpWorld->obstacles.push_back(obstacle);

			v.clear();
			v = mpCommunicator->readArr();

		}


		if(v[0].compare(END) != 0)
		{
			return false;
		}


	}

	return true;
}



bool CWorldController::initOtherAgents()
{
	vector<string> v;
	CAgent * pAgent = NULL;
	CTeam * pTeam = NULL;
	int i = 0;

	if (mpCommunicator)
	{
		mpCommunicator->sendLine(OTHERTANKS);
		mpCommunicator->readAck();

		v = mpCommunicator->readArr();
		if(0 != v.at(0).compare(BEGIN))
		{
			return false;
		}
		v.clear();
		v = mpCommunicator->readArr();
		while(0 == v.at(0).compare(OTHERTANK))
		{
			pAgent = new CAgent(v.at(1));
			pAgent->setColor(v.at(2));
            pAgent->setStatus(v.at(3));
            pAgent->setFlag(v.at(4));
            pAgent->setPosX(atof(v.at(5).c_str()));
            pAgent->setPosY(atof(v.at(6).c_str()));
            pAgent->setOrientation(atof(v.at(7).c_str()));

            pTeam = mpWorld->findTeam(pAgent->getColor());
            pTeam->addAgent(pAgent);

			v.clear();
			v = mpCommunicator->readArr();
			i++;

		}

		if(0 != v.at(0).compare(END))
		{
			return false;
		}
	}

	return true;

}

bool CWorldController::initFlags(){

	vector<string> v;
	CFlag * flag;


	if(mpCommunicator){
		mpCommunicator->sendLine(FLAGS);
		mpCommunicator->readAck();

		v = mpCommunicator->readArr();

		if(v[0].compare(BEGIN) != 0){
			return false;
		}

		v.clear();
		v = mpCommunicator->readArr();

		while(v[0].compare(FLAG) == 0)
		{
			//create new flag
			flag = new CFlag();

			flag->color = v[1];
			flag->PTcolor = v[2];
			flag->x = atof(v[3].c_str());
			flag->y = atof(v[4].c_str());

			mpWorld->flags.push_back(flag);

			v.clear();
			v = mpCommunicator->readArr();

		}


		if(v[0].compare(END) != 0)
		{
			return false;
		}


	}

	return true;
}

bool CWorldController::initBases()
{
	vector<string> v;
	CTeam * pTeam = NULL;

	if(mpCommunicator){
		mpCommunicator->sendLine(BASES);
		mpCommunicator->readAck();

		v = mpCommunicator->readArr();

		if(v[0].compare(BEGIN) != 0){
			return false;
		}

		v.clear();
		v = mpCommunicator->readArr();

		while(0 == v[0].compare(BASE))
		{
			if (0==mpWorld->mpMyTeam->getColor().compare(v[1]))
			{
				pTeam = mpWorld->mpMyTeam;
			}
			else
			{
			    pTeam = mpWorld->findTeam(v[1]);
			}

			if(pTeam)
			{
				pTeam->base->xvals.push_back(atof(v.at(2).c_str()));
				pTeam->base->yvals.push_back(atof(v.at(3).c_str()));
				pTeam->base->xvals.push_back(atof(v.at(4).c_str()));
				pTeam->base->yvals.push_back(atof(v.at(5).c_str()));
				pTeam->base->xvals.push_back(atof(v.at(6).c_str()));
				pTeam->base->yvals.push_back(atof(v.at(7).c_str()));
				pTeam->base->xvals.push_back(atof(v.at(8).c_str()));
				pTeam->base->yvals.push_back(atof(v.at(9).c_str()));
			}

			v.clear();
			v = mpCommunicator->readArr();

		}


		if(v[0].compare(END) != 0)
		{
			return false;
		}


	}

	return true;

}

CWorldController * CWorldController::getInstance()
{
	if (NULL == CWorldController::mpInstance)
	{
		CWorldController::mpInstance = new CWorldController();
	}

	return CWorldController::mpInstance;
}

bool CWorldController::updateWorld(void)
{
	updateMyAgents();
	updateOtherAgents();
	updateFlags();
	return true;
}

bool CWorldController::updateMyAgents(void)
{
	vector<string> v;
	CMyAgent * pAgent = NULL;
	int i = 0;

	if (mpCommunicator)
	{
		mpCommunicator->sendLine(MYTANKS);
		mpCommunicator->readAck();
		v = mpCommunicator->readArr();
		if(0 != v.at(0).compare(BEGIN))
		{
			return false;
		}
		v.clear();
		v = mpCommunicator->readArr();
		while(0 == v.at(0).compare(MYTANK))
		{
			pAgent = mpWorld->findMyAgent(v.at(2));
			//cout << "find agent:" << pAgent->getCallSign() << " " << pAgent->getIndex() << endl;
            //cout << "checking"<<v[3]<<v[4]<<v[5]<<v[6]<<v[7]<<v[8]<<endl;
			pAgent->setStatus(v.at(3));
            pAgent->setShotsAvailable(atoi(v.at(4).c_str()));
            pAgent->setTimeToReload(atoi(v.at(5).c_str()));
            pAgent->setFlag(v.at(6));
            pAgent->setPosX(atof(v.at(7).c_str()));
            pAgent->setPosY(atof(v.at(8).c_str()));
            pAgent->setOrientation(atof(v.at(9).c_str()));
            pAgent->setVelocityX(atof(v.at(10).c_str()));
            pAgent->setVelocityY(atof(v.at(11).c_str()));
            pAgent->setAngleVelocity(atof(v.at(12).c_str()));
            //cout << "pos is:" << v.at(7) << " " << v.at(8) << endl;

			v.clear();
			v = mpCommunicator->readArr();
			i++;

		}

		if(0 != v.at(0).compare(END))
		{
			return false;
		}
	}
	return true;
}

bool CWorldController::updateFlags(void)
{
	vector<string> v;
	CFlag * flag = NULL;

	if(mpCommunicator){
		mpCommunicator->sendLine(FLAGS);
		mpCommunicator->readAck();

		v = mpCommunicator->readArr();

		if(v[0].compare(BEGIN) != 0){
			return false;
		}

		v.clear();
		v = mpCommunicator->readArr();

		while(v[0].compare(FLAG) == 0)
		{
			//create new flag
			flag = mpWorld->findFlag(v[1]);

			//flag->color = v[1];
			flag->PTcolor = v[2];
			flag->x = atof(v[3].c_str());
			flag->y = atof(v[4].c_str());

			v.clear();
			v = mpCommunicator->readArr();

		}


		if(v[0].compare(END) != 0)
		{
			return false;
		}


	}

	return true;
}

bool CWorldController::updateOtherAgents()
{
	vector<string> v;
	CAgent * pAgent = NULL;
	CTeam * pTeam = NULL;
	int i = 0;

	if (mpCommunicator)
	{
		mpCommunicator->sendLine(OTHERTANKS);
		mpCommunicator->readAck();

		v = mpCommunicator->readArr();
		if(0 != v.at(0).compare(BEGIN))
		{
			return false;
		}
		v.clear();
		v = mpCommunicator->readArr();
		while(0 == v.at(0).compare(OTHERTANK))
		{
			pAgent = mpWorld->findTeam(v.at(2))->findAgent(v.at(1));
			//pAgent->setColor(v.at(2));
            pAgent->setStatus(v.at(3));
            pAgent->setFlag(v.at(4));
            pAgent->setPosX(atof(v.at(5).c_str()));
            pAgent->setPosY(atof(v.at(6).c_str()));
            pAgent->setOrientation(atof(v.at(7).c_str()));

			v.clear();
			v = mpCommunicator->readArr();
			i++;

		}

		if(0 != v.at(0).compare(END))
		{
			return false;
		}
	}

	return true;

}

/*
 * CGridFilter.cpp
 *
 */

#include "CGridFilter.h"

CGridFilter::CGridFilter(double tHit, double fDetection, CGridMap * cmap){
	//assert(tHit <=1 && fDetection <= 1);
	trueHit = tHit;
	trueMiss = fDetection;
	falseAlarm = 1 - trueMiss;
	missedDetection = 1 - trueHit;
	//trueMiss = 1 - falseAlarm; //1-falseAlarm

	map = cmap;
}

CGridFilter::~CGridFilter() {
	// TODO Auto-geerated destructor stub
}

void CGridFilter::filter(COccGridState  * state){
	//Assuming observation 1 is hit, 0 is no hit and else is no data
	int i = state->getPosX();
	int j = state->getPosY();
	int height = state->getHeight();
	int width = state->getWidth();
	int obs;
	double bel_occ, bel_unocc;


	for(int x = i; x < i + height; x++ ){
		for(int y = j; y < j + width; y++){
		//for(int y = j; y < (j+width);y++){
			// Get observation from parsed occrid command
			obs = state->getState(x,y);


			if(obs == 1){
				// If detected a hit
				bel_occ = trueHit * map->getProbability(x,y);
				bel_unocc = falseAlarm * (1- map->getProbability(x,y));

				map->setProbability(x,y,(bel_occ/(bel_occ+bel_unocc)));
			}else{
				// If detected a miss
				bel_occ = missedDetection * map->getProbability(x,y);
				bel_unocc = trueMiss * (1- map->getProbability(x,y));

				map->setProbability(x,y,(bel_occ/(bel_occ+bel_unocc)));
			}

		}

	}

}

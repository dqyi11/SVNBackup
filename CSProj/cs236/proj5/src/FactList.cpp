/*
 * FactsList.cpp
 *
 *  Created on: Feb 5, 2013
 *      Author: kyle
 */

#include "FactList.h"

//namespace std {

FactList::FactList(Lex* lex, Domain* domain) {
	if( lex->assertCurrentToken(FACTS) )
		lex->advance();
	if( lex->getErrorToken() == NULL )
		if( lex->assertCurrentToken(COLON) )
			lex->advance();
	while(lex->getCurrentToken()->getTokenType() == ID){
		facts.push_back(new Fact(lex, domain));
	}
}

FactList::~FactList() {
	// TODO Auto-generated destructor stub
}

string FactList::toString() {
	stringstream rep;
	rep << "Facts(" << facts.size() << "):" << endl;
	for(vector<Fact*>::iterator it = facts.begin(); it != facts.end(); ++it)
		rep << "  " << (*it)->toString() << endl;

	return rep.str();
}

//}

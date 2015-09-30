/*
 * SchemeList.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: kyle
 */

#include "SchemeList.h"

using namespace std;

SchemeList::SchemeList(Lex* lex) {
	if( lex->assertCurrentToken(SCHEMES) )
		lex->advance();
	if( lex->getErrorToken() == NULL )
		if( lex->assertCurrentToken(COLON) )
			lex->advance();
	lex->assertCurrentToken(ID);
	while(lex->getCurrentToken()->getTokenType() == ID)
		schemes.push_back(new Scheme(lex));
}

SchemeList::~SchemeList() {
	// TODO Auto-generated destructor stub
}

string SchemeList::toString() {
	stringstream rep;
	rep << "Schemes(" << schemes.size() << "):" << endl;
	for(vector<Scheme*>::iterator it = schemes.begin(); it != schemes.end(); ++it){
		rep << "  " << (*it)->toString() << endl;
	}
	return rep.str();
}

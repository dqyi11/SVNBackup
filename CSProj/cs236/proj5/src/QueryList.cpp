/*
 * QueryList.cpp
 *
 *  Created on: Feb 5, 2013
 *      Author: kyle
 */

#include "QueryList.h"

namespace std {

QueryList::QueryList(Lex* lex, Domain* domain) {
	if( lex->assertCurrentToken(QUERIES) )
		lex->advance();
	if( lex->getErrorToken() == NULL )
		if( lex->assertCurrentToken(COLON) )
			lex->advance();
	lex->assertCurrentToken(ID);
	while(lex->getCurrentToken()->getTokenType() == ID){
		queries.push_back(new Predicate(lex, domain));
		lex->assertCurrentToken(Q_MARK);
		lex->advance();
	}
}

string QueryList::toString() {
	stringstream rep;
	rep << "Queries(" << queries.size() << "):" << endl;
	for(vector<Predicate*>::iterator it = queries.begin(); it != queries.end(); ++it){
		rep << "  " << (*it)->toString() << "?" << endl;
	}
	return rep.str();

}

} /* namespace std */

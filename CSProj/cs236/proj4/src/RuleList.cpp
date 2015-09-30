/*
 * RuleList.cpp
 *
 *  Created on: Feb 5, 2013
 *      Author: kyle
 */

#include "RuleList.h"

//amespace std {

RuleList::RuleList(Lex* lex, Domain* domain) {
	if( lex->assertCurrentToken(RULES) )
		lex->advance();
	if( lex->getErrorToken() == NULL )
		if( lex->assertCurrentToken(COLON) )
			lex->advance();
	while(lex->getCurrentToken()->getTokenType() == ID)
		rules.push_back(new Rule(lex, domain));
}

RuleList::~RuleList() {
	// TODO Auto-generated destructor stub
}

string RuleList::toString() {
	stringstream rep;
	rep << "Rules(" << rules.size() << "):" << endl;
	for(vector<Rule*>::iterator it = rules.begin(); it != rules.end(); ++it){
		rep << "  " << (*it)->toString() << endl;
	}
	return rep.str();
}

//} /* namespace std */


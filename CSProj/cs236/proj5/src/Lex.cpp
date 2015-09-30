#include "Lex.h"

#include "Input.h"
#include "TokenType.h"
#include "Utils.h"
#include "ctype.h"
#include <iostream>

using namespace std;

Lex::Lex() {
	input = new Input();
    generateTokens(input);
}

Lex::Lex(const char* filename): errorToken(NULL) {
    input = new Input(filename);
    failToken = new Token(UNDEFINED, "", 0);
    generateTokens(input);
}

Lex::Lex(istream& istream) {
    input = new Input(istream);
    generateTokens(input);
}

Lex::Lex(const Lex& lex) {
    input = new Input(*lex.input);
    tokens = new vector<Token*>();

    vector<Token*>::iterator iter;
    for(iter=lex.tokens->begin(); iter != lex.tokens->end(); iter++) {
        Token* newToken = new Token(**iter);
        tokens->push_back(newToken);
    }

    index = lex.index;
    state = lex.state;
}

Lex::~Lex(){
    for (int i = 0; i < tokens->size(); i++) {
        delete (*tokens)[i];
    }
    delete tokens;
    delete input;
}

bool Lex::operator==(const Lex& lex) {
    bool result = (tokens->size() == lex.tokens->size()) && (index == lex.index);
    if(result) {
        vector<Token*>::iterator iter1;
        vector<Token*>::iterator iter2;
        iter1 = tokens->begin();
        iter2 = lex.tokens->begin();
        while(result && iter1 != tokens->end() && iter2 != lex.tokens->end()) {
            result = **iter1 == **iter2;
            iter1++;
            iter2++;
        }
        result = result && iter1 == tokens->end() && iter2 == lex.tokens->end();
    }
    return result;
}

string Lex::toString() const {
    int count = 0;
    string result;
    while(count < tokens->size()) {
        Token* token = (*tokens)[count];
        result += token->toString();
        count++;
    }
    result += "Total Tokens = ";
    string countToString;
    result += itoa(countToString, count);
    result += "\n";
    return result;
}

void Lex::generateTokens(Input* input) {
    tokens = new vector<Token*>();
    index = 0;

    state = Start;
    while(state != End) {
        state = nextState();
    }
    emit(EOF); //this should always be the last token, right?
}

Token* Lex::getCurrentToken() {
	if( getErrorToken() != NULL)
		return failToken;

    return (*tokens).at(index);
}
Token* Lex::nextToken() {
	Token* current = getCurrentToken();
	advance();
	return current;
}

void Lex::advance() {
	if( errorToken == NULL ) //don't keep advancing if there's an error
		index++;
}

bool Lex::hasNext() {
    return index < tokens->size();
}


State Lex::nextState() {
    State result;
    char character;
    switch(state) {
        case Start:				result = getNextState(); break;
        case Comma:				emit(COMMA); result = getNextState(); break;
        case Period:			emit(PERIOD); result = getNextState(); break;
        case QMark:				emit(Q_MARK); result = getNextState(); break;
        case OpenParen:			emit(LEFT_PAREN); result = getNextState(); break;
        case CloseParen:		emit(RIGHT_PAREN); result = getNextState(); break;
        case Add:				emit(ADD); result = getNextState(); break;
        case Multiply:			emit(MULTIPLY); result = getNextState(); break;

        case SawColon:
            character = input->getCurrentCharacter();
            if(character == '-') {
                result = Colon_Dash;
                input->advance();
            } else { //Every other character
            	emit(COLON);
            	result = getNextState();
            }
            break;
        case Colon_Dash:          emit(COLON_DASH); result = getNextState(); break;
        case SawAQuote:  
            character = input->getCurrentCharacter();
            if(character == '\'') {
                result = PossibleEndOfString;
            } else if(character == -1) {
            	result = UnexpectedEnd;
            } else { //Every other character
                result = ProcessingString;
            }
            input->advance();
            break;
        case ProcessingString:  
            character = input->getCurrentCharacter();
            if(character == '\'') {
                result = PossibleEndOfString;
            } else if(character == -1) {
            	result = UnexpectedEnd;
            } else { //Every other character
                result = ProcessingString;
            }
            input->advance();
            break;
        case PossibleEndOfString:
            if(input->getCurrentCharacter() == '\'') {
                input->advance();
                result = ProcessingString;
            } else { //Every other character
            	emit(STRING);
                result = getNextState();
            }
            break;
        case Whitespace:
        	input->mark();
        	result = getNextState();
        	break;
        case ProcessingID:
        	character = input->getCurrentCharacter();
        	if(isalpha(character) || isdigit(character)) {
        		result = ProcessingID;
        		input->advance();
        	} else {
        		if( input->getTokensValue() == "Schemes" )
        			emit(SCHEMES);
        		else if( input->getTokensValue() == "Facts" )
        			emit(FACTS);
        		else if( input->getTokensValue() == "Rules" )
        			emit(RULES);
        		else if( input->getTokensValue() == "Queries" )
        			emit(QUERIES);
        		else
        			emit(ID);
        		result = getNextState();
        	}
        	break;
        case EnterComment:
        	character = input->getCurrentCharacter();
        	if(character == '|'){
        		result = ProcessingMLComment;
        		input->advance();
        	}
        	else if( character == '\n' || character == -1 ){
        		emit(COMMENT);
        		result = getNextState();
        	}
        	else{
        		result = ProcessingComment;
        		input->advance();
        	}
        	break;
        case ProcessingComment:
        	character = input->getCurrentCharacter();
        	if( character == '\n' || character == -1 ){ //end of comment
        		emit(COMMENT);
        		result = getNextState();
        	} else {
        		result = ProcessingComment;
        		input->advance();
        	}
        	break;
        case ProcessingMLComment:
        	character = input->getCurrentCharacter();
        	if( character == '|' ){
        		result = PossibleMLEnd;
        	} else if( character == -1 )
        		result = UnexpectedEnd;
        	else
        		result = ProcessingMLComment;
			input->advance();
        	break;
        case PossibleMLEnd:
        	character = input->getCurrentCharacter();
        	if( character == '#' ){
        		input->advance(); //consume that last #
        		emit(COMMENT);
        		result = getNextState();
        	}else if( character == '|' ){
				input->advance();
				result = PossibleMLEnd;
        	} else {
        		input->advance();
        		result = ProcessingMLComment;
        	}
    	break;
        case Undefined:				emit(UNDEFINED); result=getNextState(); break;
        case UnexpectedEnd: 		emit(UNDEFINED); result=End; break; //EOF found during string search
        case End:	break;
    };
    return result;
}

State Lex::getNextState() {
    State result;
    char currentCharacter = input->getCurrentCharacter();

    //The handling of checking for whitespace and setting the result to Whitespace and
    //checking for letters and setting the result to Id will probably best be handled by
    //if statements rather then the switch statement.
    if(isspace(currentCharacter))
    	result = Whitespace;
    else if(isalpha(currentCharacter))
    	result = ProcessingID;
    else{
		switch(currentCharacter) {
			case ','  : result = Comma; break;
			case '.'  : result = Period; break;
			case '?'  : result = QMark; break;
			case '('  : result = OpenParen; break;
			case ')'  : result = CloseParen; break;
			case '+'  : result = Add; break;
			case '*'  : result = Multiply; break;
			case ':'  : result = SawColon; break;
			case '\'' : result = ProcessingString; break;
			case '#'  : result = EnterComment; break;
			case -1   : result = End; break;
			default:
				//string error = "ERROR:: in Lex::getNextState, Expecting  ";
				//error += "'\'', '.', '?', '(', ')', '+', '*', '=', '!', '<', '>', ':' but found ";
				//error += currentCharacter;
				//error += '.';
				//throw error.c_str();
				result = Undefined; break;
		}
    }
    input->advance();
    return result;
}

void Lex::emit(TokenType tokenType) {
    Token* token = new Token(tokenType, input->getTokensValue(), input->getCurrentTokensLineNumber());
    storeToken(token);
    input->mark();
}

void Lex::storeToken(Token* token) {
    //This section should ignore whitespace and comments and change the token type to the appropriate value
    //if the value of the token is "Schemes", "Facts", "Rules", or "Queries".
    tokens->push_back(token);
}

void Lex::markErrorToken(){ if( errorToken == NULL ) errorToken = getCurrentToken(); }

Token* Lex::getErrorToken() {
	return errorToken;
}

bool Lex::assertCurrentToken(TokenType assertedType) {
	if( getCurrentToken()->getTokenType() == assertedType )
		return true;
	markErrorToken();
	return false;
}

void Lex::clearComments() {
    for(vector<Token*>::iterator it=tokens->begin(); it != tokens->end(); ++it)
    	while((*it)->getTokenType() == COMMENT )
    		tokens->erase(it);


}

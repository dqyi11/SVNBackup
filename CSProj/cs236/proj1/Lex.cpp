#include "Lex.h"

#include "Input.h"
#include "TokenType.h"
#include "Utils.h"
#include <iostream>

using namespace std;

Lex::Lex() {
	input = new Input();
    generateTokens(input);
}

Lex::Lex(const char* filename) {
    input = new Input(filename);

    //cout << input->toString() << endl;
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
}

Token* Lex::getCurrentToken() {
    return (*tokens)[index];
}

void Lex::advance() {
    index++;
}

bool Lex::hasNext() {
    return index < tokens->size();
}

State Lex::nextState() {
    State result;
    char character;

    // cout << "Lex::nextState = " << StateToString(state) << endl;
    switch(state) {
        case Start:               input->mark(); result = getNextState(); break;
        case Comma:               emit(COMMA); result = getNextState(); break;
        case Period:              emit(PERIOD); result = getNextState(); break;
        case Q_Mark:              emit(Q_MARK); result = getNextState(); break;
        case Left_Paren:          emit(LEFT_PAREN); result = getNextState(); break;
        case Right_Paren:         emit(RIGHT_PAREN); result = getNextState(); break;
        case SawColon:
            character = input->getCurrentCharacter();
            if(character == '-') {
                result = Colon_Dash;
                input->advance();
            } else { //Every other character
            	emit(COLON);
            	result = Start;
                //throw "ERROR:: in case SawColon:, Expecting  '-' but found " + character + '.';
            }
            break;
        case Colon_Dash:          emit(COLON_DASH); result = getNextState(); break;
        case SawAQuote:  
            character = input->getCurrentCharacter();
            if(character == '\'') {
                result = PossibleEndOfString;
            } else if(character == -1) {
                throw "ERROR:: in Saw_A_Quote::nextState, reached EOF before end of string.";
            } else { //Every other character
                result = ProcessingString;
            }
            input->advance();
            break;
        case Multiply:            emit(MULTIPLY); result = getNextState(); break;
        case Add:                 emit(ADD); result = getNextState(); break;
        case ProcessingString:  
            character = input->getCurrentCharacter();
            if(character == '\'') {
                result = PossibleEndOfString;
            } else if(character == -1) {
            	emit(UNDEFINED);
            	input->mark();
            	emit(EOF);
            	result = End;
                //throw "ERROR:: in ProcessingString::nextState, reached EOF before end of string.";
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
        case SawPound:
        	character = input->getCurrentCharacter();
			if(character == '|') {
				result = ProcessingBlockComment;
				input->advance();
			} else if(character == '\n') {
        		result = PossibleEndOfLineComment;
        		emit(COMMENT);
        		result = getNextState();
			} else {
				result = ProcessingLineComment;
				input->advance();
			}
        	break;
        case ProcessingLineComment:
        	character = input->getCurrentCharacter();
        	if(character == '\n' || character == -1) {
        		result = PossibleEndOfLineComment;
        		emit(COMMENT);
        		result = getNextState();

        	} else {
        		result = ProcessingLineComment;
				input->advance();
        	}

        	break;
        case PossibleEndOfLineComment:  	emit(COMMENT); result = getNextState(); break;
        case ProcessingBlockComment:
        	character = input->getCurrentCharacter();
        	if(character == '|') {
        		result = PossibleEndOfBlockComment;
        	} else if(character == -1)
        	{
        		emit(UNDEFINED);
        		input->mark();
        		emit(EOF);
        		result = End;
        		break;
        	}
        	else {
        		result = ProcessingBlockComment;
        	}
        	input->advance();
        	break;
        case PossibleEndOfBlockComment:
        	character = input->getCurrentCharacter();
        	if(character == '#') {
        		result = EndOfBlockComment;
        	} else if(character == -1) {
        		emit(UNDEFINED);
        		input->mark();
        		emit(EOF);
        		result = End;
        		break;
        		//throw "ERROR:: in PossibleEndOfBlockComment::nextState, reached EOF before end of string.";
        	} else if(character == '|')
        	{
        		result = PossibleEndOfBlockComment;
        	} else { //Every other character
                result = ProcessingBlockComment;
            }
        	input->advance();
        	break;
        case EndOfBlockComment:    emit(COMMENT); result = getNextState(); break;
        case ProcessingID:
        	character = input->getCurrentCharacter();
        	if(isLetter(character) || isDigit(character))
        	{
        		result = ProcessingID;
        		input->advance();
        	}
        	else
        	{
        		string currentTokenValue = input->getTokensValue();
        		if(isKeyword(currentTokenValue)>=0)
        		{
        			TokenType type = getKeywordType(currentTokenValue);
        			emit(type);
        			result = Start;
        			break;
        		}
        		else
        		{
        			emit(ID);
        			result = Start;
        		}
        	}
        	break;
        case Undefined:            emit(UNDEFINED); result = getNextState(); break;
        case Whitespace:           emit(WHITESPACE); result = getNextState(); break;
        case End:
        	//emit(EOF);
        	throw "ERROR:: in End state:, the Input should be empty once you reach the End state.";
            break;
    };
    return result;
}

State Lex::getNextState() {
    State result;
    char currentCharacter = input->getCurrentCharacter();

    //cout << "Lex::getNextState " << currentCharacter << endl;

    //The handling of checking for whitespace and setting the result to Whilespace and
    //checking for letters and setting the result to Id will probably best be handled by
    //if statements rather then the switch statement.
    if(isspace(currentCharacter))
    {
    	result = Whitespace;
    }
    else if(isLetter(currentCharacter))
	{
    	if(result!=ProcessingString
    			|| result!=ProcessingLineComment)
    	{
    		result = ProcessingID;
    	}

	}
    else if(isDigit(currentCharacter))
    {
    	if(result!=ProcessingID)
    	{
    		result = Undefined;
    	}
    }
    else
    switch(currentCharacter) {
        case ','  : result = Comma; break;
        case '.'  : result = Period; break;
        case '?'  : result = Q_Mark; break;
        case '('  : result = Left_Paren; break;
        case ')'  : result = Right_Paren; break;
        case ':'  : result = SawColon; break;
        case '*'  : result = Multiply; break;
        case '+'  : result = Add; break;
        case '\'' : result = ProcessingString; break;
        case '#'  : result = SawPound; break;
        case '$'  :
        case '&'  :
        case '^'  : result = Undefined; break;
        case -1   : emit(EOF); result = End; break;
        default: 
        	/*
            string error = "ERROR:: in Lex::getNextState, Expecting  ";
            error += "'\'', '.', '?', '(', ')', '+', '*', '=', '!', '<', '>', ':' but found ";
            error += currentCharacter;
            error += '.';
            throw error.c_str();
            */
        	result = Undefined;
            break;
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
    //This section shoud ignore whitespace and comments and change the token type to the appropriate value
    //if the value of the token is "Schemes", "Facts", "Rules", or "Queries".
	if(NULL==token)
	{
		return;
	}
	// Ignore whitespace
	if(WHITESPACE==token->getTokenType())
	{
		return;
	}
    tokens->push_back(token);
}

int main(int argc, char* argv[]) {
    Lex lex(argv[1]);
    cout << lex.toString();
    return 0;
}

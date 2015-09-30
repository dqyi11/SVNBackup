#include "Token.h"

#include "Utils.h"

#include <string>
#include <sstream>

using namespace std;

Token::Token() {
    tokenType = NUL;
    lineNumber = 0;
    value = "";
}

Token::Token(TokenType newType, string newValue, unsigned newTokensLineNumber) {
    tokenType = newType;
    lineNumber = newTokensLineNumber;
    value = newValue;
}

Token::Token(const Token& token) {
    tokenType = token.tokenType;
    lineNumber = token.lineNumber;
    value = token.value;
}

Token::~Token(){};

TokenType Token::getTokenType() const {
    return tokenType;
}

unsigned Token::getLineNumber() const {
    return lineNumber;
}

string Token::getTokensValue() const {
    return value;
}

string Token::toString() const {
    string lineNumberString;
    itoa(lineNumberString, lineNumber);
    string string = "(";
    string += TokenTypeToString(tokenType);
    string += ",\"";
    string += value;
    string += "\",";
    string += lineNumberString;
    string += ")\n";
    return string;
};

bool Token::operator==(const Token& token) {
    bool result = lineNumber == token.lineNumber &&
                  value == token.value &&
                  tokenType == token.tokenType;
    return result;
}

void Token::setTokenType(const TokenType newTokenType) {
    tokenType = newTokenType;
}

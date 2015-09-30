#include "Input.h"

#include "Utils.h"

#include <iostream>
#include <fstream>
using namespace std;

void Input::init(istream& in) {
    while(!in.eof()) {
        char c = in.get();
    if(in.eof())break;
        characters += c;
    };
    characters += -1;

    lineNumber = 1;
    currentTokensLineNumber = 1;
    currentCharacterLocation = 0;
    currentTokensStartLocation = 0;
};

Input::Input() {
    characters += -1;
    lineNumber = 1;
    currentTokensLineNumber = 1;
    currentCharacterLocation = 0;
    currentTokensStartLocation = 0;
}

Input::Input(const char* fileName) {
    ifstream in(fileName, ios::in);
    if(in.is_open()){
        init(in);
    };
    in.close();
};

Input::Input(istream& in) {
    init(in);
};

Input::Input(const Input& input) 
{
    characters.assign(input.characters);
    lineNumber = input.lineNumber;
    currentTokensLineNumber = input.currentTokensLineNumber;
    currentCharacterLocation = input.currentCharacterLocation;
    currentTokensStartLocation = input.currentTokensStartLocation;
}

Input::~Input(){};

bool Input::operator==(const Input& input) const {
    bool result =
        characters.compare(input.characters) == 0                      &&
        currentCharacterLocation == input.currentCharacterLocation     &&
        currentTokensStartLocation == input.currentTokensStartLocation &&
        lineNumber == lineNumber                                       &&
        currentTokensLineNumber == currentTokensLineNumber;
    return result;
}

string Input::toString() const{
    string result; 
    string intString;
    result += "characters = \"";
    result += characters.substr(0, characters.length()-1);
    result += "\"\n";
    result += "currentCharacterLocation = " + itoa(intString, currentCharacterLocation) + "\n";
    result += "currentTokensStartLocation = " + itoa(intString, currentTokensStartLocation) + "\n";
    result += "lineNumber = " + itoa(intString, lineNumber) + "\n";
    result += "currentTokensLineNumber = " + itoa(intString, lineNumber) + "\n";
    return result;
}


char Input::getCurrentCharacter() const {
    return characters[currentCharacterLocation];
};

unsigned int Input::getCurrentTokensLineNumber() const {
    return currentTokensLineNumber;
};

bool Input::eof() const {
    return characters[currentCharacterLocation] == -1;
};

string Input::getTokensValue() const {
    return characters.substr(currentTokensStartLocation, currentCharacterLocation - currentTokensStartLocation);
};

void  Input::advance() {
    if (!eof()) {
        if(characters[currentCharacterLocation] == '\n') {
            lineNumber++;
        }
        currentCharacterLocation++;
    }
};

void Input::mark() {
    currentTokensStartLocation = currentCharacterLocation;
    currentTokensLineNumber = lineNumber;
};

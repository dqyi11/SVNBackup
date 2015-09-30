#include "Utils.h"

#include <sstream>

using namespace std;

const int keywordsNum = 4;
const string keywords[] = {"Schemes", "Facts", "Rules", "Queries"};
const TokenType keywordTokens[] = {SCHEMES, FACTS, RULES, QUERIES};

string& itoa(string& answer, int i) {
    stringstream ss;
    ss << i;
    answer = ss.str();
    return answer;
}

bool isLetter(char character)
{
	if((character>='a' && character<='z')
			||
		(character>='A' && character<='Z'))
	{
		return true;
	}

	return false;
}

bool isDigit(char character)
{
	if(character>='0' && character<='9')
	{
		return true;
	}

	return false;
}

int isKeyword(string word)
{
	for(int i=0;i<keywordsNum;i++)
	{
		if(0==word.compare(keywords[i]))
		{
			return i;
		}
	}

	return -1;
}

TokenType getKeywordType(string word)
{
	int index = isKeyword(word);

	if(index>=0)
	{
		return keywordTokens[index];
	}

	return NUL;
}

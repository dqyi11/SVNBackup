/*
 * Domain.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#include "Domain.h"
#include "sstream"

Domain::Domain() {
	// TODO Auto-generated constructor stub
	mDomainTokens.clear();
}

Domain::~Domain() {
	// TODO Auto-generated destructor stub
	mDomainTokens.clear();
}

std::string Domain::toString()
{
	std::stringstream strStream;
	std::string output = "";

	strStream.clear();
	strStream << "Domain";
	strStream << "(";
	strStream << mDomainTokens.size();
	strStream << "):";
	strStream >> output;
	strStream.clear();

	output += "\n";

	list<Token>::iterator it;
	for(it=mDomainTokens.begin();it!=mDomainTokens.end();it++)
	{
		output += "  ";
		output += (*it).getTokensValue();
		output += "\n";
	}

	return output;
}

bool Domain::hasToken(Token token)
{
	list<Token>::iterator it;
	for(it=mDomainTokens.begin();it!=mDomainTokens.end();it++)
	{
		if((*it).getTokensValue().compare(token.getTokensValue())==0)
		{
			return true;
		}
	}
	return false;
}

void Domain::addToken(Token token)
{
	if(STRING!=token.getTokenType())
	{
		return;
	}
	if(!hasToken(token))
	{
		if(mDomainTokens.size()==0)
		{
			mDomainTokens.push_back(token);
		}
		else
		{
			bool inserted = false;
			list<Token>::iterator it;
			for(it=mDomainTokens.begin();it!=mDomainTokens.end();it++)
			{
				if((*it).getTokensValue().compare(token.getTokensValue())>0)
				{
					mDomainTokens.insert(it,token);
					inserted = true;
					break;
				}
			}
			if(false==inserted)
			{
				mDomainTokens.push_back(token);
			}
		}
	}
}

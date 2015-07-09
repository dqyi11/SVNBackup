#ifndef UTILITIES
#define UTILITIES

#include <list>
#include "subtree.h"

bool hasDuplication(std::list<RRTNode*> list)
{
    for(std::list<RRTNode*>::iterator it1 = list.begin(); it1!=list.end() ; it1++)
    {
        for(std::list<RRTNode*>::iterator it2 = list.begin(); it1!=list.end() ; it2++)
        {
            if(it1!=it2 && (*it1)==(*it2))
            {
                return true;
            }
        }
    }
    return false;
}

#endif // UTILITIES


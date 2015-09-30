/*
 * Datalog.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: walter
 */

#include <iostream>
#include "Datalog.h"

using namespace std;

int main(int argc, char* argv[]) {
    Parser parser(argv[1]);
    //parser.printTokens();
     cout << parser.parseTokens();

    return 0;
}


#ifndef Input_h
#define Input_h

#include <istream>

using namespace std;

/**
 * Constains the sequence of characters read in from the input file. It also
 * remembers the current location in the sequence, the line number on
 * which the current token starts, and the value of the current token upto
 * the current location.
 *
 * DOMAIN:
 *      characters:                 Sequence<char> 
 *      currentCharacterLocation:   unsigned
 *      currentTokensStartLocation: unsigned
 *      currentTokensLineNumber:    unsigned
 *      lineNumber:                 unsigned
 */
class Input {
    public:
    //Constructors
        /**
         * The default constructor.
         *
         * Parameters: none
         * PreCondition: none
         * PostCondition: 
         *    |characters| = 1               AND
         *    character[0] = EOF
         *    currentCharacterLocation = 0   AND
         *    currentTokensStartLocation = 0 AND
         *    lineNumber = 1                 AND
         *    currentTokensLineNumber = 1
         */
        Input();

        /**
         * Reads in the characters from the file indicated and initializes
         * currentCharacterLocation, currentTokensStartLocation, and
         * currentTokensLineNumber
         *
         * Parameters: fileName -- the name of the file to be read from
         * PreCondition: the file to be read from exists and is readable.
         * PostCondition:
         *    characters = contents of file specified by file name with
         *        the EOF character appended to the end                  AND
         *     currentCharacterLocation = 0                              AND
         *     currentTokensStartLocation = 0                            AND
         *     lineNumber = 1                                            AND
         *   currentTokensLineNumber = 1
         */
        Input(const char* fileName);
        
        /**
         * Reads in the characters from the istream indicated and initializes
         * currentCharacterLocation, currentTokensStartLocation, and
         * currentTokensLineNumber.  Used primarily to be able to read from
         * the command line.
         *
         * Parameters: istream -- the input stream to be read from.
         * PreCondition: none
         *     characters = contents of istream with
         *        the EOF character appended to the end                  AND
         *     currentCharacterLocation = 0                              AND
         *     currentTokensStartLocation = 0                            AND
         *     lineNumber = 1                                            AND
         *     currentTokensLineNumber = 1
         */
        Input(istream& istream);

         /**
          * Copy Constructor.  Not really needed in this application since Input is a singleton but
          * c++ it is wise to always define a copy constructor so we don't get surprises.
          *
          * Parameters: input -- the input to be copied.
          * Precondition: none
          * Postcondition: this == input
          */
        Input(const Input& input);
        
    //Destructors
         /**
          * The destructor for the class Input.
          *
          *  Parameters: none
          *  PreCondition: none
          *  PostCondition:
          *       none of the attributes created on heap exist.
          */
        ~Input();
    
    //Queries
         /**
          * Tests to see if two inputs are equal.  Not needed in this application but wise to include
          * it always.
          *
          * Parameters: input -- the other input we are going to test for equality.
          * PreCondtion: none
          * PostCondition: result =
          *     characters = input.characters                                 AND
          *     currentCharacterLocation = input.currentCharacterLocation     AND
          *     currentTokensStartLocation = input.currentTokensStartLocation AND
          *     lineNumber = input.lineNumber                                 AND
          *     currentTokensLineNumber = input.currentTokensLineNumber 
          */
         bool operator==(const Input& input) const;

         /**
          * Returns a string representation of the Input.
          *
          * Output Syntax: Result::-                     Characters
          *                                              CurrentCharacterLocation
          *                                              CurrentTokensStartLocation
          *                                              LineNumber
          *                                              CurrentTokensLineNumber
          *                Characters::-                 "characters = " '"' string '"' '\n'
          *                CurrentCharacterLocation::-   "currentCharacterLocation = " int '\n'
          *                CurrentTokensStartLocation::- "currentTokensStartLocation = " int '\n'
          *                LineNumber::-                 "lineNumber = " int '\n'
          *                CurrentTokensLineNumber::-    "currentTokenLineNumber = " int '\n'
          * 
          * Parameters: none
          * PreCondition: none
          * PostCondition: result =
          *     "characters = " + '"' + charactrers.toString() + '"' + '\n'
          *     "currentCharacterLocation = " + currentCharacterLocation.toString() + '\n'
          *     "currentTokensStartLocation = " + currentTokensStartLocation.toString() + '\n'
          *     "lineNumber = " + LineNumber.toString() + '\n'
          *     "currentTokenLineNumber = " + currentTokensLineNumber.toString() + '\n'
          */
         string toString() const;

         /**
          * Returns the character at the currentCharacterLocation
          *
          * Parameters: none
          * PreCondition: currentCharacterLocation < |characters|
          * PostCondition: result = characters[currentCharacterLocation]
          */
         char getCurrentCharacter() const;
       
         /**
          * The getter for the currentTokensLineNumber.
          *
          * Parameters: none
          * PreCondition: none
          * PostCondition: result = currentTokensLineNumber
          */
         unsigned int getCurrentTokensLineNumber() const;
               
         /**
          * Returns true if there are no more characters, false otherwise
          *
          * Parameters: none
          * PreCondition: none
          * PostCondition: result = characters[currentCharacterLocation] = EOF
          */
         bool eof() const;
        
         /**
          * Returns the value of the current token from the
          * currentTokensStartLocation to currentCharacterLocation - 1 inclusive
          *
          * Parameters: none
          * PreCondition: none
          * PostCondition:
          *   result =
          *     characters[currentCharacterLocation..currentCharacterLocation-1]
          */
         string getTokensValue() const;

    //Commands

         /**
          * If not eof() and the current character is a '\n' then increment the
          * lineNumber by 1.  If not eof() increment the
          * currentCharacterLocation by 1.
          *
          * Parameters: none
          * PreCondition: none
          * PostCondition:
          *   NOT eof() AND characters[currentCharacterLocation'] != '\n' =>
          *       lineNumber = lineNumber' + 1                               AND
          *   NOT eof() =>
          *       currentCharacterLocation = currentCharacterLocation + 1
          */
         void advance();
        
         /**
          * Marks this the beginning of a new token.  Specifically, it sets the
          * location of the new tokens first character and records the new token's
          * line number.
          *
          * Parameters: none
          * PreCondition: none
          * PostCondition:
          *   currentTokensStartLocation = currentCharacterLocation AND
          *   currentTokenLineNumber = lineNumber
          */
         void mark();

    private:
        //Domain Implementation
            string       characters;
            unsigned int currentCharacterLocation;
            unsigned int currentTokensLineNumber;
            unsigned int currentTokensStartLocation;
            unsigned int lineNumber;

        //Auxiliary Methods
            //Code common to both constructors.
            void init(istream& in);
};
#endif

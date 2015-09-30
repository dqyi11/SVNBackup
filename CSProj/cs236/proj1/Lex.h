#ifndef Lex_h
#define Lex_h

#include "Token.h"
#include "Input.h"
#include "State.h"
#include <vector>
#include <istream>

using namespace std;

/**
 * The lexical analyzer for project 1 in CS 236.
 *
 * The alphabet for the Finite State Machine/Finite State Transducer/Lexical Analyzer is the ASCII alphabet union EOF
 *
 * Regular Expresion Syntax:                                      TokenTypes
 *
 *     ( :-?                                                  |   //COLON or COLON_DASH                
 *       ,                                                    |   //COMMA
 *       .                                                    |   //PERIOD
 *       <=?                                                  |   //LT or LE
 *       >=?                                                  |   //GT or GE
 *       =                                                    |   //EQ
 *       !=                                                   |   //NE
 *       (                                                    |   //LPAREN
 *       )                                                    |   //RPAREN
 *       \*                                                   |   //STAR
 *       \+                                                   |   //PLUS
 *       \?                                                   |   //Q_MARK
 *       [A-Za-z] [A-Za-z0-9]*                                |   //ID
 *       ' ([^']* ('')? | '')* '                              |   //STRING
 *       # ([^\nEOF]* (\n | EOF) | \| ([^\|]* (\|[^#])?)* \|# |  //INLINE_COMMENT OR MULTI_LINE_COMMENT
 *       [\s\t\n\v\f\r]                                          //WHITESPACE -- defined in cctype::isspace(int)
 *     )*
 *
 *     The keyWords: "Schemes", "Facts", "Rules", and "Queries" are created after evaluating at Ids.
 *
 *     See also: students.cs.byu.edu/~cs236ta/spring2012/project1/#tokenTypes
 *
 * The only part of the syntax we handle in this example is:
 *
 *     ( ,                                                    |   //COMMA
 *       .                                                    |   //PERIOD
 *       !=                                                   |   //NE
 *       [A-Za-z] [A-Za-z0-9]*                                |   //ID
 *     )*
 *
 *     Domain:
 *         tokens       : sequence<Token>
 *         currentToken : int
 */
class Lex {
    public:
    //Constructors
        /**
         * The default constructor.
         *
         * Parameters: none
         * PreCondition: none
         * PostCondition: |tokens| = 0 AND currentToken = 0
         */  
		Lex();

        /**
         * The constructor for the lexical analyzer.  If the characters in the specified iostream are syntatically
         * correct, it converts them into a sequence of tokens.
         *
         * Errors: input is not syntatically correct
         * Parameters: istream -- the iostream we will read characters from
         * PreCondition: istream is syntatically correct.
         * PostCondition: tokens contains sequence of tokens parsed from the input stream AND currentToken = 0.
         */
        Lex(istream& istream);

        /**
         * The constructor for the lexical analyzer. Does the same as the Lex(istream&) constructor after opening
         * the file identified by the file name as an istream. 
         *
         * Errors: file does not exist, file cannot be read, file is not a text file, and
         *         input is not syntatically correct
         * Parameters: fileName -- name of the file we will read from.
         * PreCondition: istream is syntatically correct.
         * PostCondition: tokens contains sequence of tokens parsed from the input stream AND currentToken = 0.
         */
        Lex(const char* fileName);

        /**
         * The copy constructor.
         *
         * Parameters: lex -- the lexical analyzer to be copied.
         * Precondition: none
         * Postcondition: this = lex
         */
        Lex(const Lex& lex);

    //Destructor
        /**
         * The destructor.
         *
         * Parameters: none
         * PreCondition: none
         * PostCondition: all memory for this has been reclaimed.
         */
        ~Lex();


    //Queries
        /**
         * The == operator for the lexical analyzer.
         *
         * Parameters: lex -- the lexical analyzer we are going to compare to.
         * PreCondition: none
         * PostCondition : result = tokens = lex.tokens AND currentToken = lex.currentToken
         */
        bool operator==(const Lex& lex);

        /** Returns the string representation of the lexical analyzer according to the syntax specified in
         *  students.cs.byu.edu/~cs236ta/spring2012/project1/#outputFormat
         *
         *  Parameters: none
         *  PreCondition: none
         *  PostCondition: result is the string representation of the lexical analyzer according to the syntax
         *  described in students.cs.byu.edu/~cs236ta/spring2012/project1/#outputFormat
         */
        std::string toString() const;

        /**
         * Returns the current Token.
         *
         * Parameters: none
         * PreCondition: hasNext()
         * PostCondition: result = tokens[currentToken] 
         */
        Token* getCurrentToken(); 

        /**
         * Returns true if there is another token, false otherwise.
         *
         * Parameters: none
         * PreCondition: none
         * PostCondition: result = currentToken < |tokens|
         */
        bool hasNext();

    //Commands
        /**
         * Increments the currentTokenValue
         *
         * Parameters: none
         * PreCondition: none
         * PostCondition: currentToken = currentToken' + 1
         */
        void advance(); 


    private:
        //Domain Implementation
            Input* input;
            std::vector<Token*>* tokens;
            int index;
            State state;

        //Auxiliary Methods
            void generateTokens(Input* input);
            State nextState();
            void storeToken(Token* token);
            State getNextState();
            void emit(TokenType tokenType);
};
#endif

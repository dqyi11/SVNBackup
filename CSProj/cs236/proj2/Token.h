#ifndef Token_h
#define Token_h

#include "TokenType.h"
/**
 * The lexical analyzer provides access to a list of tokens.  This defines what
 * a token is.
 *
 * DOMAIN:
 *     tokenType             : TokenType
 *     tokensValue           : string
 *     tokensLineNumber      : unsigned
 */
class Token {
    public:
    //Constructors
        /**
         * The copy constructor.
         *
         * Parameters: none
         * PreCondition: none
         * PostCondition: tokenType = NUL AND tokensValue = "" AND tokensLineNumber = 0
         */
        Token();

        /**
         * Creates a new Token from the newType, newValue and newTokensLineNumber
         *
         * Parameters: newType             -- the tokenType of the new token
         *             newValue            -- the value of the new token
         *             newTokensLineNumber -- the new token's line number
         * PreCondtion: none
         * PostCondition: tokenType = newType AND
         *                tokensValue = newValue AND
         *                tokensLineNumber = newTokensLineNumber
         */
        Token(TokenType newType, std::string newValue, unsigned newTokensLineNumber);

        /**
         * The copy constructor
         *
         * Parameters: token -- the token being copied
         * PreCondition: none
         * PostCondition: tokenType = token.tokenType AND
         *                tokensValue = token.tokensValue AND
         *                tokensLineNumber = token.tokensLineNumber
         */
        Token(const Token& token);

    //Destructors
        /**
         *The destructor for the Token class.  
         * 
         * Parameters: none
         * PreCondition: none
         * PostCondtion: true
         */
        ~Token();

    //Queries
        /**
         * The tokenType getter.
         *
         * Parameters: none
         * PreCondition: none
         * PostCondition: result = tokenType
         */
        TokenType   getTokenType() const;

        /**
         * The tokensLineNumber getter.
         *
         * Parameters: none
         * PreCondition: none
         * PostCondition: result = tokensLineNumber
         */
        unsigned    getLineNumber() const;

        /**
         * The tokensValue getter.
         *
         * Parameters: none
         * PreCondition: none
         * PostCondition: result = tokensValue
         */
        std::string getTokensValue() const;

        /**
         * The toString method for a Token.
         * 
         * Output syntax: "(" string ",\"" string "\"," int ")\n"
         *
         * Parameters: none
         * PreCondition: none
         * PostCondition: result =
         *     "(" + TokenTypeToString(tokenType) + ",\"" + tokenValue + "\"," + * itoa(tokensLineNumber) + ")"
         */
        std::string toString() const;

        /**
         * The equality operator for Tokens.
         *
         * Parameters: token -- the token we are going to compare against.
         * PreConditions: none
         * PostCondition: result = tokenType = token.tokenType AND
         *                         tokensValue = token.tokensValue AND
         *                         tokensLineNumber = token. tokensLineNumber
         */
        bool operator==(const Token& token);

    //Commands
        /**
         * The tokenType setter.
         *
         * Parameters: none
         * PreCondition: none
         * PostCondition: tokenType = newTokenType 
         */
        void setTokenType(const TokenType newTokenType);
        
    private:
        //Domain Implementation
            //The names used in the domain definition do not necessarily match the names used in the
            //implementation but they often do.
            TokenType    tokenType;
            unsigned     lineNumber;
            std::string  value;
};
#endif

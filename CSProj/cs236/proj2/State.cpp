#include "State.h"

using namespace std;

string StateToString(State tokenType){
    string result = "";
    switch(tokenType){
        case Comma:                      result = "Comma"; break;
        case Period:                     result = "Period"; break;
        case Q_Mark:                     result = "Q_Mark"; break;
        case Left_Paren:                 result = "Left_Paren"; break;
        case Right_Paren:                result = "Right_Paren"; break;
        case SawColon:                   result = "SawColon"; break;
        case Colon_Dash:                 result = "Colon_Dash"; break;
        case SawAQuote:                  result = "SawAQuote"; break;
        case Multiply:                   result = "Multiply"; break;
        case Add:                        result = "Add"; break;
        case ProcessingString:           result = "ProcessingString"; break;
        case PossibleEndOfString:        result = "PossibleEndOfString"; break;
        case ProcessingLineComment:      result = "ProcessingLineComment"; break;
        case PossibleEndOfLineComment:   result = "PossibleEndOfLineComment"; break;
        case ProcessingBlockComment:     result = "ProcessingBlockComment"; break;
        case PossibleEndOfBlockComment:  result = "PossibleEndOfBlockComment"; break;
        case EndOfBlockComment:          result = "EndOfBlockComment"; break;
        case ProcessingID:               result = "ProcessingID"; break;
        case Undefined:                  result = "Undefined"; break;
        case Whitespace:                 result = "Whitespace"; break;
        case Start:                      result = "Start"; break;
        case SawPound:                   result = "SawPound"; break;
        case End:                        result = "End"; break;
        default:                         result = "default";
    }
    return result;
};

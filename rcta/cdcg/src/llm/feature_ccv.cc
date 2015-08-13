/**
 * @file    feature_ccv.cc
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 * The implementation of a class that looks at the value of the continuous CV
 */

#include <sstream>

#include <h2sl/feature_ccv.h>

using namespace std;
using namespace h2sl;

Feature_CCV::
Feature_CCV( const double& cv ) : Feature( false ),
                                        _cv( cv ) {

}

Feature_CCV::
~Feature_CCV() {

}

Feature_CCV::
Feature_CCV( const Feature_CCV& other ) : Feature( other ),
                                        _cv( other._cv ){

}

Feature_CCV&
Feature_CCV::
operator=( const Feature_CCV& other ) {
  _invert = other._invert;
  _cv = other._cv;
  return (*this);
}

bool
Feature_CCV::
value( const double& cv,
        const Grounding* grounding,
        const vector< Grounding* >& children,
        const Phrase* phrase,   
        const World* world ){
  if( _invert ){
    return ( cv != _cv );
  } else {
    return ( cv == _cv );
  }
}

void
Feature_CCV::
to_xml( xmlDocPtr doc, xmlNodePtr root )const{
  xmlNodePtr node = xmlNewDocNode( doc, NULL, ( xmlChar* )( "feature_cv" ), NULL );
  stringstream invert_string;
  invert_string << _invert;
  xmlNewProp( node, ( const xmlChar* )( "invert" ), ( const xmlChar* )( invert_string.str().c_str() ) );
  stringstream cv_string;
  cv_string << _cv;
  xmlNewProp( node, ( const xmlChar* )( "cv" ), ( const xmlChar* )( cv_string.str().c_str() ) );
  xmlAddChild( root, node );
  return;
}

void
Feature_CCV::
from_xml( xmlNodePtr root ){
  _invert = false;
  _cv = 0;
  if( root->type == XML_ELEMENT_NODE ){
    xmlChar * tmp = xmlGetProp( root, ( const xmlChar* )( "invert" ) );
    if( tmp != NULL ){
      string invert_string = ( char* )( tmp );
      _invert = ( bool )( strtol( invert_string.c_str(), NULL, 10 ) );
      xmlFree( tmp );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "cv" ) );
    if( tmp != NULL ){
      string cv_string = ( char* )( tmp );
      _cv = ( double )( strtod( cv_string.c_str(), NULL ) );
      xmlFree( tmp );
    }
  }
  return;
}

namespace h2sl {
  ostream&
  operator<<( ostream& out,
              const Feature_CCV& other ) {
    return out;
  }

}

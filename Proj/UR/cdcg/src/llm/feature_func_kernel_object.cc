/**
 * @file    feature_func_kernel_object.cc
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 * 
 * @section DESCRIPTION
 *
 * The implementation of a class used to check for a match with a function kernel's object
 */

#include <sstream>

#include "h2sl/region.h"
#include "h2sl/feature_func_kernel_object.h"

using namespace std;
using namespace h2sl;

Feature_Func_Kernel_Object::
Feature_Func_Kernel_Object( const bool& invert,
                        const unsigned int& objectType ) : Feature( invert ),
                                                            _object_type( objectType ) {

}

Feature_Func_Kernel_Object::
~Feature_Func_Kernel_Object() {

}

Feature_Func_Kernel_Object::
Feature_Func_Kernel_Object( const Feature_Func_Kernel_Object& other ) : Feature( other ),
                                                              _object_type( other._object_type ){

}

Feature_Func_Kernel_Object&
Feature_Func_Kernel_Object::
operator=( const Feature_Func_Kernel_Object& other ) {
  _invert = other._invert;
  _object_type = other._object_type;
  return (*this);
}

bool
Feature_Func_Kernel_Object::
value( const unsigned int& cv,
        const Grounding* grounding,
        const vector< Grounding* >& children,
        const Phrase* phrase,
        const World* world ){
  const Region * region = dynamic_cast< const Region* >( grounding );
  if( region != NULL ){
    if( region->object().type() == _object_type ){
      return !_invert;
    }
  }
  return false;
}

void
Feature_Func_Kernel_Object::
to_xml( xmlDocPtr doc, xmlNodePtr root )const{
  xmlNodePtr node = xmlNewDocNode( doc, NULL, ( xmlChar* )( "feature_func_kernel_object" ), NULL );
  stringstream invert_string;
  invert_string << _invert;
  xmlNewProp( node, ( const xmlChar* )( "invert" ), ( const xmlChar* )( invert_string.str().c_str() ) );
  stringstream object_type_string;
  object_type_string << _object_type;
  xmlNewProp( node, ( const xmlChar* )( "object_type" ), ( const xmlChar* )( object_type_string.str().c_str() ) );
  xmlAddChild( root, node );
  return;
}

void
Feature_Func_Kernel_Object::
from_xml( xmlNodePtr root ){
  _invert = false;
  _object_type = 0;
  if( root->type == XML_ELEMENT_NODE ){
    xmlChar * tmp = xmlGetProp( root, ( const xmlChar* )( "invert" ) );
    if( tmp != NULL ){
      string invert_string = ( char* )( tmp );
      _invert = ( bool )( strtol( invert_string.c_str(), NULL, 10 ) );
      xmlFree( tmp );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "object_type" ) );
    if( tmp != NULL ){
      string object_type_string = ( char* )( tmp );
      _object_type = strtol( object_type_string.c_str(), NULL, 10 );
      xmlFree( tmp );
    }
  }
  return;
}

namespace h2sl {
  ostream&
  operator<<( ostream& out,
              const Feature_Func_Kernel_Object& other ) {
    return out;
  }
}

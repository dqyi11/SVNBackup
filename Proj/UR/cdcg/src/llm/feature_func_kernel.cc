/**
 * @file    feature_func_kernel.cc
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 * 
 * @section DESCRIPTION
 *
 * The implementation of a class used to check for a match with a function kernel's object
 */

#include <sstream>

#include "h2sl_cdcg/func_kernel.h"
#include "h2sl_cdcg/feature_func_kernel.h"

using namespace std;
using namespace h2sl;
using namespace h2sl_cdcg;

Feature_Func_Kernel::
Feature_Func_Kernel( const bool& invert,
                        const unsigned int& kernelType ) : Feature( invert ),
                                                            _kernel_type( kernelType ) {

}

Feature_Func_Kernel::
~Feature_Func_Kernel() {

}

Feature_Func_Kernel::
Feature_Func_Kernel( const Feature_Func_Kernel& other ) : Feature( other ),
                                                              _kernel_type( other._kernel_type ){

}

Feature_Func_Kernel&
Feature_Func_Kernel::
operator=( const Feature_Func_Kernel& other ) {
  _invert = other._invert;
  _kernel_type = other._kernel_type;
  return (*this);
}

bool
Feature_Func_Kernel::
value( const unsigned int& cv,
        const Grounding* grounding,
        const vector< Grounding* >& children,
        const Phrase* phrase,
        const World* world ){
  const Func_Kernel * func_kernel = dynamic_cast< const Func_Kernel* >( grounding );
  if( func_kernel != NULL ){
    if( func_kernel->type() == _kernel_type ){
      return !_invert;
    }
  }
  return false;
}

void
Feature_Func_Kernel::
to_xml( xmlDocPtr doc, xmlNodePtr root )const{
  xmlNodePtr node = xmlNewDocNode( doc, NULL, ( xmlChar* )( "feature_func_kernel" ), NULL );
  stringstream invert_string;
  invert_string << _invert;
  xmlNewProp( node, ( const xmlChar* )( "invert" ), ( const xmlChar* )( invert_string.str().c_str() ) );
  stringstream kernel_type_string;
  kernel_type_string << _kernel_type;
  xmlNewProp( node, ( const xmlChar* )( "kernel_type" ), ( const xmlChar* )( kernel_type_string.str().c_str() ) );
  stringstream weight_string;
  weight_string << _weight;
  xmlNewProp( node, ( const xmlChar* )( "weight" ), ( const xmlChar* )( weight_string.str().c_str() ) );
  xmlAddChild( root, node );
  return;
}

void
Feature_Func_Kernel::
from_xml( xmlNodePtr root ){
  _invert = false;
  _kernel_type = 0;
  if( root->type == XML_ELEMENT_NODE ){
    xmlChar * tmp = xmlGetProp( root, ( const xmlChar* )( "invert" ) );
    if( tmp != NULL ){
      string invert_string = ( char* )( tmp );
      _invert = ( bool )( strtol( invert_string.c_str(), NULL, 10 ) );
      xmlFree( tmp );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "kernel_type" ) );
    if( tmp != NULL ){
      string kernel_type_string = ( char* )( tmp );
      _kernel_type = strtol( kernel_type_string.c_str(), NULL, 10 );
      xmlFree( tmp );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "weight" ) );
    if( tmp != NULL ){
      string weight_string = ( char* )( tmp );
      _weight = strtof( weight_string.c_str(), NULL );
    }
  }
  return;
}

namespace h2sl_cdcg {
  ostream&
  operator<<( ostream& out,
              const Feature_Func_Kernel& other ) {
    return out;
  }
}

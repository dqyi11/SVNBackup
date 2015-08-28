/**
 * @file    feature_func_kernel_matches_child_region.cc
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 *          Matthew R. Walter (mwalter@csail.mit.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The implementation of a class used to check for a func kernel match with a child
 */

#include <sstream>

#include "h2sl/object.h"
#include "h2sl/region.h"
#include "h2sl_cdcg/func_kernel.h"
#include "h2sl_cdcg/feature_func_kernel_matches_child_region.h"

using namespace std;
using namespace h2sl;
using namespace h2sl_cdcg;

Feature_Func_Kernel_Matches_Child_Region::
Feature_Func_Kernel_Matches_Child_Region( const bool& invert ) : h2sl::Feature( invert ) {

}

Feature_Func_Kernel_Matches_Child_Region::
~Feature_Func_Kernel_Matches_Child_Region() {

}

Feature_Func_Kernel_Matches_Child_Region::
Feature_Func_Kernel_Matches_Child_Region( const Feature_Func_Kernel_Matches_Child_Region& other ) : h2sl::Feature( other ) {

}

Feature_Func_Kernel_Matches_Child_Region&
Feature_Func_Kernel_Matches_Child_Region::
operator=( const Feature_Func_Kernel_Matches_Child_Region& other ) {
  _invert = other._invert;
  return (*this);
}

bool
Feature_Func_Kernel_Matches_Child_Region::
value( const unsigned int& cv,
        const h2sl::Grounding* grounding,
        const vector< h2sl::Grounding* >& children,
        const h2sl::Phrase* phrase,
        const h2sl::World* world ){
  const Func_Kernel * func_kernel = dynamic_cast< const Func_Kernel* >( grounding );
  if( func_kernel != NULL ){
    for( unsigned int i = 0; i < children.size(); i++ ){
      const Region* child = dynamic_cast< const Region* >( children[ i ] );
      if( child != NULL ){
        if( child->object() == func_kernel->object() ){
          return !_invert;
        }
      }
    }   
    return _invert;
  }
  return false;
}

void
Feature_Func_Kernel_Matches_Child_Region::
to_xml( xmlDocPtr doc, xmlNodePtr root )const{
  xmlNodePtr node = xmlNewDocNode( doc, NULL, ( xmlChar* )( "feature_func_kernel_matches_child_region" ), NULL );
  stringstream invert_string;
  invert_string << _invert;
  xmlNewProp( node, ( const xmlChar* )( "invert" ), ( const xmlChar* )( invert_string.str().c_str() ) );
  xmlAddChild( root, node );
  return;
}

void
Feature_Func_Kernel_Matches_Child_Region::
from_xml( xmlNodePtr root ){
  _invert = false;
  if( root->type == XML_ELEMENT_NODE ){
    xmlChar * tmp = xmlGetProp( root, ( const xmlChar* )( "invert" ) );
    if( tmp != NULL ){
      string invert_string = ( char* )( tmp );
      _invert = ( bool )( strtol( invert_string.c_str(), NULL, 10 ) );
      xmlFree( tmp );
    }
  }
  return;
}

namespace h2sl_cdcg {
  ostream&
  operator<<( ostream& out,
              const Feature_Func_Kernel_Matches_Child_Region& other ) {
    return out;
  }

}

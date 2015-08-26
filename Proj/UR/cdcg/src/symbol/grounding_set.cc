/**
 * @file    grounding_set.cc
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The implementation of a class used to describe a set of groundings
 */

#include "h2sl/object.h"
#include "h2sl/region.h"
#include "h2sl/constraint.h"
#include "h2sl_cdcg/func_kernel.h"

#include "h2sl_cdcg/grounding_set.h"

using namespace std;
using namespace h2sl_cdcg;

Grounding_Set::
Grounding_Set( const vector< Grounding* >& groundings ) : h2sl::Grounding_Set( groundings ) {

}

Grounding_Set::
~Grounding_Set() {
  clear();
}

Grounding_Set::
Grounding_Set( const Grounding_Set& other ) : h2sl::Grounding_Set( other ) {
}

Grounding_Set&
Grounding_Set::
operator=( const Grounding_Set& other ) {
  clear();
  _groundings.resize( other._groundings.size(), NULL );
  for( unsigned int i = 0; i < other._groundings.size(); i++ ){
    _groundings[ i ] = other._groundings[ i ]->dup();
  }
  return (*this);
}

void
Grounding_Set::
to_xml( const string& filename )const{
  xmlDocPtr doc = xmlNewDoc( ( xmlChar* )( "1.0" ) );
  xmlNodePtr root = xmlNewDocNode( doc, NULL, ( xmlChar* )( "root" ), NULL );
  xmlDocSetRootElement( doc, root );
  to_xml( doc, root );
  xmlSaveFormatFileEnc( filename.c_str(), doc, "UTF-8", 1 );
  xmlFreeDoc( doc );
  return;
}

void
Grounding_Set::
to_xml( xmlDocPtr doc,
        xmlNodePtr root )const{
  xmlNodePtr node = xmlNewDocNode( doc, NULL, ( const xmlChar* )( "grounding_set" ), NULL );
  for( unsigned int i = 0; i < _groundings.size(); i++ ){
    if( _groundings[ i ] != NULL ){
      _groundings[ i ]->to_xml( doc, node );
    }
  }
  xmlAddChild( root, node );
  return;
}

void
Grounding_Set::
from_xml( const string& filename ){
  xmlDoc * doc = NULL;
  xmlNodePtr root = NULL;
  doc = xmlReadFile( filename.c_str(), NULL, 0 );
  if( doc != NULL ){
    root = xmlDocGetRootElement( doc );
    if( root->type == XML_ELEMENT_NODE ){
      xmlNodePtr l1 = NULL;
      for( l1 = root->children; l1; l1 = l1->next ){
        if( l1->type == XML_ELEMENT_NODE ){
          if( xmlStrcmp( l1->name, ( const xmlChar* )( "grounding_set" ) ) == 0 ){
            from_xml( l1 );
          }
        }
      }
    }
    xmlFreeDoc( doc );
  }
  return;
}

void
Grounding_Set::
from_xml( xmlNodePtr root ){
  clear();
  if( root->type == XML_ELEMENT_NODE ){
    xmlNodePtr l1 = NULL;
    for( l1 = root->children; l1; l1 = l1->next ){
      if( l1->type == XML_ELEMENT_NODE ){
        if( xmlStrcmp( l1->name, ( const xmlChar* )( "object" ) ) == 0 ){
          cout << "Find grounding of object " << endl;
          _groundings.push_back( new h2sl::Object() );
          _groundings.back()->from_xml( l1 );
        } else if ( xmlStrcmp( l1->name, ( const xmlChar* )( "region" ) ) == 0 ){
          cout << "Find grounding of region " << endl;
          _groundings.push_back( new h2sl::Region() );
          _groundings.back()->from_xml( l1 );
        } else if ( xmlStrcmp( l1->name, ( const xmlChar* )( "constraint" ) ) == 0 ){
          cout << "Find grounding of constraint " << endl;
          _groundings.push_back( new h2sl::Constraint() );
          _groundings.back()->from_xml( l1 );
        } else if ( xmlStrcmp( l1->name, ( const xmlChar* )( "func_kernel" ) ) == 0 ){
          cout << "Find grounding of func_kernel " << endl;
          _groundings.push_back( new Func_Kernel() );
          _groundings.back()->from_xml( l1 );
        } 
      }
    }
  }
  return;
}

namespace h2sl_cdcg {
  ostream&
  operator<<( ostream& out,
              const Grounding_Set& other ) {
    for( unsigned int i = 0; i < other.groundings().size(); i++ ){
      if( other.groundings()[ i ] != NULL ){
        out << *other.groundings()[ i ];
      }
      if( i != ( other.groundings().size() - 1 ) ){
        out << ",";
      } 
    }
    return out;
  }
}

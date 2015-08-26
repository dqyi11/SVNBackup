/**
 * @file    func_kernel.cc
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The implementation of a class used to describe a kernel of an object
 */

#include <stdlib.h>
#include <sstream>
#include "h2sl_cdcg/func_kernel.h"

using namespace std;
using namespace h2sl;
using namespace h2sl_cdcg;

Func_Kernel::
Func_Kernel( const unsigned int& type,
        const float& weight,
        const h2sl::Object& object ) : h2sl::Grounding(),
                                  _type( type ),
                                  _weight( weight ),
                                  _object( object ) {

}

Func_Kernel::
Func_Kernel( const func_kernel_type_t& type,
        const float& weight,
        const h2sl::Object& object ) : h2sl::Grounding(),
                                  _type( type ),
                                  _weight( weight ),
                                  _object( object ){

}

Func_Kernel::
~Func_Kernel() {

}

Func_Kernel::
Func_Kernel( const Func_Kernel& other ) : h2sl::Grounding( other ),
                                _type( other._type ),
                                _weight( other._weight ),
                                _object( other._object ){

}

Func_Kernel&
Func_Kernel::
operator=( const Func_Kernel& other ) {
  _type = other._type;
  _object = other._object;
  _weight = other._weight;
  return (*this);
}

bool
Func_Kernel::
operator==( const Func_Kernel& other )const{
  if( _type != other._type ){
    return false;
  } if( _object != other._object ){
    return false;
  } else {
    return true;
  }
}

bool
Func_Kernel::
operator!=( const Func_Kernel& other )const{
  return !( *this == other );
}

Grounding*
Func_Kernel::
dup( void )const{
  return new Func_Kernel( *this );
}

string
Func_Kernel::
type_to_std_string( const unsigned int& type ){
  switch( type ){
  case( FUNC_KERNEL_TYPE_GAUSSIAN ):
    return "gaussian";
    break;
  case( FUNC_KERNEL_TYPE_UNKNOWN ):
  default:
    return "na";
  }
}

unsigned int
Func_Kernel::
type_from_std_string( const string& type ){
  for( unsigned int i = 0; i < NUM_OBJECT_TYPES; i++ ){
    if( type == Func_Kernel::type_to_std_string( i ) ){
      return i;
    }
  }
  return OBJECT_TYPE_UNKNOWN;
}

void
Func_Kernel::
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
Func_Kernel::
to_xml( xmlDocPtr doc,
        xmlNodePtr root )const{
  xmlNodePtr node = xmlNewDocNode( doc, NULL, ( const xmlChar* )( "func_kernel" ), NULL );
  xmlNewProp( node, ( const xmlChar* )( "type" ), ( const xmlChar* )( Func_Kernel::type_to_std_string( _type ).c_str() ) );
  stringstream weight_str;
  weight_str << _weight;
  xmlNewProp( node, ( const xmlChar* )( "weight" ), ( const xmlChar* )( weight_str.str().c_str() ) );
  _object.to_xml( doc, node );
  xmlAddChild( root, node );
  return;
}

void
Func_Kernel::
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
          if( xmlStrcmp( l1->name, ( const xmlChar* )( "func_kernel" ) ) == 0 ){
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
Func_Kernel::
from_xml( xmlNodePtr root ){
  _type = FUNC_KERNEL_TYPE_UNKNOWN;
  _object = Object();
  if( root->type == XML_ELEMENT_NODE ){
    xmlChar * tmp = xmlGetProp( root, ( const xmlChar* )( "type" ) );
    if( tmp != NULL ){
      string type_string = ( char* )( tmp );
      _type = Func_Kernel::type_from_std_string( type_string );
      xmlFree( tmp );
    }
    tmp = xmlGetProp( root, ( const xmlChar* )( "weight" ) );
    if( tmp != NULL ){
      string weight_string = ( char* )( tmp );
      _weight = strtof( weight_string.c_str() , NULL ); 
      xmlFree( tmp );
    }
    xmlNodePtr l1 = NULL; 
    for( l1 = root->children; l1; l1 = l1->next ){
      if( l1->type == XML_ELEMENT_NODE ){
        if( xmlStrcmp( l1->name, ( const xmlChar* )( "object" ) ) == 0 ){
          _object.from_xml( l1 );
        }
      }
    }
  }
  return;
}

namespace h2sl_cdcg {
  ostream&
  operator<<( ostream& out,
              const Func_Kernel& other ) {
    out << "Func_Kernel(";
    out << "type=\"" << Func_Kernel::type_to_std_string( other.type() ) << "\",";
    out << "object=" << other.object();
    out << " weight=" << other.weight();
    out << " resolution=" << other.resolution();
    out << ")";
    return out;
  }
}

/**
 * @file    phrase.cc
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 *          Matthew R. Walter (mwalter@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * This file is part of h2sl.
 *
 * Copyright (C) 2014 by the Massachusetts Institute of Technology
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see
 * <http://www.gnu.org/licenses/gpl-2.0.html> or write to the Free
 * Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * @section DESCRIPTION
 *
 * The implementation of a class used to represent a phrase
 */

#include <fstream>
#include <sstream>

#include <h2sl_cdcg/phrase.h>
#include <h2sl_cdcg/grounding_set.h>

using namespace std;
using namespace h2sl_cdcg;

Phrase::
Phrase( const h2sl::phrase_type_t& type,
        const string& text,
        const vector< h2sl::Word >& words,
        const vector< Phrase* >& children,
        h2sl::Grounding* grounding ) {

  _type = type;
  _text = text;
  _words = words;
  _grounding = grounding;

  for( unsigned int i=0; i < children.size(); i++ ) {
    Phrase* phrase = children[i]; 
    _children.push_back( phrase ); 
  }
}

Phrase::
~Phrase() {

}

Phrase::
Phrase( const Phrase& other ) { 
  _type = other._type;
  _text = other._text;
  _words = other._words;
  _grounding = NULL;

  for( unsigned int i = 0; i < other._children.size(); i++ ){
    _children.push_back( other._children[ i ]->dup() );
  }
  if( other._grounding != NULL ){
    _grounding = other._grounding->dup();
  }
}

Phrase&
Phrase::
operator=( const Phrase& other ) {
  _type = other._type;
  _text = other._text;
  _words = other._words;
  for( unsigned int i = 0; i < _children.size(); i++ ){
    if( _children[ i ] != NULL ){
      delete _children[ i ];
      _children[ i ] = NULL;
    }
  }
  for( unsigned int i = 0; i < other._children.size(); i++ ){
    _children.push_back( other._children[ i ]->dup() );
  }
  if( other._grounding != NULL ){
    _grounding = other._grounding->dup();
  } else {
    _grounding = NULL;  
  }
  return (*this);
}

bool
Phrase::
operator==( const Phrase& other )const{
  if( _type != other._type ){
    return false;
  } else if ( _text != other._text ){
    return false;
  } else if ( _words.size() != other._words.size() ){
    return false;
  } else if ( _children.size() != other._children.size() ){
    return false;
  } else {
    for( unsigned int i = 0; i < _words.size(); i++ ){
      if( _words[ i ] != other._words[ i ] ){
        return false;
      }
    }   
    for( unsigned int i = 0; i < _children.size(); i++ ){
      if( *_children[ i ] != *other._children[ i ] ){
        return false;
      }
    }
    return true;
  }
}

bool
Phrase::
operator!=( const Phrase& other )const{
  return !( *this == other );
}

Phrase*
Phrase::
dup( void )const{
  return new Phrase( *this );
}

Phrase*
Phrase::
dup( const bool& empty )const{
  if( empty ){
    return new Phrase();
  } else {
    return new Phrase( *this );
  }
}

void
Phrase::
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
Phrase::
to_xml( xmlDocPtr doc,
        xmlNodePtr root )const{
  xmlNodePtr node = xmlNewDocNode( doc, NULL, ( const xmlChar* )( phrase_type_t_to_std_string( _type ).c_str() ), NULL );
  xmlNodePtr grounding_node = xmlNewDocNode( doc, NULL, ( const xmlChar* )( "grounding" ), NULL );
  if( _grounding != NULL ){
    _grounding->to_xml( doc, grounding_node );    
  } 
  xmlAddChild( node, grounding_node );
  for( unsigned int i = 0; i < _words.size(); i++ ){
    _words[ i ].to_xml( doc, node );
  }   
  for( unsigned int i = 0; i < _children.size(); i++ ){
    if( _children[ i ] != NULL ){
      _children[ i ]->to_xml( doc, node );
    }
  }
  xmlAddChild( root, node );
  return;
}

void 
Phrase::
from_xml( const std::string& filename ){
  xmlDoc * doc = NULL;
  xmlNodePtr root = NULL;
  doc = xmlReadFile( filename.c_str(), NULL, 0 );
  if( doc != NULL ){
    root = xmlDocGetRootElement( doc );
    if( root->type == XML_ELEMENT_NODE ){
      xmlNodePtr l1 = NULL;
      for( l1 = root->children; l1; l1 = l1->next ){
        if( l1->type == XML_ELEMENT_NODE ){
          for( unsigned int i = 0; i < h2sl::NUM_PHRASE_TYPES; i++ ){
            if( xmlStrcmp( l1->name, ( const xmlChar* )( h2sl::Phrase::phrase_type_t_to_std_string( ( h2sl::phrase_type_t )( i ) ).c_str() ) ) == 0 ){
              from_xml( l1 );
            }
          }
        }
      }
    }
    xmlFreeDoc( doc );
  }
  return;
}

void 
Phrase::
from_xml( xmlNodePtr root ){
  for( unsigned int i = 0; i < _children.size(); i++ ){
    if( _children[ i ] != NULL ){
      delete _children[ i ];
      _children[ i ] = NULL;
    }
  }
  _children.clear();
  if( root->type == XML_ELEMENT_NODE ){
    for( unsigned int i = 0; i < h2sl::NUM_PHRASE_TYPES; i++ ){
      if( xmlStrcmp( root->name, ( const xmlChar* )( phrase_type_t_to_std_string( ( h2sl::phrase_type_t )( i ) ).c_str() ) ) == 0 ){
        _type = ( h2sl::phrase_type_t )( i );
      }
    }
    xmlNodePtr l1 = NULL;
    for( l1 = root->children; l1; l1 = l1->next ){
      if( l1->type == XML_ELEMENT_NODE ){
        if( xmlStrcmp( l1->name, ( const xmlChar* )( "grounding" ) ) == 0 ){
          xmlNodePtr l2 = NULL;
          for( l2 = l1->children; l2; l2 = l2->next ){
            if( l2->type == XML_ELEMENT_NODE ){
              if( xmlStrcmp( l2->name, ( const xmlChar* )( "grounding_set" ) ) == 0 ){
                _grounding = new h2sl_cdcg::Grounding_Set();
                _grounding->from_xml( l2 );
              }
            }
          }
        }
        for( unsigned int i = 0; i < h2sl::NUM_POS_TAGS; i++ ){
          if( xmlStrcmp( l1->name, ( const xmlChar* )( h2sl::pos_t_to_std_string( ( h2sl::pos_t )( i ) ).c_str() ) ) == 0 ){
            _words.push_back( h2sl::Word() );
            _words.back().from_xml( l1 );
          }
        }
        for( unsigned int i = 0; i < h2sl::NUM_PHRASE_TYPES; i++ ){
          if( xmlStrcmp( l1->name, ( const xmlChar* )( h2sl::Phrase::phrase_type_t_to_std_string( ( h2sl::phrase_type_t )( i ) ).c_str() ) ) == 0 ){
            _children.push_back( new Phrase() );
            _children.back()->from_xml( l1 );
          }
        }
      }
    }
  }
  return;
}

namespace h2sl_cdcg {
  ostream&
  operator<<( ostream& out,
              const Phrase& other ) {
    out << Phrase::phrase_type_t_to_std_string( other.type() );
    for( unsigned int i = 0; i < other.words().size(); i++ ){
      out << "(" << other.words()[ i ] << ")";
      if( i != ( other.words().size() - 1 ) ){
        out << ",";
      }
    }
    for( unsigned int i = 0; i < other.children().size(); i++ ){
      out << "{" << *other.children()[ i ] << "}";
      if( i != ( other.children().size() - 1 ) ){
        out << ",";
      }
    }
    out << " ";
    if( other.grounding() != NULL ){
      out << "grounding:{" << *other.grounding() << "}";
    } else {
      out << "grounding:{NULL}";
    }
    return out;
  }
}

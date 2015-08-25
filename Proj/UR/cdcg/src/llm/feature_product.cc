/**
 * @file    feature_product.cc
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The implementation of a class used to describe a product of features
 */

#include "h2sl/feature_word.h"
#include "h2sl/feature_num_words.h"
#include "h2sl/feature_cv.h"
#include "h2sl/feature_object.h"
#include "h2sl/feature_region_object.h"
#include "h2sl/feature_region.h"
#include "h2sl/feature_constraint.h"
#include "h2sl/feature_region_object_matches_child.h"
#include "h2sl/feature_region_matches_child.h"
#include "h2sl/feature_region_merge_partially_known_regions.h"
#include "h2sl/feature_constraint_parent_matches_child_region.h"
#include "h2sl/feature_constraint_child_matches_child_region.h"
#include "h2sl/feature_constraint_parent_is_robot.h"
#include "h2sl/feature_constraint_child_is_robot.h"
#include "h2sl_cdcg/feature_func_kernel.h"
#include "h2sl_cdcg/feature_product.h"
#include "h2sl_cdcg/feature_func_kernel_matches_child.h"

using namespace std;
using namespace h2sl_cdcg;

Feature_Product::
Feature_Product() : h2sl::Feature_Product() {

}

Feature_Product::
~Feature_Product() {

}

Feature_Product::
Feature_Product( const Feature_Product& other ) : h2sl::Feature_Product( other ) {

}

Feature_Product&
Feature_Product::
operator=( const Feature_Product& other ) {
  _feature_groups = other._feature_groups;
  _values = other._values;
  return (*this);
}

void 
Feature_Product::
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
Feature_Product::
to_xml( xmlDocPtr doc, 
        xmlNodePtr root )const{
  xmlNodePtr node = xmlNewDocNode( doc, NULL, ( xmlChar* )( "feature_product" ), NULL );
  for( unsigned int i = 0; i < _feature_groups.size(); i++ ){
    xmlNodePtr feature_group_node = xmlNewDocNode( doc, NULL, ( const xmlChar* )( "feature_group" ), NULL );
    for( unsigned int j = 0; j < _feature_groups[ i ].size(); j++ ){
      _feature_groups[ i ][ j ]->to_xml( doc, feature_group_node );
    }
    xmlAddChild( node, feature_group_node );
  }
  xmlAddChild( root, node );
  return;
}

void 
Feature_Product::
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
          if( xmlStrcmp( l1->name, ( const xmlChar* )( "feature_product" ) ) == 0 ){
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
Feature_Product::
from_xml( xmlNodePtr root ){
  _values.clear();

  for( unsigned int i = 0; i < _feature_groups.size(); i++ ){
    for( unsigned int j = 0; j < _feature_groups[ i ].size(); j++ ){
      if( _feature_groups[ i ][ j ] != NULL ){
        delete _feature_groups[ i ][ j ];
        _feature_groups[ i ][ j ] = NULL;
      }
    }
    _feature_groups[ i ].clear();
  }
  _feature_groups.clear();

  if( root->type == XML_ELEMENT_NODE ){
    xmlNodePtr l1 = NULL;
    for( l1 = root->children; l1; l1 = l1->next ){
      if( l1->type == XML_ELEMENT_NODE ){
        if( xmlStrcmp( l1->name, ( const xmlChar* )( "feature_group" ) ) == 0 ){
          _feature_groups.push_back( vector< h2sl::Feature* >() );
          xmlNodePtr l2 = NULL;
          for( l2 = l1->children; l2; l2 = l2->next ){
            if( l2->type == XML_ELEMENT_NODE ){
              if( xmlStrcmp( l2->name, ( const xmlChar* )( "feature_cv" ) ) == 0 ){
                _feature_groups.back().push_back( new h2sl::Feature_CV() );
                _feature_groups.back().back()->from_xml( l2 );
              } else if ( xmlStrcmp( l2->name, ( const xmlChar* )( "feature_word" ) ) == 0 ){
                _feature_groups.back().push_back( new h2sl::Feature_Word() );
                _feature_groups.back().back()->from_xml( l2 );
              } else if ( xmlStrcmp( l2->name, ( const xmlChar* )( "feature_num_words" ) ) == 0 ){
                _feature_groups.back().push_back( new h2sl::Feature_Num_Words() );
                _feature_groups.back().back()->from_xml( l2 );
              } else if ( xmlStrcmp( l2->name, ( const xmlChar* )( "feature_object" ) ) == 0 ){
                _feature_groups.back().push_back( new h2sl::Feature_Object() );
                _feature_groups.back().back()->from_xml( l2 );
              } else if ( xmlStrcmp( l2->name, ( const xmlChar* )( "feature_region_object" ) ) == 0 ){
                _feature_groups.back().push_back( new h2sl::Feature_Region_Object() );
                _feature_groups.back().back()->from_xml( l2 );
              } else if ( xmlStrcmp( l2->name, ( const xmlChar* )( "feature_region" ) ) == 0 ){
                _feature_groups.back().push_back( new h2sl::Feature_Region() );
                _feature_groups.back().back()->from_xml( l2 );
              } else if ( xmlStrcmp( l2->name, ( const xmlChar* )( "feature_constraint" ) ) == 0 ){
                _feature_groups.back().push_back( new h2sl::Feature_Constraint() );
                _feature_groups.back().back()->from_xml( l2 );
              } else if ( xmlStrcmp( l2->name, ( const xmlChar* )( "feature_region_object_matches_child" ) ) == 0 ){
                _feature_groups.back().push_back( new h2sl::Feature_Region_Object_Matches_Child() );
                _feature_groups.back().back()->from_xml( l2 );
              } else if ( xmlStrcmp( l2->name, ( const xmlChar* )( "feature_region_matches_child" ) ) == 0 ){
                _feature_groups.back().push_back( new h2sl::Feature_Region_Matches_Child() );
                _feature_groups.back().back()->from_xml( l2 );
              } else if ( xmlStrcmp( l2->name, ( const xmlChar* )( "feature_region_merge_partially_known_regions" ) ) == 0 ){
                _feature_groups.back().push_back( new h2sl::Feature_Region_Merge_Partially_Known_Regions() );
                _feature_groups.back().back()->from_xml( l2 );
              } else if ( xmlStrcmp( l2->name, ( const xmlChar* )( "feature_constraint_parent_matches_child_region" ) ) == 0 ){
                _feature_groups.back().push_back( new h2sl::Feature_Constraint_Parent_Matches_Child_Region() );
                _feature_groups.back().back()->from_xml( l2 );
              } else if ( xmlStrcmp( l2->name, ( const xmlChar* )( "feature_constraint_child_matches_child_region" ) ) == 0 ){
                _feature_groups.back().push_back( new h2sl::Feature_Constraint_Child_Matches_Child_Region() );
                _feature_groups.back().back()->from_xml( l2 );
              } else if ( xmlStrcmp( l2->name, ( const xmlChar* )( "feature_constraint_parent_is_robot" ) ) == 0 ){
                _feature_groups.back().push_back( new h2sl::Feature_Constraint_Parent_Is_Robot() );
                _feature_groups.back().back()->from_xml( l2 );
              } else if ( xmlStrcmp( l2->name, ( const xmlChar* )( "feature_constraint_child_is_robot" ) ) == 0 ){
                _feature_groups.back().push_back( new h2sl::Feature_Constraint_Child_Is_Robot() );
                _feature_groups.back().back()->from_xml( l2 );
              } else if ( xmlStrcmp( l2->name, ( const xmlChar* )( "feature_func_kernel" ) ) == 0 ){
                _feature_groups.back().push_back( new Feature_Func_Kernel() );
                _feature_groups.back().back()->from_xml( l2 ); 
              } else if ( xmlStrcmp( l2->name, ( const xmlChar* )( "feature_func_kernel_matches_child" ) ) == 0 ){
                _feature_groups.back().push_back( new Feature_Func_Kernel_Matches_Child() );
                _feature_groups.back().back()->from_xml( l2 ); 
              } 
              else {
                cout << "could not load feature " << l2->name << endl;
              } 
            }
          }
        }
      }
    }
  }
  _values.resize( _feature_groups.size() );
  for( unsigned int i = 0; i < _feature_groups.size(); i++ ){
    _values[ i ].resize( _feature_groups[ i ].size() );
  }
  return;
}

namespace h2sl_cdcg {
  ostream&
  operator<<( ostream& out,
              const Feature_Product& other ) {
    return out;
  }
}

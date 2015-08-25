/**
 * @file    dcg.cc
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The implementation of a class used to represent a Distributed 
 *   Correspondence Graph
 */

#include <fstream>
#include <utility>

#include "h2sl/grounding_set.h"
#include "h2sl/region.h"
#include "h2sl/constraint.h"
#include "h2sl_cdcg/dcg.h"

using namespace std;
using namespace h2sl_cdcg;

DCG::
DCG() : h2sl::DCG() {

}

DCG::
~DCG() {

}

DCG::
DCG( const DCG& other ) : h2sl::DCG( other ) {

}

DCG&
DCG::
operator=( const DCG& other ) {
  _search_spaces = other._search_spaces;
  _solutions = other._solutions;
  _root = other._root;
  return (*this);
}

void
DCG::
fill_search_spaces( const h2sl::World* world ){
  for( unsigned int i = 0; i < _search_spaces.size(); i++ ){
    if( _search_spaces[ i ].second != NULL ){
      delete _search_spaces[ i ].second;
      _search_spaces[ i ].second = NULL;
    }
    _search_spaces.clear();
  }

  std::vector< unsigned int > binary_cvs;
  binary_cvs.push_back( h2sl::CV_FALSE );
  binary_cvs.push_back( h2sl::CV_TRUE );

  std::vector< unsigned int > ternary_cvs;
  ternary_cvs.push_back( h2sl::CV_FALSE );
  ternary_cvs.push_back( h2sl::CV_TRUE );
  ternary_cvs.push_back( h2sl::CV_INVERTED );

  // add the NP groundings
  for( unsigned int i = 0; i < h2sl::NUM_REGION_TYPES; i++ ){
    if( i != h2sl::REGION_TYPE_UNKNOWN ){
      _search_spaces.push_back( pair< vector< unsigned int >, h2sl::Grounding* >( binary_cvs, new h2sl::Region( i, h2sl::Object() ) ) );
    }
    for( unsigned int j = 0; j < world->objects().size(); j++ ){
      _search_spaces.push_back( pair< vector< unsigned int >, h2sl::Grounding* >( binary_cvs, new h2sl::Region( i, *world->objects()[ j ] ) ) );
    }
  }

  // add the PP groundings
  for( unsigned int i = 0; i < h2sl::NUM_REGION_TYPES; i++ ){
    if( i != h2sl::REGION_TYPE_UNKNOWN ){
      _search_spaces.push_back( pair< vector< unsigned int >, h2sl::Grounding* >( binary_cvs, new h2sl::Region( i, h2sl::Object() ) ) );
    }
    for( unsigned int j = 0; j < world->objects().size(); j++ ){
      _search_spaces.push_back( pair< vector< unsigned int >, h2sl::Grounding* >( binary_cvs, new h2sl::Region( i, *world->objects()[ j ] ) ) );
    }
  }

  // add the VP groundings
  for( unsigned int i = h2sl::CONSTRAINT_TYPE_INSIDE; i < h2sl::NUM_CONSTRAINT_TYPES; i++ ){
    for( unsigned int j = 0; j < world->objects().size(); j++ ){
      for( unsigned int k = 0; k < h2sl::NUM_REGION_TYPES; k++ ){
        for( unsigned int l = 0; l < world->objects().size(); l++ ){
          for( unsigned int m = 0; m < h2sl::NUM_REGION_TYPES; m++ ){
            if( ( j != l ) || ( k != m ) ){
              _search_spaces.push_back( pair< vector< unsigned int >, h2sl::Grounding* >( ternary_cvs, new h2sl::Constraint( i, h2sl::Region( k, *world->objects()[ j ] ), h2sl::Region( m, *world->objects()[ l ] ) ) ) );
            }
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
              const DCG& other ) {
    
    return out;
  }
}

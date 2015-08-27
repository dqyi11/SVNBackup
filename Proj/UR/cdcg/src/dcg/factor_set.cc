/**
 * @file    factor_set.cc
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The implementation of a class used to represent a factor set
 */

#include "h2sl/common.h"
#include "h2sl/constraint.h"
#include "h2sl/grounding.h"
#include "h2sl/llm.h"
#include "h2sl/world.h"
#include "h2sl_cdcg/ccv.h"
#include "h2sl_cdcg/factor_set.h"

using namespace std;
using namespace h2sl_cdcg;


bool
factor_set_solution_sort( const h2sl::Factor_Set_Solution& a,
                              const h2sl::Factor_Set_Solution& b ){
  return a.pygx > b.pygx;
}

Factor_Set::
Factor_Set( const h2sl::Phrase* phrase ) : _phrase( phrase ),
                                          _children(),
                                          _solutions() {

}

Factor_Set::
~Factor_Set() {

}

Factor_Set::
Factor_Set( const Factor_Set& other ) : _phrase( other._phrase ),
                                                _children( other._children ),
                                                _solutions( other._solutions ){

}

Factor_Set&
Factor_Set::
operator=( const Factor_Set& other ) {
  _phrase = other._phrase;
  _children = other._children;
  _solutions = other._solutions;
  return (*this);
}

void
Factor_Set::
search( const vector< pair< vector< unsigned int >, h2sl::Grounding* > >& searchSpace,
        const h2sl::World* world, 
        h2sl::LLM* llm,
        const unsigned int beamWidth,
        const bool& debug ){

  vector< vector< unsigned int > > child_solution_indices;
  for( unsigned int i = 0; i < _children.size(); i++ ){
    child_solution_indices.push_back( vector< unsigned int >() );
    for( unsigned int j = 0; j < _children[ i ]->solutions().size(); j++ ){
      child_solution_indices.back().push_back( j );
    }
  }

  vector< vector< unsigned int > > child_solution_indices_cartesian_power = h2sl::cartesian_power( child_solution_indices );
  if( child_solution_indices_cartesian_power.empty() ){
    child_solution_indices_cartesian_power.push_back( vector< unsigned int >() );
  }

  vector< bool > evaluate_feature_types( h2sl::NUM_FEATURE_TYPES, true );

  vector< vector< h2sl::Factor_Set_Solution > > solutions_vector;
  for( unsigned int i = 0; i < child_solution_indices_cartesian_power.size(); i++ ){
    solutions_vector.push_back( vector< h2sl::Factor_Set_Solution >() );
    solutions_vector.back().push_back( h2sl::Factor_Set_Solution() );
    solutions_vector.back().back().children = child_solution_indices_cartesian_power[ i ];
    solutions_vector.back().back().cv.resize( h2sl_cdcg::NUM_CCVS );

    vector< h2sl::Grounding* > child_groundings;
    for( unsigned int j = 0; j < child_solution_indices_cartesian_power[ i ].size(); j++ ){
      solutions_vector.back().back().pygx *= _children[ j ]->solutions()[ child_solution_indices_cartesian_power[ i ][ j ] ].pygx;
      for( unsigned int k = 0; k < _children[ j ]->solutions()[ child_solution_indices_cartesian_power[ i ][ j ] ].groundings.size(); k++ ){
        child_groundings.push_back( _children[ j ]->solutions()[ child_solution_indices_cartesian_power[ i ][ j ] ].groundings[ k ] );
      }
    }
    
    for( unsigned int j = 0; j < searchSpace.size(); j++ ){
      unsigned int num_solutions = solutions_vector.back().size();
      for( unsigned int k = 1; k < searchSpace[ j ].first.size(); k++ ){
        for( unsigned int l = 0; l < num_solutions; l++ ){
          solutions_vector.back().push_back( solutions_vector.back()[ l ] );
        } 
      }
  
      for( unsigned int k = 0; k < searchSpace[ j ].first.size(); k++ ){
        double value = llm->pygx( searchSpace[ j ].first[ k ], searchSpace[ j ].second, child_groundings, _phrase, world, searchSpace[ j ].first, evaluate_feature_types );
        evaluate_feature_types[ h2sl::FEATURE_TYPE_LANGUAGE ] = false;
        for( unsigned int l = 0; l < num_solutions; l++ ){
          solutions_vector.back()[ k * num_solutions + l ].cv[ searchSpace[ j ].first[ k ] ].push_back( j ); 
          solutions_vector.back()[ k * num_solutions + l ].pygx *= value; 
        }
      }
      
      sort( solutions_vector.back().begin(), solutions_vector.back().end(), factor_set_solution_sort );   
      
      if( solutions_vector.back().size() > beamWidth ){
        solutions_vector.back().erase( solutions_vector.back().begin() + beamWidth, solutions_vector.back().end() );
      }
    }
  }

  // flatten solutions
  _solutions.clear();
  for( unsigned int i = 0; i < solutions_vector.size(); i++ ){
    for( unsigned int j = 0; j < solutions_vector[ i ].size(); j++ ){
      _solutions.push_back( solutions_vector[ i ][ j ] );
    }
  } 

  if( debug ){
    cout << "  sorting through " << _solutions.size() << " solutions for \"" << _phrase->words_to_std_string() << "\"" << endl;
  }

  sort( _solutions.begin(), _solutions.end(), factor_set_solution_sort );
  if( _solutions.size() > beamWidth ){
    _solutions.erase( _solutions.begin() + beamWidth, _solutions.end() );
  }

  for( unsigned int i = 0; i < _solutions.size(); i++ ){
    for( unsigned int j = 0; j < _solutions[ i ].cv[ h2sl::CV_TRUE ].size(); j++ ){
      _solutions[ i ].groundings.push_back( searchSpace[ _solutions[ i ].cv[ h2sl::CV_TRUE ][ j ] ].second );
    }
  }

  if( debug ){
    for( unsigned int i = 0; i < _solutions.size(); i++ ){
      cout << "solutions[" << i << "] TRUE [" << _solutions[ i ].cv[ h2sl::CV_TRUE ].size() << "]:{";
      for( unsigned int j = 0; j < _solutions[ i ].cv[ h2sl::CV_TRUE ].size(); j++ ){
        cout << _solutions[ i ].cv[ h2sl::CV_TRUE ][ j ];
        if( j != ( _solutions[ i ].cv[ h2sl::CV_TRUE ].size() - 1 ) ){
          cout << ",";
        }
      }
      cout << "} ";
      cout << "groundings[" << _solutions[ i ].groundings.size() << "]:{";
      cout << "} ";
      cout << "children[" << _solutions[ i ].children.size() << "]:{";
      for( unsigned int j = 0; j < _solutions[ i ].children.size(); j++ ){
        cout << _solutions[ i ].children[ j ];
        if( j != ( _solutions[ i ].children.size() - 1 ) ){
          cout << ",";
        }
      }
      cout << "} ";
      cout << "pygx:" << _solutions[ i ].pygx << endl;
    }  
  }

  return;
}

namespace h2sl_cdcg {
  ostream&
  operator<<( ostream& out,
              const Factor_Set& other ) {
    out << endl;
    out << "phrase:\"" << other.phrase() << "\" ";
    return out;
  }
}

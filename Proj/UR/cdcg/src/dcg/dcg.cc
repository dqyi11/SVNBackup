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
#include "h2sl/object.h"
#include "h2sl/constraint.h"
#include "h2sl_cdcg/dcg.h"
#include "h2sl_cdcg/ccv.h"
#include "h2sl_cdcg/func_kernel.h"

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

  std::vector< unsigned int > ccvs;
  ccvs.push_back( h2sl_cdcg::CCV_ZERO );
  ccvs.push_back( h2sl_cdcg::CCV_ONE );
  ccvs.push_back( h2sl_cdcg::CCV_TWO );
  ccvs.push_back( h2sl_cdcg::CCV_THREE );
  ccvs.push_back( h2sl_cdcg::CCV_FOUR );
  ccvs.push_back( h2sl_cdcg::CCV_FIVE );

  // add the NP groundings
  for( unsigned int i = 0; i < h2sl::NUM_REGION_TYPES; i++ ){
    if( i != h2sl::REGION_TYPE_UNKNOWN ){
      _search_spaces.push_back( pair< vector< unsigned int >, h2sl::Grounding* >( binary_cvs, new h2sl::Region( i, h2sl::Object() ) ) );
    }
    for( unsigned int j = 0; j < world->objects().size(); j++ ){
      _search_spaces.push_back( pair< vector< unsigned int >, h2sl::Grounding* >( binary_cvs, new h2sl::Region( i, *world->objects()[ j ] ) ) );
    }
  }
  
  for( unsigned int j = 0; j < world->objects().size(); j++ ){
    _search_spaces.push_back( pair< vector< unsigned int >, h2sl::Grounding* >( binary_cvs, new h2sl::Object( *world->objects()[ j ] ) ) ); 
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
  for( unsigned int i = 0; i < h2sl_cdcg::NUM_FUNC_KERNEL_TYPES; i++ ){
    if( i != h2sl_cdcg::FUNC_KERNEL_TYPE_UNKNOWN ){
      _search_spaces.push_back( pair< vector< unsigned int >, h2sl::Grounding* >( ccvs, new h2sl_cdcg::Func_Kernel( i, h2sl::Object() ) ) );
    }
    for( unsigned int j = 0; j < world->objects().size(); j++ ){
      _search_spaces.push_back( pair< vector< unsigned int >, h2sl::Grounding* >( ccvs, new h2sl_cdcg::Func_Kernel( i, *world->objects()[ j ] ) ) );
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

bool
DCG::
leaf_search( const h2sl::Phrase* phrase,
              const h2sl::World* world,
              h2sl::LLM * llm,
              const unsigned int beamWidth,
              const bool& debug ){
  for( unsigned int i = 0; i < _solutions.size(); i++ ){
    if( _solutions[ i ].second != NULL ){
      delete _solutions[ i ].second;
      _solutions[ i ].second = NULL;
    }
  }
  _solutions.clear();

  if( phrase != NULL ){
    fill_search_spaces( world );

    if( _root != NULL ){
      delete _root;
      _root = NULL;
    }

    _root = new h2sl::Factor_Set( phrase->dup() );
    _fill_factors( _root, _root->phrase() );  

    h2sl::Factor_Set * leaf = NULL;
    _find_leaf( _root, leaf );
    while( leaf != NULL ){
      cout << "SEARCH " << leaf->phrase() << endl;
      leaf->search( _search_spaces,
                    world,
                    llm,
                    beamWidth,
                    debug );
      leaf = NULL;
      _find_leaf( _root, leaf );
    }
  
    for( unsigned int i = 0; i < _root->solutions().size(); i++ ){
      _solutions.push_back( pair< double, h2sl::Phrase* >( _root->solutions()[ i ].pygx, _root->phrase()->dup() ) );

      for( unsigned int j = 0; j < _solutions.back().second->children().size(); j++ ){
        if( _solutions.back().second->children()[ j ] != NULL ){
          delete _solutions.back().second->children()[ j ];
          _solutions.back().second->children()[ j ];
        }
      }
      _solutions.back().second->children().clear();
      if( _solutions.back().second->grounding() != NULL ){
        delete _solutions.back().second->grounding();
        _solutions.back().second->grounding() = NULL;
      }
      _fill_phrase( _root, _root->solutions()[ i ], _solutions.back().second );
    }

    return true;
  } else {
    return false;
  }
}

void
DCG::
to_latex( const string& filename )const{
  ofstream outfile;
  outfile.open( filename.c_str() );
  outfile << "\\begin{tikzpicture}[textnode/.style={anchor=mid,font=\\tiny},nodeknown/.style={circle,draw=black!80,fill=black!10,minimum size=5mm,font=\\tiny,top color=white,bottom color=black!20},nodeunknown/.style={circle,draw=black!80,fill=white,minimum size=5mm,font=\\tiny},factor/.style={rectangle,draw=black!80,fill=black!80,minimum size=2mm,font=\\tiny,text=white},dot/.style={circle,draw=black!80,fill=black!80,minimum size=0.25mm,font=\\tiny}]" << endl;
  outfile << "\\end{tikzpicture}" << endl;
  outfile.close();
  return;
} 

void
DCG::
_find_leaf( h2sl::Factor_Set* node, 
            h2sl::Factor_Set*& leaf ){ 
  if( node->solutions().empty() ){
    bool all_children_known = true;
    for( unsigned int i = 0; i < node->children().size(); i++ ){
      if( node->children()[ i ]->solutions().empty() ){
        all_children_known = false;
      } 
    } 
    if( all_children_known ){
      leaf = node;
    }
  }

  for( unsigned int i = 0; i < node->children().size(); i++ ){
    _find_leaf( node->children()[ i ], leaf );
  }
  return;
}

void
DCG::
_fill_phrase( h2sl::Factor_Set* node,
              h2sl::Factor_Set_Solution& solution,
              h2sl::Phrase* phrase ){
  phrase->grounding() = new h2sl::Grounding_Set();
  for( unsigned int i = 0; i < solution.groundings.size(); i++ ){
    dynamic_cast< h2sl::Grounding_Set* >( phrase->grounding() )->groundings().push_back( solution.groundings[ i ] );
  }
  for( unsigned int i = 0; i < node->children().size(); i++ ){
    phrase->children().push_back( node->children()[ i ]->phrase()->dup() );
    for( unsigned int j = 0; j < phrase->children().back()->children().size(); j++ ){
      if( phrase->children().back()->children()[ j ] != NULL ){
        delete phrase->children().back()->children()[ j ];
        phrase->children().back()->children()[ j ];
      }
    }
    phrase->children().back()->children().clear();
    if( phrase->children().back()->grounding() != NULL ){
      delete phrase->children().back()->grounding();
      phrase->children().back()->grounding() = NULL;
    }

    _fill_phrase( node->children()[ i ],
                  node->children()[ i ]->solutions()[ solution.children[ i ] ],
                  phrase->children().back() );

  }
  return;
}

void
DCG::
_fill_factors( h2sl::Factor_Set* node,
                const h2sl::Phrase* phrase, 
                const bool& fill ){
  for( unsigned int i = 0; i < phrase->children().size(); i++ ){
    node->children().push_back( new h2sl::Factor_Set( phrase->children()[ i ] ) );
    _fill_factors( node->children().back(), phrase->children()[ i ] );
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

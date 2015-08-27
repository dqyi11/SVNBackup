/**
 * @file    factor.cc
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The implementation of a class used to describe a model factor
 */

#include <h2sl/region.h>
#include <h2sl/constraint.h>
#include <h2sl/object.h>
#include <h2sl/llm.h>
#include <h2sl_cdcg/func_kernel.h>
#include <h2sl_cdcg/factor.h>

using namespace std;
using namespace h2sl_cdcg;

Factor::
Factor( const unsigned int& cv,
        h2sl::Grounding* grounding,
        h2sl::Phrase* phrase,
        const h2sl::World* world,
        const vector< Factor* >& children,
        h2sl::LLM* llm,
        const vector< unsigned int >& cvs,
        const unsigned int& solutionIndex ) : _cv( cv ),
                                                  _grounding( grounding ),
                                                  _phrase( phrase ),
                                                  _world( world ),
                                                  _children( children ),
                                                  _llm( llm ),
                                                  _cvs( cvs ),
                                                  _solution_index( solutionIndex ),
                                                  _pygx( 0.0 ){

}

Factor::
~Factor() {

}

Factor::
Factor( const Factor& other ) : _cv( other._cv ),
                                _grounding( other._grounding ),
                                _phrase( other._phrase ),
                                _world( other._world ),
                                _children( other._children ),
                                _llm( other._llm ),
                                _cvs( other._cvs ),
                                _solution_index( other._solution_index ),
                                _pygx( other._pygx ){

}

Factor&
Factor::
operator=( const Factor& other ) {
  _cv = other._cv;
  _grounding = other._grounding;
  _children = other._children;
  _phrase = other._phrase;
  _world = other._world;
  _llm = other._llm;
  _cvs = other._cvs;
  _solution_index = other._solution_index;
  _pygx = other._pygx;
  return (*this);
}

double
Factor::
value( const unsigned int& cv ){
  _cv = cv;
  if( _llm != NULL ){
    vector< h2sl::Grounding* > children;
    for( unsigned int i = 0; i < _children.size(); i++ ){ 
      _children[ i ]->cv() = _children[ i ]->cv();
      if( _children[ i ]->cvs().size() == 2 ){
        if( _children[ i ]->cv() == h2sl::CV_TRUE ){
          children.push_back( _children[ i ]->grounding() );
        } 
      } else if ( _children[ i ]->cvs().size() == 3 ){
        if( ( _children[ i ]->cv() == h2sl::CV_INVERTED ) || ( _children[ i ]->cv() == h2sl::CV_TRUE ) ){
          children.push_back( _children[ i ]->grounding() );
        }
      } else if ( _children[ i ]->cvs().size() > 3 ){
        if( _children[ i ]->cv() > h2sl_cdcg::CCV_ZERO ){
          children.push_back( _children[ i ]->grounding() );
        }
      }      
    }
    _pygx = _llm->pygx( _cv, _grounding, children, _phrase, _world, _cvs );
    return _pygx;
  } else {
    return 0.5;
  }
}

namespace h2sl_cdcg {
  ostream&
  operator<<( ostream& out,
              const Factor& other ) {
    out << "cv:\"" << other.cv() << "\" ";
    if( dynamic_cast< const h2sl::Region* >( other.grounding() ) != NULL ){
      out << "grounding:(" << *static_cast< const h2sl::Region* >( other.grounding() ) << ") ";
    } else if( dynamic_cast< const h2sl::Constraint* >( other.grounding() ) != NULL ){
      out << "grounding:(" << *static_cast< const h2sl::Constraint* >( other.grounding() ) << ") ";
    } else if( dynamic_cast< const h2sl::Object* >( other.grounding() ) != NULL ){
      out << "grounding:(" << *static_cast< const h2sl::Object* >( other.grounding() ) << ") ";
    } else if( dynamic_cast< const Func_Kernel* >( other.grounding() ) != NULL ){
      out << "grounding:(" << *static_cast< const Func_Kernel* >( other.grounding() ) << ") ";
    } else {
      out << "grounding:(NULL) ";
    }
    if( other.phrase() != NULL ){
      out << "phrase:(\"" << other.phrase()->words_to_std_string() << "\") ";
    } else {
      out << "phrase:(NULL) ";
    }
    out << "children[" << other.children().size() << "]:{";
    for( unsigned int i = 0; i < other.children().size(); i++ ){
      out << other.children()[ i ];
      if( i != ( other.children().size() - 1 ) ){
        out << ",";
      }
    }
    out << "} ";
    out << "solution_index:\"" << other.solution_index() << "\"";
    return out;
  }
}

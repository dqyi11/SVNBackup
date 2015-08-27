/**
 * @file    factor.h
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 *          Matthew R. Walter (mwalter@csail.mit.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to describe a model factor
 */

#ifndef H2SL_FACTOR_H_CDCG
#define H2SL_FACTOR_H_CDCG

#include "h2sl/factor.h"

namespace h2sl_cdcg {
  class Factor : public h2sl::Factor {
  public:
    Factor( const unsigned int& cv = 0, h2sl::Grounding* grounding = NULL, h2sl::Phrase* phrase = NULL, const h2sl::World* world = NULL, const std::vector< Factor* >& children = std::vector< Factor* >(), h2sl::LLM* llm = NULL, const std::vector< unsigned int >& cvs = std::vector< unsigned int >(), const unsigned int& solutionIndex = 0 );
    virtual ~Factor();
    Factor( const Factor& other );
    Factor& operator=( const Factor& other );

    double value( const unsigned int& cv );

    inline unsigned int& cv( void ){ return _cv; };
    inline const unsigned int& cv( void )const{ return _cv; };
    inline h2sl::Grounding*& grounding( void ){ return _grounding; };
    inline const h2sl::Grounding* grounding( void )const{ return _grounding; };
    inline h2sl::Phrase*& phrase( void ){ return _phrase; };
    inline const h2sl::Phrase* phrase( void )const{ return _phrase; };
    inline const h2sl::World* world( void )const{ return _world; };
    inline std::vector< Factor* >& children( void ){ return _children; };
    inline const std::vector< Factor* >& children( void )const{ return _children; };
    inline h2sl::LLM*& llm( void ){ return _llm; };
    inline const h2sl::LLM* llm( void )const{ return _llm; };
    inline std::vector< unsigned int >& cvs( void ){ return _cvs; };
    inline const std::vector< unsigned int >& cvs( void )const{ return _cvs; }; 
    inline unsigned int& solution_index( void ){ return _solution_index; };
    inline const unsigned int& solution_index( void )const{ return _solution_index; }; 
    inline const double& pygx( void )const{ return _pygx; };

  protected:
    unsigned int _cv;
    h2sl::Grounding* _grounding;
    h2sl::Phrase* _phrase;
    const h2sl::World* _world;
    std::vector< Factor* > _children;
    h2sl::LLM * _llm;
    std::vector< unsigned int > _cvs;
    unsigned int _solution_index;
    double _pygx;   

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Factor& other );
}

#endif /* H2SL_FACTOR_H_CDCG */

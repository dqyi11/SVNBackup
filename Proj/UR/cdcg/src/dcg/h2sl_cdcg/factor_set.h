/**
 * @file    factor_set.h
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to represent a factor set
 */

#ifndef H2SL_FACTOR_SET_H_CDCG
#define H2SL_FACTOR_SET_H_CDCG

#include "h2sl/factor_set.h"
#include "h2sl_cdcg/phrase.h"

namespace h2sl_cdcg {
  /*
  class Factor_Set_Solution {
  public:
    Factor_Set_Solution();
    virtual ~Factor_Set_Solution();
    Factor_Set_Solution( const Factor_Set_Solution& other );
    Factor_Set_Solution& operator=( const Factor_Set_Solution& other );
  
    std::vector< std::vector< unsigned int > > cv;
    std::vector< unsigned int > children;
    std::vector< Grounding* > groundings;
    double pygx;
  };*/

  class Factor_Set : public h2sl::Factor_Set {
  public:
    Factor_Set( const h2sl::Phrase* phrase = NULL );
    virtual ~Factor_Set();
    Factor_Set( const Factor_Set& other );
    Factor_Set& operator=( const Factor_Set& other );

    void search( const std::vector< std::pair< std::vector< unsigned int >, h2sl::Grounding* > >& searchSpace, const h2sl::World* world, h2sl::LLM* llm, const unsigned int beamWidth = 4, const bool& debug = false );

    inline const h2sl::Phrase* phrase( void )const{ return _phrase; };

    inline std::vector< Factor_Set* >& children( void ){ return _children; };
    inline const std::vector< Factor_Set* >& children( void )const{ return _children; };

    inline std::vector< h2sl::Factor_Set_Solution >& solutions( void ){ return _solutions; };
    inline const std::vector< h2sl::Factor_Set_Solution >& solutions( void )const{ return _solutions; };

  protected:
    const h2sl::Phrase* _phrase;
    std::vector< Factor_Set* > _children;
    std::vector< h2sl::Factor_Set_Solution > _solutions;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Factor_Set& other );
}

#endif /* H2SL_FACTOR_SET_H_CDCG */

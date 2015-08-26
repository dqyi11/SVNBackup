/**
 * @file    dcg.h
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to represent a Distributed Correspondence
 *   Graph
 */

#ifndef H2SL_DCG_H_CDCG
#define H2SL_DCG_H_CDCG

#include "h2sl/dcg.h"

namespace h2sl_cdcg {
  class DCG : public h2sl::DCG {
  public:
    DCG();
    virtual ~DCG();
    DCG( const DCG& other );
    DCG& operator=( const DCG& other );

    virtual void fill_search_spaces( const h2sl::World* world );
    void construct( const h2sl::Phrase* phrase, const h2sl::World* world, const bool& fill = false );
    bool leaf_search( const h2sl::Phrase* phrase, const h2sl::World* world, h2sl::LLM* llm, const unsigned int beamWidth = 4, const bool& debug = false );

    void to_latex( const std::string& filename )const;

    inline const std::vector< std::pair< std::vector< unsigned int >, h2sl::Grounding* > >& search_spaces( void ){ return _search_spaces; };
    inline const std::vector< std::pair< double, h2sl::Phrase* > >& solutions( void )const{ return _solutions; };
    inline const h2sl::Factor_Set* root( void )const{ return _root; };
  protected:
    virtual void _find_leaf( h2sl::Factor_Set* node, h2sl::Factor_Set*& leaf );
    virtual void _fill_phrase( h2sl::Factor_Set* node, h2sl::Factor_Set_Solution& solution, h2sl::Phrase* phrase );
    virtual void _fill_factors( h2sl::Factor_Set* node, const h2sl::Phrase* phrase, const bool& fill = false );  
  private:

  };
  std::ostream& operator<<( std::ostream& out, const DCG& other );
}

#endif /* H2SL_DCG_H_CDCG */

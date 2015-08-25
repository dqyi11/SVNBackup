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

  protected:
  
  private:

  };
  std::ostream& operator<<( std::ostream& out, const DCG& other );
}

#endif /* H2SL_DCG_H_CDCG */

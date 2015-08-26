/**
 * @file    grounding_set.h
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to describe a set of groundings
 */

#ifndef H2SL_GROUNDING_SET_H_CDCG
#define H2SL_GROUNDING_SET_H_CDCG

#include <iostream>
#include <vector>

#include "h2sl/grounding_set.h"

namespace h2sl_cdcg {
  class Grounding_Set: public h2sl::Grounding_Set {
  public:
    Grounding_Set( const std::vector< Grounding* >& groundings = std::vector< Grounding* >() );
    virtual ~Grounding_Set();
    Grounding_Set( const Grounding_Set& other );
    Grounding_Set& operator=( const Grounding_Set& other );

    virtual void to_xml( const std::string& filename )const;
    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( const std::string& filename );
    virtual void from_xml( xmlNodePtr root );

  protected:

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Grounding_Set& other );
}

#endif /* H2SL_GROUNDING_SET_H_CDCG */

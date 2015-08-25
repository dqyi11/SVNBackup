/**
 * @file    feature_set.h
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to describe a set of features
 */

#ifndef H2SL_FEATURE_SET_H_CDCG
#define H2SL_FEATURE_SET_H_CDCG

#include <iostream>
#include <vector>
#include <libxml/tree.h>

#include <h2sl/grounding.h>
#include <h2sl/feature.h>
#include <h2sl/feature_set.h>
#include <h2sl_cdcg/feature_product.h>

namespace h2sl_cdcg {

  class Feature_Set: public h2sl::Feature_Set{
  public:
    Feature_Set();
    virtual ~Feature_Set();
    Feature_Set( const Feature_Set& other );
    Feature_Set& operator=( const Feature_Set& other );

    virtual void to_xml( const std::string& filename )const;
    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( const std::string& filename );
    virtual void from_xml( xmlNodePtr root );

  protected:

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Feature_Set& other );

}

#endif /* H2SL_FEATURE_SET_H_CDCG */

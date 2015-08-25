/**
 * @file    feature_product.h
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to describe a product of features
 */

#ifndef H2SL_FEATURE_PRODUCT_H_CDCG
#define H2SL_FEATURE_PRODUCT_H_CDCG

#include "h2sl/feature_product.h"

namespace h2sl_cdcg {
  class Feature_Product : public h2sl::Feature_Product {
  public:
    Feature_Product();
    virtual ~Feature_Product();
    Feature_Product( const Feature_Product& other );
    Feature_Product& operator=( const Feature_Product& other );

    virtual void to_xml( const std::string& filename )const;
    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( const std::string& filename );
    virtual void from_xml( xmlNodePtr root );

  protected:

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Feature_Product& other );
}

#endif /* H2SL_FEATURE_PRODUCT_H_CDCG */

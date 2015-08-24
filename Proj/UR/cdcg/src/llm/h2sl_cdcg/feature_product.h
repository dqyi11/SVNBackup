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

    void indices( const unsigned int& cv, const h2sl::Grounding* grounding, const std::vector< h2sl::Grounding* >& children, const h2sl::Phrase* phrase, const h2sl::World* world, std::vector< unsigned int >& indices, const std::vector< bool >& evaluateFeatureTypes );
    void evaluate( const unsigned int& cv, const h2sl::Grounding* grounding, const std::vector< h2sl::Grounding* >& children, const h2sl::Phrase* phrase, const h2sl::World* world, const std::vector< bool >& evaluateFeatureTypes );

    virtual void to_xml( const std::string& filename )const;
    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( const std::string& filename );
    virtual void from_xml( xmlNodePtr root );

    unsigned int size( void )const;

    inline std::vector< std::vector< h2sl::Feature* > >& feature_groups( void ){ return _feature_groups; };
    inline const std::vector< std::vector< h2sl::Feature* > >& feature_groups( void )const{ return _feature_groups; };
    inline std::vector< std::vector< bool > >& values( void ){ return _values; };
    inline const std::vector< std::vector< bool > >& values( void )const{ return _values; };

  protected:
    std::vector< std::vector< h2sl::Feature* > > _feature_groups;
    std::vector< std::vector< bool > > _values;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Feature_Product& other );
}

#endif /* H2SL_FEATURE_PRODUCT_H_CDCG */

/** 
 * @file    feature_func_kernel_matches_child_region.h
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to check for a match between a grounding func kernel and a child object
 */

#ifndef H2SL_FEATURE_FUNC_KERNEL_MATCHES_CHILD_REGION_H
#define H2SL_FEATURE_FUNC_KERNEL_MATCHES_CHILD_REGION_H

#include <iostream>

#include <h2sl/feature.h>

namespace h2sl_cdcg {
  class Feature_Func_Kernel_Matches_Child_Region: public h2sl::Feature {
  public:
    Feature_Func_Kernel_Matches_Child_Region( const bool& invert = false );
    virtual ~Feature_Func_Kernel_Matches_Child_Region();
    Feature_Func_Kernel_Matches_Child_Region( const Feature_Func_Kernel_Matches_Child_Region& other );
    Feature_Func_Kernel_Matches_Child_Region& operator=( const Feature_Func_Kernel_Matches_Child_Region& other );

    virtual bool value( const unsigned int& cv, const h2sl::Grounding* grounding, const std::vector< h2sl::Grounding* >& children, const h2sl::Phrase* phrase, const h2sl::World* world );

    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( xmlNodePtr root );

    virtual inline const h2sl::feature_type_t type( void )const{ return h2sl::FEATURE_TYPE_GROUNDING; };

  protected:

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Feature_Func_Kernel_Matches_Child_Region& other );
}

#endif /* H2SL_FEATURE_FUNC_KERNEL_MATCHES_CHILD_REGION_H */
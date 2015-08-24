/**
 * @file    feature_func_kernel.h
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to check for a match with a func kernel's object
 */

#ifndef H2SL_FEATURE_FUNC_KERNEL_OBJECT_H_CDCG
#define H2SL_FEATURE_FUNC_KERNEL_OBJECT_H_CDCG

#include <iostream>

#include "h2sl/feature.h"

namespace h2sl_cdcg {
  class Feature_Func_Kernel : public h2sl::Feature {
  public:
    Feature_Func_Kernel( const bool& invert = false, const unsigned int& kernelType = 0 );
    virtual ~Feature_Func_Kernel();
    Feature_Func_Kernel( const Feature_Func_Kernel& other );
    Feature_Func_Kernel& operator=( const Feature_Func_Kernel& other );

    virtual bool value( const unsigned int& cv, const h2sl::Grounding* grounding, const std::vector< h2sl::Grounding* >& children, const h2sl::Phrase* phrase, const h2sl::World* world );

    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( xmlNodePtr root );

    inline unsigned int& kernel_type( void ){ return _kernel_type; };
    inline const unsigned int& kernel_type( void )const{ return _kernel_type; };

    virtual inline const h2sl::feature_type_t type( void )const{ return h2sl::FEATURE_TYPE_GROUNDING; };

  protected:
    unsigned int _kernel_type;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Feature_Func_Kernel& other );
}

#endif /* H2SL_FEATURE_FUNC_KERNEL_OBJECT_H_CDCG */

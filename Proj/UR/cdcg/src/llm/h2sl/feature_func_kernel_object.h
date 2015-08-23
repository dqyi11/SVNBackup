/**
 * @file    feature_func_kernel_object.h
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to check for a match with a func kernel's object
 */

#ifndef H2SL_FEATURE_FUNC_KERNEL_OBJECT_H
#define H2SL_FEATURE_FUNC_KERNEL_OBJECT_H

#include <iostream>

#include "h2sl/feature.h"

namespace h2sl {
  class Feature_Func_Kernel_Object: public Feature {
  public:
    Feature_Func_Kernel_Object( const bool& invert = false, const unsigned int& objectType = 0 );
    virtual ~Feature_Func_Kernel_Object();
    Feature_Func_Kernel_Object( const Feature_Func_Kernel_Object& other );
    Feature_Func_Kernel_Object& operator=( const Feature_Func_Kernel_Object& other );

    virtual bool value( const unsigned int& cv, const Grounding* grounding, const std::vector< Grounding* >& children, const Phrase* phrase, const World* world );

    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( xmlNodePtr root );

    inline unsigned int& object_type( void ){ return _object_type; };
    inline const unsigned int& object_type( void )const{ return _object_type; };

    virtual inline const feature_type_t type( void )const{ return FEATURE_TYPE_GROUNDING; };

  protected:
    unsigned int _object_type;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Feature_Func_Kernel_Object& other );
}

#endif /* H2SL_FEATURE_FUNC_KERNEL_OBJECT_H */

/**
 * @file    feature_ccv.h
 * @author  Daqing Yi (daqing.yi@byu.edu)
 *
 * @section DESCRIPTION
 *
 * The interface for a class that looks at the value of the CV
 */

#ifndef H2SL_FEATURE_CCV_H
#define H2SL_FEATURE_CCV_H

#include <iostream>

#include <h2sl/feature.h>

namespace h2sl {
  class Feature_CCV: public Feature {
  public:
    Feature_CCV( const int& cv = 0 );
    virtual ~Feature_CCV();
    Feature_CCV( const Feature_CCV& other );
    Feature_CCV& operator=( const Feature_CCV& other );

    virtual bool value( const int& cv, const Grounding* grounding, const std::vector< Grounding* >& children, const Phrase* phrase, const World* world ); 

    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( xmlNodePtr root );

    inline int& cv( void ){ return _ccv; };
    inline const int& cv( void )const{ return _ccv; };
    virtual inline const feature_type_t type( void )const{ return FEATURE_TYPE_CORRESPONDENCE; };

  protected:
    int _ccv;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Feature_CCV& other );
}

#endif /* H2SL_FEATURE_CCV_H */

/** 
 * @file   feature_ccv.h
 * @author Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 * @section DESCRIPTION
 * The class used to describe a continuous correspondence variable
 */

#ifndef H2SL_FEATURE_CCV_H
#define H2SL_FEATURE_CCV_H

#include <h2sl/feature.h>

namespace h2sl {
  class Feature_CCV: public Feature {
    public:
      Feature_CCV( const double& cv = 0.0 );
      virtual ~Feature_CCV();
      Feature_CCV( const Feature_CCV& other );
      Feature_CCV& operator=( const Feature_CCV& other );

      virtual bool value( const double& ccv, const Grounding* grounding, const std::vector< Grounding* >& children, const Phrase* phrase, const World* world ); 

      virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

      virtual void from_xml( xmlNodePtr root );

      inline double& cv( void ){ return _cv; };

      inline const double& cv( void )const{ return _cv; };
      virtual inline const feature_type_t type( void )const{ return FEATURE_TYPE_CORRESPONDENCE; };

    protected:
      double _cv;

    private:

  };
  std::ostream& operator<<( std::ostream& out, const Feature_CCV& other );
}


#endif /* H2SL_FEATURE_CCV_H */

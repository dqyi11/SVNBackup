/**
 * @file    func_kernel.h
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 * @section DESCRIPTION
 *
 * The interface for a class used to describe a func_kernel of space
 */

#ifndef H2SL_FUNC_KERNEL_H
#define H2SL_FUNC_KERNEL_H

#include <iostream>

#include "h2sl/grounding.h"
#include "h2sl/region.h"
#include "h2sl_cdcg/ccv.h"

using namespace h2sl;

namespace h2sl_cdcg {
  typedef enum {
    FUNC_KERNEL_TYPE_UNKNOWN,
    FUNC_KERNEL_TYPE_GAUSSIAN,
    NUM_FUNC_KERNEL_TYPES
  } func_kernel_type_t;

  class Func_Kernel: public Grounding {
  public:
    Func_Kernel( const unsigned int& type = 0, const Region& region = Region() );
    Func_Kernel( const func_kernel_type_t& type, const Region& region );
    virtual ~Func_Kernel();
    Func_Kernel( const Func_Kernel& other );
    Func_Kernel& operator=( const Func_Kernel& other );
    bool operator==( const Func_Kernel& other )const;
    bool operator!=( const Func_Kernel& other )const;
    virtual Grounding* dup( void )const;

    static std::string type_to_std_string( const unsigned int& type );
    static unsigned int type_from_std_string( const std::string& type );

    virtual void to_xml( const std::string& filename )const;
    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( const std::string& filename );
    virtual void from_xml( xmlNodePtr root );

    inline unsigned int& type( void ){ return _type; };
    inline const unsigned int& type( void )const{ return _type; };
    inline Region& region( void ){ return _region; };
    inline const Region& region( void )const{ return _region; };
    inline float& weight( void ){ return _weight; };
    inline const float& weight( void )const{ return _weight; };
    inline unsigned int resolution( void ){ return APPROX_RESOLUTION; };
    inline const unsigned int resolution( void )const{ return APPROX_RESOLUTION; };

  protected:
    unsigned int _type;
    float _weight;
    Region _region;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Func_Kernel& other );
}

#endif /* H2SL_FUNC_KERNEL_H */

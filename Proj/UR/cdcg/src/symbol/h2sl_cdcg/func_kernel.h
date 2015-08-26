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
#include "h2sl/object.h"
#include "h2sl_cdcg/ccv.h"


namespace h2sl_cdcg {
  typedef enum {
    FUNC_KERNEL_TYPE_UNKNOWN,
    FUNC_KERNEL_TYPE_GAUSSIAN,
    NUM_FUNC_KERNEL_TYPES
  } func_kernel_type_t;

  class Func_Kernel: public h2sl::Grounding {
  public:
    Func_Kernel( const unsigned int& type = 0, const h2sl::Object& object = h2sl::Object() );
    Func_Kernel( const func_kernel_type_t& type, const h2sl::Object& object );
    virtual ~Func_Kernel();
    Func_Kernel( const Func_Kernel& other );
    Func_Kernel& operator=( const Func_Kernel& other );
    bool operator==( const Func_Kernel& other )const;
    bool operator!=( const Func_Kernel& other )const;
    virtual h2sl::Grounding* dup( void )const;

    static std::string type_to_std_string( const unsigned int& type );
    static unsigned int type_from_std_string( const std::string& type );

    virtual void to_xml( const std::string& filename )const;
    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( const std::string& filename );
    virtual void from_xml( xmlNodePtr root );

    inline unsigned int& type( void ){ return _type; };
    inline const unsigned int& type( void )const{ return _type; };
    inline h2sl::Object& object( void ){ return _object; };
    inline const h2sl::Object& object( void )const{ return _object; };
    inline float& weight( void ){ return _weight; };
    inline const float& weight( void )const{ return _weight; };
    inline unsigned int resolution( void ){ return NUM_CCVS - CCV_ZERO; };
    inline const unsigned int resolution( void )const{ return NUM_CCVS - CCV_ZERO; };

  protected:
    unsigned int _type;
    float _weight;
    h2sl::Object _object;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Func_Kernel& other );
}

#endif /* H2SL_FUNC_KERNEL_H */

/**
 * @file    object_kernel.h
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 *          Matthew R. Walter (mwalter@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * This file is part of h2sl.
 *
 * Copyright (C) 2014 by the Massachusetts Institute of Technology
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see
 * <http://www.gnu.org/licenses/gpl-2.0.html> or write to the Free
 * Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to describe a object_kernel of space
 */

#ifndef H2SL_OBJECT_KERNEL_H
#define H2SL_OBJECT_KERNEL_H

#include <iostream>

#include "h2sl/grounding.h"
#include "h2sl/object.h"

namespace h2sl {
  typedef enum {
    OBJECT_KERNEL_TYPE_UNKNOWN,
    OBJECT_KERNEL_TYPE_GAUSSIAN,
    NUM_OBJECT_KERNEL_TYPES
  } object_kernel_type_t;

  class Object_Kernel: public Grounding {
  public:
    Object_Kernel( const unsigned int& type = 0, const Object& object = Object() );
    Object_Kernel( const object_kernel_type_t& type, const Object& object );
    virtual ~Object_Kernel();
    Object_Kernel( const Object_Kernel& other );
    Object_Kernel& operator=( const Object_Kernel& other );
    bool operator==( const Object_Kernel& other )const;
    bool operator!=( const Object_Kernel& other )const;
    virtual Grounding* dup( void )const;

    static std::string type_to_std_string( const unsigned int& type );
    static unsigned int type_from_std_string( const std::string& type );

    virtual void to_xml( const std::string& filename )const;
    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( const std::string& filename );
    virtual void from_xml( xmlNodePtr root );

    inline unsigned int& type( void ){ return _type; };
    inline const unsigned int& type( void )const{ return _type; };
    inline Object& object( void ){ return _object; };
    inline const Object& object( void )const{ return _object; };

  protected:
    unsigned int _type;
    Object _object;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Object_Kernel& other );
}

#endif /* H2SL_OBJECT_KERNEL_H */

/**
 * @file    feature_func_kernel_object.h
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
 * The interface for a class used to check for a match with a region's object
 */

#ifndef H2SL_FEATURE_FUNC_KERNEL_OBJECT_H
#define H2SL_FEATURE_FUNC_KERNEL_OBJECT_H

#include <iostream>

#include "h2sl/feature.h"

namespace h2sl_cdcg {
  class Feature_Func_Kernel_Object: public h2sl::Feature {
  public:
    Feature_Func_Kernel_Object( const bool& invert = false, const unsigned int& objectType = 0 );
    virtual ~Feature_Func_Kernel_Object();
    Feature_Func_Kernel_Object( const Feature_Func_Kernel_Object& other );
    Feature_Func_Kernel_Object& operator=( const Feature_Func_Kernel_Object& other );

    virtual bool value( const unsigned int& cv, const h2sl::Grounding* grounding, const std::vector< h2sl::Grounding* >& children, const h2sl::Phrase* phrase, const h2sl::World* world );

    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( xmlNodePtr root );

    inline unsigned int& object_type( void ){ return _object_type; };
    inline const unsigned int& object_type( void )const{ return _object_type; };

    virtual inline const h2sl::feature_type_t type( void )const{ return h2sl::FEATURE_TYPE_GROUNDING; };

  protected:
    unsigned int _object_type;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Feature_Func_Kernel_Object& other );
}

#endif /* H2SL_FEATURE_FUNC_KERNEL_OBJECT_H */

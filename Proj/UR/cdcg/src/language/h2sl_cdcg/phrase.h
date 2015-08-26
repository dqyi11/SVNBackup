/**
 * @file    phrase.h
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
 * The interface for a class used to represent a phrase
 */

#ifndef H2SL_PHRASE_H_CDCG
#define H2SL_PHRASE_H_CDCG

#include <h2sl/phrase.h>

namespace h2sl_cdcg {

  class Phrase : public h2sl::Phrase {
  public:
    Phrase( const h2sl::phrase_type_t& type = h2sl::PHRASE_UNKNOWN, const std::string& text = "na", const std::vector< h2sl::Word >& words = std::vector< h2sl::Word >(), const std::vector< Phrase* >& children = std::vector< Phrase* >(), h2sl::Grounding* grounding = NULL );
    virtual ~Phrase();
    Phrase( const Phrase& other );
    Phrase& operator=( const Phrase& other );
    bool operator==( const Phrase& other )const;
    bool operator!=( const Phrase& other )const;

    Phrase* dup( void )const;
    Phrase* dup( const bool& empty )const;

    virtual void to_xml( const std::string& filename )const;
    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( const std::string& filename );
    virtual void from_xml( xmlNodePtr root );


  protected:

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Phrase& other );
}

#endif /* H2SL_PHRASE_H_CDCG */

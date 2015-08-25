/**
 * @file    dcg_demo_cdcg.cc
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * A DCG class demo program
 */

#include <iostream>
#include <iomanip>
#include <sys/time.h>
#include "h2sl/common.h"
#include "h2sl/phrase.h"
#include "h2sl/constraint.h"
#include "h2sl/parser_cyk.h"
#include "h2sl_cdcg/dcg.h"
#include "h2sl_cdcg/feature_set.h"
#include "dcg_demo_cdcg_cmdline.h"

using namespace std;
using namespace h2sl;
using namespace h2sl_cdcg;

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of DCG class demo program" << endl;
  
  gengetopt_args_info args;
  if( cmdline_parser( argc, argv, &args ) != 0 ){
    exit(1);
  }

  Parser< Phrase > * parser = new Parser_CYK< Phrase >();
  Grammar * grammar = new Grammar();
  grammar->from_xml( args.grammar_arg );

  vector< Phrase* > phrases;

  World * world = new World();
  if( args.world_given ){
    world->from_xml( args.world_arg );
  }

  h2sl_cdcg::Feature_Set * feature_set = new h2sl_cdcg::Feature_Set();
  LLM * llm = new LLM( feature_set );
  if( args.llm_given ){
    llm->from_xml( args.llm_arg );
  }

  h2sl_cdcg::DCG * dcg = new h2sl_cdcg::DCG();
  
  struct timeval start_time;
  gettimeofday( &start_time, NULL );

  dcg->fill_search_spaces( world );

  struct timeval end_time;
  gettimeofday( &end_time, NULL );

  cout << "finished fill_seach_space in " << diff_time( start_time, end_time ) << " seconds" << endl;

  cout << endl << "search_spaces.size(): " << dcg->search_spaces().size() << endl << endl;

  cout << "parsing \"" << args.command_arg << "\"" << endl;
  if( parser->parse( *grammar, args.command_arg, phrases ) ){
    for( unsigned int i = 0; i < phrases.size(); i++ ){
      if( phrases[ i ] != NULL ){
        cout << "phrases[" << i << "]:" << *phrases[ i ] << endl;
    
        gettimeofday( &start_time, NULL );

        dcg->leaf_search( phrases[ i ], world, llm, args.beam_width_arg );

        gettimeofday( &end_time, NULL );

        cout << "finished search in " << diff_time( start_time, end_time ) << " seconds" << endl;   
        for( unsigned int j = 0; j < dcg->solutions().size(); j++ ){
          cout << "  solutions[" << j << "]:" << *dcg->solutions()[ j ].second << " (" << dcg->solutions()[ j ].first << ")" << endl;
        }
   
        if( args.output_given ){
          string filename = args.output_arg;
          if( filename.find( ".xml" ) != string::npos ){
            if( phrases.size() == 1 ){
              dcg->solutions().front().second->to_xml( filename );
            } else {
              boost::trim_if( filename, boost::is_any_of( ".xml" ) );
              stringstream tmp;
              tmp << filename << "_" << setw( 4 ) << setfill( '0' ) << i << ".xml";
              dcg->solutions().front().second->to_xml( tmp.str() );
            }
          }
        }

        if( args.latex_output_given ){
          string filename = args.latex_output_arg;
          if( filename.find( ".tex" ) != string::npos ){
            if( phrases.size() == 1 ){
              dcg->to_latex( filename );
            } else {
              boost::trim_if( filename, boost::is_any_of( ".tex" ) );
              stringstream tmp;
              tmp << filename << "_" << setw( 4 ) << setfill( '0' ) << i << ".tex";
              dcg->to_latex( tmp.str() );
            }
          }
        } 
      }
    }
  }

  if( dcg != NULL ){
    delete dcg;
    dcg = NULL;
  }

  if( llm != NULL ){
    delete llm;
    llm = NULL;
  }

  if( feature_set != NULL ){
    delete feature_set;
    feature_set = NULL;
  }

  if( world != NULL ){
    delete world;
    world = NULL;
  }

  for( unsigned int i = 0; i < phrases.size(); i++ ){
    if( phrases[ i ] != NULL ){
      delete phrases[ i ];
      phrases[ i ] = NULL;
    }
  }

  if( grammar != NULL ){
    delete grammar;
    grammar = NULL;
  }

  if( parser != NULL ){
    delete parser;
    parser = NULL;
  }

  cout << "end of DCG class demo program" << endl;
  return status;
}

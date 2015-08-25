/**
 * @file    gui_demo_cdcg.cc
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * A GUI class demo program
 */

#include <iostream>

#include <QtGui/QApplication>

#include "h2sl/parser_cyk.h"
#include "h2sl_cdcg/feature_set.h"
#include "h2sl_cdcg/dcg.h"
#include "h2sl/gui.h"
#include "gui_demo_cdcg_cmdline.h"

using namespace std;
using namespace h2sl;
using namespace h2sl_cdcg;

int
main( int argc,
      char* argv[] ) {
  gengetopt_args_info args;
  if( cmdline_parser( argc, argv, &args ) != 0 ){
    exit(1);
  }
  
  cout << "start of GUI class demo program" << endl;

  QApplication app( argc, argv );

  Parser< Phrase > * parser = new Parser_CYK< Phrase >();
  Grammar * grammar = new Grammar();
  grammar->from_xml( args.grammar_arg );

  World * world = new World();
  world->from_xml( args.world_arg );

  h2sl_cdcg::Feature_Set * feature_set = new h2sl_cdcg::Feature_Set();

  LLM * llm = new LLM( feature_set );
  if( args.llm_given ){
    llm->from_xml( args.llm_arg );
  }

  h2sl_cdcg::DCG * dcg = new h2sl_cdcg::DCG();

  GUI gui( grammar, parser, world, llm, dcg, args.beam_width_arg );

  if( args.phrase_given ){
    Phrase * phrase = new Phrase();
    phrase->from_xml( args.phrase_arg );
  
    if( phrase != NULL ){
      delete phrase;
      phrase = NULL;
    }
  }

  gui.show(); 

  return app.exec();
}

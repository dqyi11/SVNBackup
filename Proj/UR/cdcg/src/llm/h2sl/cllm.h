/**
 * @file    cllm.h
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to represent a log-linear model that 
 * support continuous correspondence variables
 */

#ifndef H2SL_CLLM_H
#define H2SL_CLLM_H

#include <iostream>
#include <vector>
#include <libxml/tree.h>

#include <h2sl/grounding.h>
#include <h2sl/cv.h>
#include <h2sl/feature_set.h>
#include <h2sl/llm.h>

namespace h2sl {
  class CLLM_X {
  public:
    CLLM_X();
    virtual ~CLLM_X();
    CLLM_X( const CLLM_X& other );
    CLLM_X& operator=( const CLLM_X& other );
  
    inline Grounding*& grounding( void ){ return _grounding; };
    inline const Grounding* grounding( void )const{ return _grounding; };
    inline std::vector< Grounding* >& children( void ){ return _children; };   
    inline const std::vector< Grounding* >& children( void )const{ return _children; };
    inline Phrase*& phrase( void ){ return _phrase; };
    inline const Phrase* phrase( void )const{ return _phrase; }; 
    inline World*& world( void ){ return _world; };
    inline const World* world( void )const{ return _world; }; 
    inline std::vector< double  >& cvs( void ){ return _cvs; };
    inline const std::vector< double >& cvs( void )const{ return _cvs; };

  protected:
    Grounding* _grounding;
    std::vector< Grounding* > _children;
    Phrase* _phrase;
    World* _world;
    std::vector< double > _cvs;
  };
  std::ostream& operator<<( std::ostream& out, const CLLM_X& other );

  class CLLM_Train;

  class CLLM {
  public:
    CLLM( Feature_Set* featureSet = NULL );
    virtual ~CLLM();
    CLLM( const CLLM& other );
    CLLM& operator=( const CLLM& other );

    double pygx( const double& cv, const CLLM_X& x, const std::vector< double >& cvs, const std::vector< std::vector< unsigned int > >& indices );
    double pygx( const double& cv, const CLLM_X& x, const std::vector< double >& cvs, std::vector< unsigned int >& indices );
    double pygx( const double& cv, const CLLM_X& x, const std::vector< double >& cvs );
    double pygx( const double& cv, const Grounding* grounding, const std::vector< Grounding* >& children, const Phrase* phrase, const World* world, const std::vector< double >& cvs );
    double pygx( const double& cv, const Grounding* grounding, const std::vector< Grounding* >& children, const Phrase* phrase, const World* world, const std::vector< double >& cvs, const std::vector< bool >& evaluateFeatureTypes );
    void train( std::vector< std::pair< unsigned int, CLLM_X > >& examples, const unsigned int& maxIterations = 100, const double& lambda = 0.01, const double& epsilon = 0.001 );

    static double objective( CLLM* llm, const std::vector< std::pair< unsigned int, CLLM_X > >& examples, const std::vector< std::vector< std::vector< unsigned int > > >& indices, double lambda );
    static void gradient( CLLM* llm, const std::vector< std::pair< unsigned int, CLLM_X > >& examples, const std::vector< std::vector< std::vector< unsigned int > > >& indices, std::vector< double >& g, double lambda );

    virtual void to_xml( const std::string& filename )const;
    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( const std::string& filename );
    virtual void from_xml( xmlNodePtr root );

    inline std::vector< double >& weights( void ){ return _weights; };
    inline const std::vector< double >& weights( void )const{ return _weights; };
    inline Feature_Set*& feature_set( void ){ return _feature_set; };
    inline const Feature_Set* feature_set( void )const{ return _feature_set; };

  protected:
    std::vector< double > _weights;
    Feature_Set* _feature_set;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const CLLM& other );

  class CLLM_Train {
  public:
    CLLM_Train( CLLM* llm = NULL, std::vector< std::pair< unsigned int, CLLM_X > >* examples = NULL );  
    ~CLLM_Train();
    CLLM_Train( const CLLM_Train& other );
    CLLM_Train& operator=( const CLLM_Train& other );
  
    void compute_indices( void );

    inline CLLM*& llm( void ){ return _llm; };
    inline std::vector< std::pair< unsigned int, CLLM_X > >*& examples( void ){ return _examples; };
    inline std::vector< std::vector< std::vector< unsigned int > > >& indices( void ){ return _indices; };

  protected:
    CLLM* _llm;
    std::vector< std::pair< unsigned int, CLLM_X > >* _examples;
    std::vector< std::vector< std::vector< unsigned int > > > _indices;
  };
}

#endif /* H2SL_CLLM_H */

/**
 * @file    ccv.h
 * @author  Daqing Yi (daqing.yi@byu.edu)
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to describe a correspondence variable
 */

#ifndef H2SL_CCV_H_CDCG
#define H2SL_CCV_H_CDCG

#include <iostream>

#define APPROX_RESOLUTION 5

namespace h2sl {
  typedef enum {
    CCV_UNKNOWN,
    CCV_ZERO,
    CCV_ONE,
    CCV_TWO,
    CCV_THREE,
    CCV_FOUR,
    CCV_FIVE,
    NUM_CCVS
  } ccv_t;
}

#endif /* H2SL_CCV_H_CDCG */

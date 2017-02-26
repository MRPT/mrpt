/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "cs.h"
/* create a Householder reflection [v,beta,s]=house(x), overwrite x with v,
 * where (I-beta*v*v')*x = s*e1.  See Algo 5.1.1, Golub & Van Loan, 3rd ed. */
double cs_house (double *x, double *beta, int n)
{
    double s, sigma = 0 ;
    int i ;
    if (!x || !beta) return (-1) ;          /* check inputs */
    for (i = 1 ; i < n ; i++) sigma += x [i] * x [i] ;
    if (sigma == 0)
    {
        s = fabs (x [0]) ;                  /* s = |x(0)| */
        (*beta) = (x [0] <= 0) ? 2 : 0 ;
        x [0] = 1 ;
    }
    else
    {
        s = sqrt (x [0] * x [0] + sigma) ;  /* s = norm (x) */
        x [0] = (x [0] <= 0) ? (x [0] - s) : (-sigma / (x [0] + s)) ;
        (*beta) = -1. / (s * x [0]) ;
    }
    return (s) ;
}

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "cs.h"
/* x = x + beta * A(:,j), where x is a dense vector and A(:,j) is sparse */
int cs_scatter (const cs *A, int j, double beta, int *w, double *x, int mark,
    cs *C, int nz)
{
    int i, p, *Ap, *Ai, *Ci ;
    double *Ax ;
    if (!CS_CSC (A) || !w || !CS_CSC (C)) return (-1) ;     /* check inputs */
    Ap = A->p ; Ai = A->i ; Ax = A->x ; Ci = C->i ;
	// JLBC: Modification for MRPT: optimized case for beta==1:
	if (beta==1)
	{
		for (p = Ap [j] ; p < Ap [j+1] ; p++)
		{
			i = Ai [p] ;                            /* A(i,j) is nonzero */
			if (w [i] < mark)
			{
				w [i] = mark ;                      /* i is new entry in column j */
				Ci [nz++] = i ;                     /* add i to pattern of C(:,j) */
				if (x) x [i] = /* beta * */ Ax [p] ;      /* x(i) = beta*A(i,j) */
			}
			else if (x) x [i] += /*beta * */ Ax [p] ;    /* i exists in C(:,j) already */
		}
	}
	else
	{
		for (p = Ap [j] ; p < Ap [j+1] ; p++)
		{
			i = Ai [p] ;                            /* A(i,j) is nonzero */
			if (w [i] < mark)
			{
				w [i] = mark ;                      /* i is new entry in column j */
				Ci [nz++] = i ;                     /* add i to pattern of C(:,j) */
				if (x) x [i] = beta * Ax [p] ;      /* x(i) = beta*A(i,j) */
			}
			else if (x) x [i] += beta * Ax [p] ;    /* i exists in C(:,j) already */
		}
	}
    return (nz) ;
}

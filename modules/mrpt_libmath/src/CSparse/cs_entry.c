// CSparse/Source/cs_entry: add an entry to a sparse matrix in triplet-form
// CSparse, Copyright (c) 2006-2023, Timothy A. Davis. All Rights Reserved.
// SPDX-License-Identifier: LGPL-2.1+
#include "cs.h"
/* add an entry to a triplet matrix; return 1 if ok, 0 otherwise */
int cs_entry (cs *T, int i, int j, double x)
{
    if (!CS_TRIPLET (T) || i < 0 || j < 0) return (0) ;     /* check inputs */
    if (T->nz >= T->nzmax && !cs_sprealloc (T,2*(T->nzmax))) return (0) ;
    if (T->x) T->x [T->nz] = x ;
    T->i [T->nz] = i ;
    T->p [T->nz++] = j ;
    T->m = CS_MAX (T->m, i+1) ;
    T->n = CS_MAX (T->n, j+1) ;
    return (1) ;
}

//! Added by JLBC for MRPT: Just like cs_entry but doesn't extend the matrix limits.
int cs_entry_no_extend (cs *T, int i, int j, double x)
{
//    if (!CS_TRIPLET (T) || i < 0 || j < 0) return (0) ;     /* check inputs */
    if (T->nz >= T->nzmax && !cs_sprealloc (T,2*(T->nzmax))) return (0) ;
    if (T->x) T->x [T->nz] = x ;
    T->i [T->nz] = i ;
    T->p [T->nz++] = j ;
//    T->m = CS_MAX (T->m, i+1) ;
//   T->n = CS_MAX (T->n, j+1) ;
    return (1) ;
}

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+ */
#include "cs.h"
/* allocate a sparse matrix (triplet form or compressed-column form) */
cs *cs_spalloc (int m, int n, int nzmax, int values, int triplet)
{
    cs *A = cs_calloc (1, sizeof (cs)) ;    /* allocate the cs struct */
    if (!A) return (NULL) ;                 /* out of memory */
    A->m = m ;                              /* define dimensions and nzmax */
    A->n = n ;
    A->nzmax = nzmax = CS_MAX (nzmax, 1) ;
    A->nz = triplet ? 0 : -1 ;              /* allocate triplet or comp.col */
    A->p = cs_malloc (triplet ? nzmax : n+1, sizeof (int)) ;
    A->i = cs_malloc (nzmax, sizeof (int)) ;
    A->x = values ? cs_malloc (nzmax, sizeof (double)) : NULL ;
    return ((!A->p || !A->i || (values && !A->x)) ? cs_spfree (A) : A) ;
}

/* change the max # of entries sparse matrix */
int cs_sprealloc (cs *A, int nzmax)
{
    int ok, oki, okj = 1, okx = 1 ;
    if (!A) return (0) ;
    if (nzmax <= 0) nzmax = (CS_CSC (A)) ? (A->p [A->n]) : A->nz ;
    A->i = cs_realloc (A->i, nzmax, sizeof (int), &oki) ;
    if (CS_TRIPLET (A)) A->p = cs_realloc (A->p, nzmax, sizeof (int), &okj) ;
    if (A->x) A->x = cs_realloc (A->x, nzmax, sizeof (double), &okx) ;
    ok = (oki && okj && okx) ;
    if (ok) A->nzmax = nzmax ;
    return (ok) ;
}

/* free a sparse matrix */
cs *cs_spfree (cs *A)
{
    if (!A) return (NULL) ;     /* do nothing if A already NULL */
    cs_free (A->p) ;
    cs_free (A->i) ;
    cs_free (A->x) ;
    return (cs_free (A)) ;      /* free the cs struct and return NULL */
}

/* free a numeric factorization */
csn *cs_nfree (csn *N)
{
    if (!N) return (NULL) ;     /* do nothing if N already NULL */
    cs_spfree (N->L) ;
    cs_spfree (N->U) ;
    cs_free (N->pinv) ;
    cs_free (N->B) ;
    return (cs_free (N)) ;      /* free the csn struct and return NULL */
}

/* free a symbolic factorization */
css *cs_sfree (css *S)
{
    if (!S) return (NULL) ;     /* do nothing if S already NULL */
    cs_free (S->pinv) ;
    cs_free (S->q) ;
    cs_free (S->parent) ;
    cs_free (S->cp) ;
    cs_free (S->leftmost) ;
    return (cs_free (S)) ;      /* free the css struct and return NULL */
}

/* allocate a cs_dmperm or cs_scc result */
csd *cs_dalloc (int m, int n)
{
    csd *D ;
    D = cs_calloc (1, sizeof (csd)) ;
    if (!D) return (NULL) ;
    D->p = cs_malloc (m, sizeof (int)) ;
    D->r = cs_malloc (m+6, sizeof (int)) ;
    D->q = cs_malloc (n, sizeof (int)) ;
    D->s = cs_malloc (n+6, sizeof (int)) ;
    return ((!D->p || !D->r || !D->q || !D->s) ? cs_dfree (D) : D) ;
}

/* free a cs_dmperm or cs_scc result */
csd *cs_dfree (csd *D)
{
    if (!D) return (NULL) ;     /* do nothing if D already NULL */
    cs_free (D->p) ;
    cs_free (D->q) ;
    cs_free (D->r) ;
    cs_free (D->s) ;
    return (cs_free (D)) ;
}

/* free workspace and return a sparse matrix result */
cs *cs_done (cs *C, void *w, void *x, int ok)
{
    cs_free (w) ;                       /* free workspace */
    cs_free (x) ;
    return (ok ? C : cs_spfree (C)) ;   /* return result if OK, else free it */
}

/* free workspace and return int array result */
int *cs_idone (int *p, cs *C, void *w, int ok)
{
    cs_spfree (C) ;                     /* free temporary matrix */
    cs_free (w) ;                       /* free workspace */
    return (ok ? p : cs_free (p)) ;     /* return result if OK, else free it */
}

/* free workspace and return a numeric factorization (Cholesky, LU, or QR) */
csn *cs_ndone (csn *N, cs *C, void *w, void *x, int ok)
{
    cs_spfree (C) ;                     /* free temporary matrix */
    cs_free (w) ;                       /* free workspace */
    cs_free (x) ;
    return (ok ? N : cs_nfree (N)) ;    /* return result if OK, else free it */
}

/* free workspace and return a csd result */
csd *cs_ddone (csd *D, cs *C, void *w, int ok)
{
    cs_spfree (C) ;                     /* free temporary matrix */
    cs_free (w) ;                       /* free workspace */
    return (ok ? D : cs_dfree (D)) ;    /* return result if OK, else free it */
}

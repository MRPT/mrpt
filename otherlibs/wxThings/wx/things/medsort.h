/////////////////////////////////////////////////////////////////////////////
// Name:        medsort.h
// Purpose:     Macros of generic public domain median sorting algorithms
// Author:      John Labenski & mostly others
// Created:     07/01/02
// Copyright:   see macro headers, rewritten by John Labenski, 2002
// License:     Public Domain
/////////////////////////////////////////////////////////////////////////////

#ifndef __WX_MEDSORT_H__
#define __WX_MEDSORT_H__

/*
    Notes :

    code taken from http://ndevilla.free.fr/median/
    see the headers for each function taken from the files
    see bottom for benchmark data

    Each of these functions are implemented as macros that can be used to either
    DECLARE and DEFINE functions for the different element types required
    or they can be used inline

    for example:

    in a header file declare the function with
    DECLARE_WIRTHS_MEDIAN( wirths_median_int, int )
    and you'll get this code
    int wirths_median_int( int *arr, int n, int &median );

    in the c(pp) file define the function
    DEFINE_WIRTHS_MEDIAN( wirths_median_int, int )
    and get
    int withs_median_int( int *arr, int n, int &median )
    { the function itself }

    otherwise use the macro inline in some other function
    IMPLEMENT_WIRTHS_MEDIAN(int, arr, n, &median)
*/


/*---------------------------------------------------------------------------
 *  This Quickselect routine is based on the algorithm described in
 *  "Numerical recipes in C", Second Edition,
 *  Cambridge University Press, 1992, Section 8.5, ISBN 0-521-43108-5
 *  This code by Nicolas Devillard - 1998. Public domain.
 ----------------------------------------------------------------------------

    This does NOT fully sort the input array, but it does modify it

    QUICK_SELECT(elem_type, arr, n, median)
    elem_type is a valid data type, int, long, unsigned char, float...
    arr is an array delcared as elem_type *array and passed as array
    n is the size of the array, ie. total element count
    median contains the median value on exit

---------------------------------------------------------------------------*/

#define DECLARE_QUICK_SELECT( name, elem_type )                             \
    elem_type name( elem_type *arr, int n, elem_type &median );

#define DEFINE_QUICK_SELECT( name, elem_type )                              \
    elem_type name( elem_type *arr, int n, elem_type &median )              \
    IMPLEMENT_QUICK_SELECT( elem_type, arr, n, median )

#define IMPLEMENT_QUICK_SELECT(elem_type, arr, n, median)                   \
{                                                                           \
    int low=0, high=n-1, half=(low+high)/2, middle, ll, hh;                 \
                                                                            \
    for (;;) {                                                              \
        if (high <= low) /* One element only */                             \
            break; /*return arr[half] ; */                                  \
                                                                            \
        if (high == low + 1) {  /* Two elements only */                     \
            if (arr[low] > arr[high])                                       \
                { register elem_type t=arr[low];arr[low]=arr[high];arr[high]=t; } \
            break; /* return arr[half] ; */                                 \
        }                                                                   \
                                                                            \
        /* Find median of low, middle and high items; swap into low */      \
        middle = (low + high) / 2;                                          \
        if (arr[middle] > arr[high]) { register elem_type t=arr[middle];arr[middle]=arr[high];arr[high]=t; } \
        if (arr[low]    > arr[high]) { register elem_type t=arr[low];   arr[low]   =arr[high];arr[high]=t; } \
        if (arr[middle] > arr[low] ) { register elem_type t=arr[middle];arr[middle]=arr[low]; arr[low] =t; } \
                                                                            \
        /* Swap low item (now in position middle) into position (low+1) */  \
        { register elem_type t=arr[middle];arr[middle]=arr[low+1];arr[low+1]=t; } \
                                                                            \
        /* Nibble from ends towards middle, swapping items when stuck */    \
        ll = low + 1;                                                       \
        hh = high;                                                          \
        for (;;) {                                                          \
            do ll++; while (arr[low] > arr[ll] );                           \
            do hh--; while (arr[hh]  > arr[low]);                           \
                                                                            \
            if (hh < ll) break;                                             \
                                                                            \
            { register elem_type t=arr[ll];arr[ll]=arr[hh];arr[hh]=t;}      \
        }                                                                   \
                                                                            \
        /* Swap middle item (in position low) back into correct position */ \
        { register elem_type t=arr[low];arr[low]=arr[hh];arr[hh]=t; }       \
                                                                            \
        /* Re-set active partition */                                       \
        if (hh <= half) low = ll;                                           \
        if (hh >= half) high = hh - 1;                                      \
    }                                                                       \
    median = arr[half];                                                     \
}


/*---------------------------------------------------------------------------
   Function :   kth_smallest()
   In       :   array of elements, # of elements in the array, rank k
   Out      :   one element
   Job      :   find the kth smallest element in the array
   Notice   :   use the median() macro defined below to get the median.

                Reference:

                  Author: Wirth, Niklaus
                   Title: Algorithms + data structures = programs
               Publisher: Englewood Cliffs: Prentice-Hall, 1976
    Physical description: 366 p.
                  Series: Prentice-Hall Series in Automatic Computation
 ---------------------------------------------------------------------------

    This does NOT fully sort the input array, but it does modify it

    WIRTHS_KTH_SMALLEST(elem_type, arr, n, k, ksmallest)
    elem_type is a valid data type, int, long, unsigned char, float...
    arr is an array delcared as elem_type *array and passed as array
    n is the size of the array, ie. total element count
    k is the kth smallest value of the array that you want to find
    ksmallest contains the kth smallest value of arr on exit

    WIRTHS_MEDIAN(elem_type, arr, n, median) finds median value
    Calls WIRTHS_KTH_SMALLEST with (k = n/2) fills median

---------------------------------------------------------------------------*/

#define DECLARE_WIRTHS_MEDIAN( name, elem_type )                \
    elem_type name( elem_type *arr, int n, elem_type &median );

#define DEFINE_WIRTHS_MEDIAN( name, elem_type )                 \
    elem_type name( elem_type *arr, int n, elem_type &median )  \
    WIRTHS_MEDIAN( elem_type, arr, n, median )

#define IMPLEMENT_WIRTHS_MEDIAN(elem_type, arr, n, median)      \
    IMPLEMENT_WIRTHS_KTH_SMALLEST(elem_type, arr, n, (((n)&1)?((n)/2):(((n)/2)-1)), median)


#define DECLARE_WIRTHS_KTH_SMALLEST( name, elem_type )          \
    elem_type name( elem_type *arr, int n, elem_type &median );

#define DEFINE_WIRTHS_KTH_SMALLEST( name, elem_type )           \
    elem_type name( elem_type *arr, int n, elem_type &median )  \
    IMPLEMENT_WIRTHS_MEDIAN( elem_type, arr, n, median )

#define IMPLEMENT_WIRTHS_KTH_SMALLEST(elem_type, arr, n, k, ksmallest) \
{                                                               \
    register int i, j, l=0, m=n-1;                              \
    register elem_type x;                                       \
                                                                \
    while (l<m) {                                               \
        x=arr[k]; i=l; j=m;                                     \
        do {                                                    \
            while (arr[i]<x) i++;                               \
            while (x<arr[j]) j--;                               \
            if (i<=j) {                                         \
                {   register elem_type t=arr[i];                \
                    arr[i]=arr[j]; arr[j]=t; }                  \
                i++; j--; }                                     \
        } while (i<=j);                                         \
        if (j<k) l=i;                                           \
        if (k<i) m=j;                                           \
    }                                                           \
    ksmallest = arr[k];                                         \
}


/*---------------------------------------------------------------------------
 * The following code is public domain.
 * Algorithm by Torben Mogensen, implementation by N. Devillard.
 * This code in public domain.
---------------------------------------------------------------------------

    This does NOT modify NOR sort the input array

    TORBEN_MEDIAN(elem_type, arr, n )
    elem_type is a valid data type, int, long, unsigned char, float...
    arr is an array delcared as elem_type *array and passed as array
    n is the size of the array, ie. total element count

---------------------------------------------------------------------------*/

#define DECLARE_TORBEN_MEDIAN( name, elem_type )                \
    elem_type name( elem_type *arr, int n, elem_type &median );

#define DEFINE_TORBEN_MEDIAN( name, elem_type )                 \
    elem_type name( elem_type *arr, int n, elem_type &median )  \
    IMPLEMENT_TORBEN_MEDIAN( elem_type, arr, n, median )

#define IMPLEMENT_TORBEN_MEDIAN( elem_type, arr, n )                      \
{                                                               \
    int i, less, greater, equal;                                \
    elem_type  min, max, guess, maxltguess, mingtguess;         \
                                                                \
    min = max = arr[0];                                         \
    for (i=1; i<n; i++) {                                       \
        if (arr[i]<min) min=arr[i];                             \
        if (arr[i]>max) max=arr[i];                             \
    }                                                           \
                                                                \
    while (1) {                                                 \
        guess = (min+max)/2;                                    \
        less = 0; greater = 0; equal = 0;                       \
        maxltguess = min;                                       \
        mingtguess = max;                                       \
        for (i=0; i<n; i++) {                                   \
            if (arr[i]<guess) {                                 \
                less++;                                         \
                if (arr[i]>maxltguess) maxltguess = arr[i];     \
            } else if (arr[i]>guess) {                          \
                greater++;                                      \
                if (arr[i]<mingtguess) mingtguess = arr[i];     \
            } else equal++;                                     \
        }                                                       \
        if (less <= (n+1)/2 && greater <= (n+1)/2) break;       \
        else if (less>greater) max = maxltguess;                \
        else min = mingtguess;                                  \
    }                                                           \
    if (less >= (n+1)/2) median = maxltguess;                   \
    else if (less+equal >= (n+1)/2) median = guess;             \
    else median = mingtguess;                                   \
}

/*----------------------------------------------------------------------------
    Function :   pixel_qsort()
    In       :   pixel array, size of the array
    Out      :   void
    Job      :   sort out the array of pixels
    Notice   :   optimized implementation, unreadable.
---------------------------------------------------------------------------

    This fully sorts the input array by modifying it

    DECLARE_PIXEL_QSORT(name, elem_type)

----------------------------------------------------------------------------*/

#define PIXEL_QSORT_STACK_SIZE 50
#define PIXEL_QSORT_THRESHOLD 7

#define DECLARE_PIXEL_QSORT( name, elem_type )                              \
    void name( elem_type *arr, int n );

#define DEFINE_PIXEL_QSORT( name, elem_type )                               \
    void name( elem_type *arr, int n )                                      \
    IMPLEMENT_PIXEL_QSORT( name, elem_type )

#define IMPLEMENT_PIXEL_QSORT( elem_type, arr, n )                          \
{                                                                           \
    int i, ir=n, j, k, l=1, j_stack=0;                                      \
    int *i_stack ;                                                          \
    elem_type  a ;                                                          \
                                                                            \
    i_stack = (int*)malloc(PIXEL_QSORT_STACK_SIZE * sizeof(elem_type));     \
    for (;;) {                                                              \
        if (ir-l < PIXEL_QSORT_THRESHOLD) {                                 \
            for (j=l+1 ; j<=ir ; j++) {                                     \
                a = arr[j-1];                                               \
                for (i=j-1 ; i>=1 ; i--) {                                  \
                    if (arr[i-1] <= a) break;                               \
                    arr[i] = arr[i-1];                                      \
                }                                                           \
                arr[i] = a;                                                 \
            }                                                               \
            if (j_stack == 0) break;                                        \
            ir = i_stack[j_stack-- -1];                                     \
            l  = i_stack[j_stack-- -1];                                     \
        } else {                                                            \
            k = (l+ir) >> 1;                                                \
            { elem_type t=arr[k-1];arr[k-1]=arr[l];arr[l]=t; }              \
            if (arr[l] > arr[ir-1]) {                                       \
                { elem_type t=arr[l];arr[l]=arr[ir-1];arr[ir-1]=t; }        \
            }                                                               \
            if (arr[l-1] > arr[ir-1]) {                                     \
                { elem_type t=arr[l-1];arr[l-1]=arr[ir-1];arr[ir-1]=t; }    \
            }                                                               \
            if (arr[l] > arr[l-1]) {                                        \
                { elem_type t=arr[l];arr[l]=arr[l-1];arr[l-1]=t; }          \
            }                                                               \
            i = l+1; j = ir; a = arr[l-1];                                  \
            for (;;) {                                                      \
                do i++; while (arr[i-1] < a);                               \
                do j--; while (arr[j-1] > a);                               \
                if (j < i) break;                                           \
                { elem_type t=arr[i-1];arr[i-1]=arr[j-1];arr[j-1]=t; }      \
            }                                                               \
            arr[l-1] = arr[j-1];                                            \
            arr[j-1] = a;                                                   \
            j_stack += 2;                                                   \
            wxASSERT(!(j_stack>PIXEL_QSORT_STACK_SIZE));                    \
            /* if (j_stack > PIXEL_QSORT_STACK_SIZE) {  */                  \
                /* printf("stack too small in pixel_qsort: aborting"); */   \
                /* exit(-2001); } */                                        \
            if (ir-i+1 >= j-l) {                                            \
                i_stack[j_stack-1] = ir;                                    \
                i_stack[j_stack-2] = i;                                     \
                ir = j-1;                                                   \
            } else {                                                        \
                i_stack[j_stack-1] = j-1;                                   \
                i_stack[j_stack-2] = l;                                     \
                l = i;                                                      \
            }                                                               \
        }                                                                   \
    }                                                                       \
    free(i_stack);                                                          \
}


/*-------------------------------------------------------------------------
    Function :   pixel_qsort2()
    In       :   two pixel arrays, size of the arrays
    Out      :   void
    Job      :   sort out both arrays based on the first array
    Notice   :   optimized implementation, unreadable.
---------------------------------------------------------------------------

    This fully sorts the input arrays by modifying them
    Uses the first array as the comparison array

    DECLARE_PIXEL_QSORT2(name, elem_type)

----------------------------------------------------------------------------*/

#define PIXEL_QSORT2_STACK_SIZE 50
#define PIXEL_QSORT2_THRESHOLD 7

#define DECLARE_PIXEL_QSORT2( name, elem_type )                             \
    void name( elem_type *arr, elem_type *arr2, int n );

#define DEFINE_PIXEL_QSORT2( name, elem_type )                              \
    void name( elem_type *arr, elem_type *arr2, int n )                     \
    IMPLEMENT_PIXEL_QSORT2( name, elem_type )

#define IMPLEMENT_PIXEL_QSORT2( elem_type, arr, arr2, n )                   \
{                                                                           \
    int i, ir=n, j, k, l=1, j_stack=0;                                      \
    int *i_stack ;                                                          \
    elem_type  a, a2 ;                                                      \
                                                                            \
    i_stack = (int*)malloc(PIXEL_QSORT2_STACK_SIZE * sizeof(elem_type));    \
    for (;;) {                                                              \
        if (ir-l < PIXEL_QSORT2_THRESHOLD) {                                \
            for (j=l+1 ; j<=ir ; j++) {                                     \
                a = arr[j-1]; a2 = arr2[j-1];                               \
                for (i=j-1 ; i>=1 ; i--) {                                  \
                    if (arr[i-1] <= a) break;                               \
                    arr[i] = arr[i-1];                                      \
                    arr2[i] = arr2[i-1];                                    \
                }                                                           \
                arr[i] = a; arr2[i] = a2;                                   \
            }                                                               \
            if (j_stack == 0) break;                                        \
            ir = i_stack[j_stack-- -1];                                     \
            l  = i_stack[j_stack-- -1];                                     \
        } else {                                                            \
            k = (l+ir) >> 1;                                                \
            { elem_type t=arr[k-1];arr[k-1]=arr[l];arr[l]=t;  t=arr2[k-1];arr2[k-1]=arr2[l];arr2[l]=t; } \
            if (arr[l] > arr[ir-1]) {                                       \
                { elem_type t=arr[l];arr[l]=arr[ir-1];arr[ir-1]=t; t=arr2[l];arr2[l]=arr2[ir-1];arr2[ir-1]=t;} \
            }                                                               \
            if (arr[l-1] > arr[ir-1]) {                                     \
                { elem_type t=arr[l-1];arr[l-1]=arr[ir-1];arr[ir-1]=t; t=arr2[l-1];arr2[l-1]=arr2[ir-1];arr2[ir-1]=t;} \
            }                                                               \
            if (arr[l] > arr[l-1]) {                                        \
                { elem_type t=arr[l];arr[l]=arr[l-1];arr[l-1]=t; t=arr2[l];arr2[l]=arr2[l-1];arr2[l-1]=t; } \
            }                                                               \
            i = l+1; j = ir; a = arr[l-1]; a2 = arr2[l-1];                  \
            for (;;) {                                                      \
                do i++; while (arr[i-1] < a);                               \
                do j--; while (arr[j-1] > a);                               \
                if (j < i) break;                                           \
                { elem_type t=arr[i-1];arr[i-1]=arr[j-1];arr[j-1]=t; t=arr2[i-1];arr2[i-1]=arr2[j-1];arr2[j-1]=t;} \
            }                                                               \
            arr[l-1] = arr[j-1]; arr2[l-1] = arr2[j-1];                     \
            arr[j-1] = a; arr2[j-1] = a2;                                   \
            j_stack += 2;                                                   \
            wxASSERT(!(j_stack>PIXEL_QSORT_STACK_SIZE));                    \
            /* if (j_stack > PIXEL_QSORT_STACK_SIZE) {  */                  \
                /* printf("stack too small in pixel_qsort: aborting"); */   \
                /* exit(-2001); } */                                        \
            if (ir-i+1 >= j-l) {                                            \
                i_stack[j_stack-1] = ir;                                    \
                i_stack[j_stack-2] = i;                                     \
                ir = j-1;                                                   \
            } else {                                                        \
                i_stack[j_stack-1] = j-1;                                   \
                i_stack[j_stack-2] = l;                                     \
                l = i;                                                      \
            }                                                               \
        }                                                                   \
    }                                                                       \
    free(i_stack);                                                          \
}



/*
Abbreviated benchmark data from
http://ndevilla.free.fr/median/median/node13.html

(QuickSelect, Wirth, Aho/Hopcroft/Ullman, Torben, pixel quicksort)
Pentium II 400 MHz running Linux 2.0 with glibc-2.0.7.

The basic method using the libc qsort() function has not been represented here,
because it is so slow compared to the others that it would make the plot unreadable.
Furthermore, it depends on the local implementation of your C library.

Ratios have been obtained for sets with increasing number of values,
from 1e4 to 1e6. The speed ratios have been computed to the fastest method
on average (QuickSelect), then averaged over all measure points.

QuickSelect     : 1.00
WIRTH median    : 1.33
AHU median      : 3.71
Torben          : 8.95
fast pixel sort : 6.50

  Elm   Qselect Wirth   AHU     Torben  pqsort
10000   0.000   0.000   0.010   0.010   0.010
100000  0.010   0.020   0.050   0.140   0.100
200000  0.040   0.040   0.180   0.310   0.220
300000  0.070   0.060   0.190   0.470   0.340
400000  0.080   0.140   0.150   0.630   0.450
500000  0.110   0.080   0.510   0.800   0.580
600000  0.090   0.140   0.320   0.940   0.730
700000  0.120   0.100   0.450   1.100   0.810
800000  0.120   0.160   0.590   1.270   0.940
900000  0.180   0.250   0.760   1.430   1.070
1000000 0.210   0.290   0.600   1.580   1.240
*/

#endif //__WX_MEDSORT_H__

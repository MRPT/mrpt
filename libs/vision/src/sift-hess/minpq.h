/**@file
Functions and structures for implementing a minimizing priority queue.

Copyright (C) 2006  Rob Hess <hess@eecs.oregonstate.edu>
@version 1.1.1-20070913
*/


#ifndef MINPQ_H
#define MINPQ_H

#include <stdlib.h>


/******************************* Defs and macros *****************************/

/* initial # of priority queue elements for which to allocate space */
#define MINPQ_INIT_NALLOCD 512

/********************************** Structures *******************************/

/** an element in a minimizing priority queue */
struct pq_node
{
	void* data;
	int key;
};


/** a minimizing priority queue */
struct min_pq
{
	struct pq_node* pq_array;    /* array containing priority queue */
	int nallocd;                 /* number of elements allocated */
	int n;                       /**< number of elements in pq */
};


/*************************** Function Prototypes *****************************/

/**
Creates a new minimizing priority queue.
*/
extern struct min_pq* minpq_init();


/**
Inserts an element into a minimizing priority queue.

@param min_pq a minimizing priority queue
@param data the data to be inserted
@param key the key to be associated with \a data

@return Returns 0 on success or 1 on failure.
*/
extern int minpq_insert( struct min_pq* min_pq, void* data, int key );


/**
Returns the element of a minimizing priority queue with the smallest key
without removing it from the queue.

@param min_pq a minimizing priority queue

@return Returns the element of \a min_pq with the smallest key or NULL
	if \a min_pq is empty
*/
extern void* minpq_get_min( struct min_pq* min_pq );


/**
Removes and returns the element of a minimizing priority queue with the
smallest key.

@param min_pq a minimizing priority queue

@return Returns the element of \a min_pq with the smallest key of NULL
	if \a min_pq is empty
*/
extern void* minpq_extract_min( struct min_pq* min_pq );


/**
De-allocates the memory held by a minimizing priorioty queue

@param min_pq pointer to a minimizing priority queue
*/
extern void minpq_release( struct min_pq** min_pq );


#endif

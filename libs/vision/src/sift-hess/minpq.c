/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "minpq.h"
#include "utils.h"

#include <limits.h>


/************************* Local Function Prototypes *************************/

void restore_minpq_order( struct pq_node*, int, int );
void decrease_pq_node_key( struct pq_node*, int, int );


/************************** Local Inline Functions ***************************/

/* returns the array index of element i's parent */
static __inline int parent( int i )
{
	return ( i - 1 ) / 2;
}


/* returns the array index of element i's right child */
static __inline int right( int i )
{
	return 2 * i + 2;
}


/* returns the array index of element i's left child */
static __inline int left( int i )
{
	return 2 * i + 1;
}


/********************** Functions prototyped in minpq.h **********************/


/*
Creates a new minimizing priority queue.
*/
struct min_pq* minpq_init()
{
	struct min_pq* min_pq;

	min_pq = malloc( sizeof( struct min_pq ) );
	min_pq->pq_array = calloc( MINPQ_INIT_NALLOCD, sizeof( struct pq_node ) );
	min_pq->nallocd = MINPQ_INIT_NALLOCD;
	min_pq->n = 0;

	return min_pq;
}



/**
Inserts an element into a minimizing priority queue.

@param min_pq a minimizing priority queue
@param data the data to be inserted
@param key the key to be associated with \a data

@return Returns 0 on success or 1 on failure.
*/
int minpq_insert( struct min_pq* min_pq, void* data, int key )
{
	int n = min_pq->n;

	/* double array allocation if necessary */
	if( min_pq->nallocd == n )
	{
		min_pq->nallocd = array_double( &min_pq->pq_array, min_pq->nallocd,
										sizeof( struct pq_node ) );
		if( ! min_pq->nallocd )
		{
			fprintf( stderr, "Warning: unable to allocate memory, %s, line %d\n",
					__FILE__, __LINE__ );
			return 1;
		}
	}

	min_pq->pq_array[n].data = data;
	min_pq->pq_array[n].key = INT_MAX;
	decrease_pq_node_key( min_pq->pq_array, min_pq->n, key );
	min_pq->n++;

	return 0;
}



/*
Returns the element of a minimizing priority queue with the smallest key
without removing it from the queue.

@param min_pq a minimizing priority queue

@return Returns the element of \a min_pq with the smallest key or NULL
if \a min_pq is empty
*/
void* minpq_get_min( struct min_pq* min_pq )
{
	if( min_pq->n < 1 )
	{
		fprintf( stderr, "Warning: PQ empty, %s line %d\n", __FILE__, __LINE__ );
		return NULL;
	}
	return min_pq->pq_array[0].data;
}



/*
Removes and returns the element of a minimizing priority queue with the
smallest key.

@param min_pq a minimizing priority queue

@return Returns the element of \a min_pq with the smallest key of NULL
if \a min_pq is empty
*/
void* minpq_extract_min( struct min_pq* min_pq )
{
	void* data;

	if( min_pq->n < 1 )
	{
		fprintf( stderr, "Warning: PQ empty, %s line %d\n", __FILE__, __LINE__ );
		return NULL;
	}
	data = min_pq->pq_array[0].data;
	min_pq->n--;
	min_pq->pq_array[0] = min_pq->pq_array[min_pq->n];
	restore_minpq_order( min_pq->pq_array, 0, min_pq->n );

	return data;
}


/*
De-allocates the memory held by a minimizing priorioty queue

@param min_pq pointer to a minimizing priority queue
*/
void minpq_release( struct min_pq** min_pq )
{
	if( ! min_pq )
	{
		fprintf( stderr, "Warning: NULL pointer error, %s line %d\n", __FILE__,
				__LINE__ );
		return;
	}
	if( *min_pq  &&  (*min_pq)->pq_array )
	{
		free( (*min_pq)->pq_array );
		free( *min_pq );
		*min_pq = NULL;
	}
}


/************************ Functions prototyped here **************************/

/*
Decrease a minimizing pq element's key, rearranging the pq if necessary

@param pq_array minimizing priority queue array
@param i index of the element whose key is to be decreased
@param key new value of element <EM>i</EM>'s key; if greater than current
	key, no action is taken
*/
void decrease_pq_node_key( struct pq_node* pq_array, int i, int key )
{
	struct pq_node tmp;

	if( key > pq_array[i].key )
		return;

	pq_array[i].key = key;
	while( i > 0  &&  pq_array[i].key < pq_array[parent(i)].key )
	{
		tmp = pq_array[parent(i)];
		pq_array[parent(i)] = pq_array[i];
		pq_array[i] = tmp;
		i = parent(i);
	}
}



/*
Recursively restores correct priority queue order to a minimizing pq array

@param pq_array a minimizing priority queue array
@param i index at which to start reordering
@param n number of elements in \a pq_array
*/
void restore_minpq_order( struct pq_node* pq_array, int i, int n )
{
	struct pq_node tmp;
	int l, r, min = i;

	l = left( i );
	r = right( i );
	if( l < n )
		if( pq_array[l].key < pq_array[i].key )
			min = l;
	if( r < n )
		if( pq_array[r].key < pq_array[min].key )
			min = r;

	if( min != i )
	{
		tmp = pq_array[min];
		pq_array[min] = pq_array[i];
		pq_array[i] = tmp;
		restore_minpq_order( pq_array, min, n );
	}
}

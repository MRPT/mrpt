/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#ifndef KDTREE_H
#define KDTREE_H

//#include "cxcore.h"
// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 


/********************************* Structures ********************************/

struct feature;

/** a node in a k-d tree */
struct kd_node
{
	int ki;                      /**< partition key index */
	double kv;                   /**< partition key value */
	int leaf;                    /**< 1 if node is a leaf, 0 otherwise */
	struct feature* features;    /**< features at this node */
	int n;                       /**< number of features */
	struct kd_node* kd_left;     /**< left child */
	struct kd_node* kd_right;    /**< right child */
};


/*************************** Function Prototypes *****************************/

/**
A function to build a k-d tree database from keypoints in an array.

@param features an array of features
@param n the number of features in \a features

@return Returns the root of a kd tree built from \a features.
*/
extern struct kd_node* kdtree_build( struct feature* features, int n );



/**
Finds an image feature's approximate k nearest neighbors in a kd tree using
Best Bin First search.

@param kd_root root of an image feature kd tree
@param feat image feature for whose neighbors to search
@param k number of neighbors to find
@param nbrs pointer to an array in which to store pointers to neighbors
	in order of increasing descriptor distance
@param max_nn_chks search is cut off after examining this many tree entries

@return Returns the number of neighbors found and stored in \a nbrs, or
	-1 on error.
*/
extern int kdtree_bbf_knn( struct kd_node* kd_root, struct feature* feat,
						  int k, struct feature*** nbrs, int max_nn_chks );


/**
Finds an image feature's approximate k nearest neighbors within a specified
spatial region in a kd tree using Best Bin First search.

@param kd_root root of an image feature kd tree
@param feat image feature for whose neighbors to search
@param k number of neighbors to find
@param nbrs pointer to an array in which to store pointers to neighbors
	in order of increasing descriptor distance
@param max_nn_chks search is cut off after examining this many tree entries
@param rect rectangular region in which to search for neighbors
@param model if true, spatial search is based on kdtree features' model
	locations; otherwise it is based on their image locations

@return Returns the number of neighbors found and stored in \a nbrs
	(in case \a k neighbors could not be found before examining
	\a max_nn_checks keypoint entries).
*/
extern int kdtree_bbf_spatial_knn( struct kd_node* kd_root,
								struct feature* feat, int k,
								struct feature*** nbrs, int max_nn_chks,
								CvRect rect, int model );


/**
De-allocates memory held by a kd tree

@param kd_root pointer to the root of a kd tree
*/
extern void kdtree_release( struct kd_node* kd_root );


#endif

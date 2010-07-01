/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

//----------------------------------------------------------------------
// File:			brute.cpp
// Programmer:		Sunil Arya and David Mount
// Description:		Brute-force nearest neighbors
// Last modified:	05/03/05 (Version 1.1)
//----------------------------------------------------------------------
// Copyright (c) 1997-2005 University of Maryland and Sunil Arya and
// David Mount.  All Rights Reserved.
//
// This software and related documentation is part of the Approximate
// Nearest Neighbor Library (ANN).  This software is provided under
// the provisions of the Lesser GNU Public License (LGPL).  See the
// file ../ReadMe.txt for further information.
//
// The University of Maryland (U.M.) and the authors make no
// representations about the suitability or fitness of this software for
// any purpose.  It is provided "as is" without express or implied
// warranty.
//----------------------------------------------------------------------
// History:
//	Revision 0.1  03/04/98
//		Initial release
//	Revision 1.1  05/03/05
//		Added fixed-radius kNN search
//----------------------------------------------------------------------

#include <ann/ANNx.h>					// all ANN includes
#include <ann/pr_queue_k.h>					// k element priority queue

//----------------------------------------------------------------------
//		Brute-force search simply stores a pointer to the list of
//		data points and searches linearly for the nearest neighbor.
//		The k nearest neighbors are stored in a k-element priority
//		queue (which is implemented in a pretty dumb way as well).
//
//		If ANN_ALLOW_SELF_MATCH is ANNfalse then data points at distance
//		zero are not considered.
//
//		Note that the error bound eps is passed in, but it is ignored.
//		These routines compute exact nearest neighbors (which is needed
//		for validation purposes in ann_test.cpp).
//----------------------------------------------------------------------

ANNbruteForce::ANNbruteForce(			// constructor from point array
	ANNpointArray		pa,				// point array
	int					n,				// number of points
	int					dd)				// dimension
{
	dim = dd;  n_pts = n;  pts = pa;
}

ANNbruteForce::~ANNbruteForce() { }		// destructor (empty)

void ANNbruteForce::annkSearch(			// approx k near neighbor search
	ANNpoint			q,				// query point
	int					k,				// number of near neighbors to return
	ANNidxArray			nn_idx,			// nearest neighbor indices (returned)
	ANNdistArray		dd,				// dist to near neighbors (returned)
	double				eps)			// error bound (ignored)
{
	ANNmin_k mk(k);						// construct a k-limited priority queue
	int i;

	if (k > n_pts) {					// too many near neighbors?
		annError("Requesting more near neighbors than data points", ANNabort);
	}
										// run every point through queue
	for (i = 0; i < n_pts; i++) {
										// compute distance to point
		ANNdist sqDist = annDist(dim, pts[i], q);
		if (ANN_ALLOW_SELF_MATCH || sqDist != 0)
			mk.insert(sqDist, i);
	}
	for (i = 0; i < k; i++) {			// extract the k closest points
		dd[i] = mk.ith_smallest_key(i);
		nn_idx[i] = mk.ith_smallest_info(i);
	}
}

int ANNbruteForce::annkFRSearch(		// approx fixed-radius kNN search
	ANNpoint			q,				// query point
	ANNdist				sqRad,			// squared radius
	int					k,				// number of near neighbors to return
	ANNidxArray			nn_idx,			// nearest neighbor array (returned)
	ANNdistArray		dd,				// dist to near neighbors (returned)
	double				eps)			// error bound
{
	ANNmin_k mk(k);						// construct a k-limited priority queue
	int i;
	int pts_in_range = 0;				// number of points in query range
										// run every point through queue
	for (i = 0; i < n_pts; i++) {
										// compute distance to point
		ANNdist sqDist = annDist(dim, pts[i], q);
		if (sqDist <= sqRad &&			// within radius bound
			(ANN_ALLOW_SELF_MATCH || sqDist != 0)) { // ...and no self match
			mk.insert(sqDist, i);
			pts_in_range++;
		}
	}
	for (i = 0; i < k; i++) {			// extract the k closest points
		if (dd != NULL)
			dd[i] = mk.ith_smallest_key(i);
		if (nn_idx != NULL)
			nn_idx[i] = mk.ith_smallest_info(i);
	}

	return pts_in_range;
}

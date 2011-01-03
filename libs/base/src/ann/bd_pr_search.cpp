/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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
// File:			bd_pr_search.cpp
// Programmer:		David Mount
// Description:		Priority search for bd-trees
// Last modified:	01/04/05 (Version 1.0)
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
//History:
//	Revision 0.1  03/04/98
//		Initial release
//----------------------------------------------------------------------

#include <ann/bd_tree.h>					// bd-tree declarations
#include <ann/kd_pr_search.h>				// kd priority search declarations

//----------------------------------------------------------------------
//	Approximate priority searching for bd-trees.
//		See the file kd_pr_search.cc for general information on the
//		approximate nearest neighbor priority search algorithm.  Here
//		we include the extensions for shrinking nodes.
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//	bd_shrink::ann_search - search a shrinking node
//----------------------------------------------------------------------

void ANNbd_shrink::ann_pri_search(ANNdist box_dist)
{
	ANNdist inner_dist = 0;						// distance to inner box
	for (int i = 0; i < n_bnds; i++) {			// is query point in the box?
		if (bnds[i].out(ANNprQ)) {				// outside this bounding side?
												// add to inner distance
			inner_dist = (ANNdist) ANN_SUM(inner_dist, bnds[i].dist(ANNprQ));
		}
	}
	if (inner_dist <= box_dist) {				// if inner box is closer
		if (child[ANN_OUT] != KD_TRIVIAL)		// enqueue outer if not trivial
			ANNprBoxPQ->insert(box_dist,child[ANN_OUT]);
												// continue with inner child
		child[ANN_IN]->ann_pri_search(inner_dist);
	}
	else {										// if outer box is closer
		if (child[ANN_IN] != KD_TRIVIAL)		// enqueue inner if not trivial
			ANNprBoxPQ->insert(inner_dist,child[ANN_IN]);
												// continue with outer child
		child[ANN_OUT]->ann_pri_search(box_dist);
	}
	ANN_FLOP(3*n_bnds)							// increment floating ops
	ANN_SHR(1)									// one more shrinking node
}

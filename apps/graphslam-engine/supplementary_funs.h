/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/**
 * List of, supplementary to the problem functions. 
 * TODO: Consider integrating them into their respective
 * namespaces/implementation files..
 */

#ifndef SUPPLEMENTARY_FUNS_H
#define SUPPLEMENTARY_FUNS_H


#include <mrpt/graphslam.h>

namespace supplementary_funs {

	// TODO - make this a part of the optimizer
	/** 
	 * levMarqFeedback
	 *
	 * Feedback fucntion for the graph optimization
	 */
	template <
		class GRAPH_T, class NODE_REGISTRAR, class EDGE_REGISTRAR>
		void levMarqFeedback(
				const GRAPH_T &graph,
				const size_t iter,
				const size_t max_iter,
				const double cur_sq_error )
		{ }


} // end of namespace

#endif /* end of include guard: SUPPLEMENTARY_FUNS_H */

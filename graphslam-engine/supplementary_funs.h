/**
 * List of, supplementary to the problem functions. 
 * TODO: Consider integrating them into their respective
 * namespaces/implementation files..
 */

#ifndef SUPPLEMENTARY_FUNS_H
#define SUPPLEMENTARY_FUNS_H


#include <mrpt/graphslam.h>


namespace supplementary_funs {

	/** 
	 * levMarqFeedback
	 *
	 * Feedback fucntion for the graph optimization
	 */
	template <class GRAPH_T>
		void levMarqFeedback(
				const GRAPH_T &graph,
				const size_t iter,
				const size_t max_iter,
				const double cur_sq_error )
		{
		}

}

#endif /* end of include guard: SUPPLEMENTARY_FUNS_H */

#ifndef CVD_NONMAX_SUPPRESSION_H
#define CVD_NONMAX_SUPPRESSION_H

#include <vector>
#include <utility>
#include <cvd/image_ref.h>

namespace CVD
{
	/**Perform nonmaximal suppression on a set of features, in a 3 by 3 window.
	   The test is strict: a point must be greater than its neighbours.

	@param corners The corner locations
	@param scores  The corners' scores
	@param max_corners The locally maximal corners.
	@ingroup gVision
	*/
	void nonmax_suppression_strict(const std::vector<ImageRef>& corners, const std::vector<int>& scores, std::vector<ImageRef>& nmax_corners);
	/**Perform nonmaximal suppression on a set of features, in a 3 by 3 window.
	   The test is non-strict: a point must be at least as large as its neighbours.

	@param corners The corner locations
	@param scores  The corners' scores
	@param max_corners The locally maximal corners.
	@ingroup gVision
	*/
	void nonmax_suppression(const std::vector<ImageRef>& corners, const std::vector<int>& scores, std::vector<ImageRef>& nmax_corners);


	/**Perform nonmaximal suppression on a set of features, in a 3 by 3 window.
	   Non strict.

	@param corners The corner locations
	@param scores  The corners' scores
	@param max_corners The locally maximal corners, and their scores.
	@ingroup gVision
	*/
	void nonmax_suppression_with_scores(const std::vector<ImageRef>& corners, const std::vector<int>& socres, std::vector<std::pair<ImageRef,int> >& max_corners);

}

#endif

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef TRangeImageFilter_H
#define CObservation3DRangeScan_H

#include <mrpt/utils/core_defs.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/obs/link_pragmas.h>

namespace mrpt {
namespace obs {

	/** Used in CObservation3DRangeScan::project3DPointsFromDepthImageInto() */
	struct OBS_IMPEXP TRangeImageFilterParams
	{
		/** Only used if <b>both</b> rangeMask_min and rangeMask_max are present.
		  * This switches which condition must fulfill a range `D` to be accepted as valid:
		  *  - `rangeCheckBetween=true` : valid =  (D>=rangeMask_min && D<=rangeMask_max)
		  *  - `rangeCheckBetween=false`: valid = !(D>=rangeMask_min && D<=rangeMask_max)
		  *
		  * \note Default value:true */
		bool rangeCheckBetween;
		/** (Default: NULL) If provided, each data range will be tested to be greater-than (rangeMask_min) or less-than (rangeMask_max) each element in these matrices
		  * for each direction (row,col). Values of 0.0f mean no filtering at those directions.
			* If both `rangeMask_min` and `rangeMask_max` are provided, the joint filtering operation is determined by `rangeCheckBetween` */
		const mrpt::math::CMatrix * rangeMask_min, * rangeMask_max;
		TRangeImageFilterParams() : rangeCheckBetween(true), rangeMask_min(NULL), rangeMask_max(NULL)
		{}
	};

	/** Mainly for internal use within CObservation3DRangeScan::project3DPointsFromDepthImageInto() */
	struct OBS_IMPEXP TRangeImageFilter
	{
		TRangeImageFilterParams fp;
		inline bool do_range_filter(size_t r, size_t c, const float D) const; //!< Returns true if the point (r,c) with depth D passes all filters.
		inline TRangeImageFilter(const TRangeImageFilterParams &filter_params) : fp(filter_params) {}
		inline TRangeImageFilter() {}
	};


	// ======== Implementation ========
	bool TRangeImageFilter::do_range_filter(size_t r, size_t c, const float D)  const {
		// Filters:
		if (D<=.0f)
			return false;
		// Greater-than/Less-than filters:
		bool pass_gt=true, pass_lt=true;
		bool has_min_filter = false, has_max_filter = false;
		if (fp.rangeMask_min) {
			const float min_d = fp.rangeMask_min->coeff(r,c);
			if (min_d!=.0f) {
				has_min_filter = true;
				pass_gt = (D>=min_d);
			}
		}
		if (fp.rangeMask_max) {
			const float max_d = fp.rangeMask_max->coeff(r,c);
			if (max_d!=.0f) {
				has_max_filter=true;
				pass_lt = (D<=max_d);
			}
		}
		if (has_min_filter && has_max_filter) {
			return fp.rangeCheckBetween ? (pass_gt && pass_lt) : !(pass_gt && pass_lt);
		}
		else return pass_gt && pass_lt;
	} // do_range_filter()

} } // End of namespaces

#endif

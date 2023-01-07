/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/optional_ref.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/poses_frwds.h>
#include <mrpt/system/COutputLogger.h>

namespace mrpt::slam
{
/** Used as base class for other result structures of each particular algorithm
 * in CMetricMapsAlignmentAlgorithm derived classes.
 *
 * \ingroup mrpt_slam_grp
 */
struct TMetricMapAlignmentResult
{
	TMetricMapAlignmentResult() = default;
	virtual ~TMetricMapAlignmentResult() = default;

	double executionTime = 0;
};

/** A base class for any algorithm able of maps alignment. There are two methods
 *   depending on an PDF or a single 2D Pose value is available as initial guess
 * for the methods.
 *
 * \sa CPointsMap
 * \ingroup mrpt_slam_grp
 */
class CMetricMapsAlignmentAlgorithm : public mrpt::system::COutputLogger
{
   public:
	CMetricMapsAlignmentAlgorithm()
		: mrpt::system::COutputLogger("CMetricMapsAlignmentAlgorithm")
	{
	}
	/** Dtor */
	~CMetricMapsAlignmentAlgorithm() override = default;

	/** The method for aligning a pair of metric maps, for SE(2) relative poses.
	 * The meaning of some parameters and the kind of the maps to be aligned
	 * are implementation dependant, so look at the derived classes for
	 * instructions. The target is to find a PDF for the pose displacement
	 * between maps, **thus the pose of m2 relative to m1**. This pose is
	 * returned as a PDF rather than a single value.
	 *
	 * \param m1			[IN] The first map
	 * \param m2			[IN] The second map. The pose of this map respect to
	 * m1 is to be estimated.
	 * \param grossEst		[IN] An initial gross estimation for the
	 * displacement. If a given algorithm doesn't need it, set to
	 * `CPose2D(0,0,0)` for example.
	 * \param runningTime	[OUT] A pointer to a container for obtaining the
	 * algorithm running time in seconds, or nullptr if you don't need it.
	 * \param info			[OUT] See derived classes for details, or nullptr if
	 * it
	 * isn't needed.
	 *
	 * \return A smart pointer to the output estimated pose PDF.
	 * \sa CICP
	 */
	mrpt::poses::CPosePDF::Ptr Align(
		const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
		const mrpt::poses::CPose2D& grossEst,
		mrpt::optional_ref<TMetricMapAlignmentResult> outInfo = std::nullopt);

	/** \overload of Align() with a PDF for the initial pose.
	 *
	 * \return A smart pointer to the output estimated pose PDF.
	 * \sa CICP
	 */
	virtual mrpt::poses::CPosePDF::Ptr AlignPDF(
		const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
		const mrpt::poses::CPosePDFGaussian& initialEstimationPDF,
		mrpt::optional_ref<TMetricMapAlignmentResult> outInfo =
			std::nullopt) = 0;

	/** \overload of Align() for SE(3) poses.
	 *
	 * \return A smart pointer to the output estimated pose PDF.
	 * \sa CICP
	 */
	mrpt::poses::CPose3DPDF::Ptr Align3D(
		const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
		const mrpt::poses::CPose3D& grossEst,
		mrpt::optional_ref<TMetricMapAlignmentResult> outInfo = std::nullopt);

	/** \overload of Align3D() for initial guess given as a pose PDF.
	 *
	 * \return A smart pointer to the output estimated pose PDF.
	 * \sa CICP
	 */
	virtual mrpt::poses::CPose3DPDF::Ptr Align3DPDF(
		const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
		const mrpt::poses::CPose3DPDFGaussian& initialEstimationPDF,
		mrpt::optional_ref<TMetricMapAlignmentResult> outInfo =
			std::nullopt) = 0;
};

}  // namespace mrpt::slam

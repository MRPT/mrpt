/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/poses/poses_frwds.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose3DPDF.h>

#include <mrpt/system/COutputLogger.h>

namespace mrpt::slam
{
/** A base class for any algorithm able of maps alignment. There are two methods
 *   depending on an PDF or a single 2D Pose value is available as initial guess
 * for the methods.
 *
 * \sa CPointsMap, \ingroup mrpt_slam_grp
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
	/** The method for aligning a pair of metric maps, aligning only 2D +
	 * orientation.
	 *   The meaning of some parameters and the kind of the maps to be aligned
	 * are implementation dependant,
	 *    so look into the derived classes for instructions.
	 *  The target is to find a PDF for the pose displacement between
	 *   maps, <b>thus the pose of m2 relative to m1</b>. This pose
	 *   is returned as a PDF rather than a single value.
	 *
	 * \param m1			[IN] The first map
	 * \param m2			[IN] The second map. The pose of this map respect to
	 * m1
	 * is
	 * to
	 * be estimated.
	 * \param grossEst		[IN] An initial gross estimation for the
	 * displacement.
	 * If a given algorithm doesn't need it, set to <code>CPose2D(0,0,0)</code>
	 * for example.
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
		const mrpt::poses::CPose2D& grossEst, float* runningTime = nullptr,
		void* info = nullptr);

	/** The virtual method for aligning a pair of metric maps, aligning only 2D
	 * + orientation.
	 *   The meaning of some parameters are implementation dependant,
	 *    so look at the derived classes for more details.
	 *  The goal is to find a PDF for the pose displacement between
	 *   maps, that is, <b>the pose of m2 relative to m1</b>. This pose
	 *   is returned as a PDF rather than a single value.
	 *
	 *  \note This method can be configurated with a "options" structure in the
	 * implementation classes.
	 *
	 * \param m1			[IN] The first map (MUST BE A COccupancyGridMap2D
	 * derived
	 * class)
	 * \param m2			[IN] The second map. (MUST BE A CPointsMap derived
	 * class)
	 * The
	 * pose of this map respect to m1 is to be estimated.
	 * \param initialEstimationPDF	[IN] An initial gross estimation for the
	 * displacement.
	 * \param runningTime	[OUT] A pointer to a container for obtaining the
	 * algorithm running time in seconds, or nullptr if you don't need it.
	 * \param info			[OUT] See derived classes for details, or nullptr if
	 * it
	 * isn't needed.
	 *
	 * \return A smart pointer to the output estimated pose PDF.
	 * \sa CICP
	 */
	virtual mrpt::poses::CPosePDF::Ptr AlignPDF(
		const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
		const mrpt::poses::CPosePDFGaussian& initialEstimationPDF,
		float* runningTime = nullptr, void* info = nullptr) = 0;

	/** The method for aligning a pair of metric maps, aligning the full 6D
	 * pose.
	 *   The meaning of some parameters and the kind of the maps to be aligned
	 * are implementation dependant,
	 *    so look into the derived classes for instructions.
	 *  The target is to find a PDF for the pose displacement between
	 *   maps, <b>thus the pose of m2 relative to m1</b>. This pose
	 *   is returned as a PDF rather than a single value.
	 *
	 * \param m1			[IN] The first map
	 * \param m2			[IN] The second map. The pose of this map respect to
	 * m1
	 * is
	 * to
	 * be estimated.
	 * \param grossEst		[IN] An initial gross estimation for the
	 * displacement.
	 * If a given algorithm doesn't need it, set to <code>CPose3D(0,0,0)</code>
	 * for example.
	 * \param runningTime	[OUT] A pointer to a container for obtaining the
	 * algorithm running time in seconds, or nullptr if you don't need it.
	 * \param info			[OUT] See derived classes for details, or nullptr if
	 * it
	 * isn't needed.
	 *
	 * \return A smart pointer to the output estimated pose PDF.
	 * \sa CICP
	 */
	mrpt::poses::CPose3DPDF::Ptr Align3D(
		const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
		const mrpt::poses::CPose3D& grossEst, float* runningTime = nullptr,
		void* info = nullptr);

	/** The virtual method for aligning a pair of metric maps, aligning the full
	 * 6D pose.
	 *   The meaning of some parameters are implementation dependant,
	 *    so look at the derived classes for more details.
	 *  The goal is to find a PDF for the pose displacement between
	 *   maps, that is, <b>the pose of m2 relative to m1</b>. This pose
	 *   is returned as a PDF rather than a single value.
	 *
	 *  \note This method can be configurated with a "options" structure in the
	 * implementation classes.
	 *
	 * \param m1			[IN] The first map (MUST BE A COccupancyGridMap2D
	 * derived
	 * class)
	 * \param m2			[IN] The second map. (MUST BE A CPointsMap derived
	 * class)
	 * The
	 * pose of this map respect to m1 is to be estimated.
	 * \param initialEstimationPDF	[IN] An initial gross estimation for the
	 * displacement.
	 * \param runningTime	[OUT] A pointer to a container for obtaining the
	 * algorithm running time in seconds, or nullptr if you don't need it.
	 * \param info			[OUT] See derived classes for details, or nullptr if
	 * it
	 * isn't needed.
	 *
	 * \return A smart pointer to the output estimated pose PDF.
	 * \sa CICP
	 */
	virtual mrpt::poses::CPose3DPDF::Ptr Align3DPDF(
		const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
		const mrpt::poses::CPose3DPDFGaussian& initialEstimationPDF,
		float* runningTime = nullptr, void* info = nullptr) = 0;
};

}  // namespace mrpt::slam

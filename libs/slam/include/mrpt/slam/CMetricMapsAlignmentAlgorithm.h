/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CMetricMapsAlignmentAlgorithm_H
#define CMetricMapsAlignmentAlgorithm_H

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/poses/poses_frwds.h>

#include <mrpt/utils/COutputLogger.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
namespace slam
{
	/** A base class for any algorithm able of maps alignment. There are two methods
	 *   depending on an PDF or a single 2D Pose value is available as initial guess for the methods.
     *
	 * \sa CPointsMap, \ingroup mrpt_slam_grp 
	 */
	class SLAM_IMPEXP  CMetricMapsAlignmentAlgorithm : public mrpt::utils::COutputLogger
	{
	public:
		CMetricMapsAlignmentAlgorithm() : mrpt::utils::COutputLogger("CMetricMapsAlignmentAlgorithm") {}
		/** Dtor */
		virtual ~CMetricMapsAlignmentAlgorithm() { }

		/** The method for aligning a pair of metric maps, aligning only 2D + orientation.
		 *   The meaning of some parameters and the kind of the maps to be aligned are implementation dependant,
		 *    so look into the derived classes for instructions.
		 *  The target is to find a PDF for the pose displacement between
		 *   maps, <b>thus the pose of m2 relative to m1</b>. This pose
		 *   is returned as a PDF rather than a single value.
		 *
		 * \param m1			[IN] The first map
		 * \param m2			[IN] The second map. The pose of this map respect to m1 is to be estimated.
		 * \param grossEst		[IN] An initial gross estimation for the displacement. If a given algorithm doesn't need it, set to <code>CPose2D(0,0,0)</code> for example.
		 * \param runningTime	[OUT] A pointer to a container for obtaining the algorithm running time in seconds, or NULL if you don't need it.
		 * \param info			[OUT] See derived classes for details, or NULL if it isn't needed.
		 *
		 * \return A smart pointer to the output estimated pose PDF.
		 * \sa CICP
		 */
		mrpt::poses::CPosePDFPtr Align(
				const mrpt::maps::CMetricMap		*m1,
				const mrpt::maps::CMetricMap		*m2,
				const mrpt::poses::CPose2D			&grossEst,
				float					*runningTime = NULL,
				void					*info = NULL );

        /** The virtual method for aligning a pair of metric maps, aligning only 2D + orientation.
		 *   The meaning of some parameters are implementation dependant,
		 *    so look at the derived classes for more details.
		 *  The goal is to find a PDF for the pose displacement between
		 *   maps, that is, <b>the pose of m2 relative to m1</b>. This pose
		 *   is returned as a PDF rather than a single value.
		 *
		 *  \note This method can be configurated with a "options" structure in the implementation classes.
		 *
		 * \param m1			[IN] The first map (MUST BE A COccupancyGridMap2D  derived class)
		 * \param m2			[IN] The second map. (MUST BE A CPointsMap derived class) The pose of this map respect to m1 is to be estimated.
		 * \param initialEstimationPDF	[IN] An initial gross estimation for the displacement.
		 * \param runningTime	[OUT] A pointer to a container for obtaining the algorithm running time in seconds, or NULL if you don't need it.
		 * \param info			[OUT] See derived classes for details, or NULL if it isn't needed.
		 *
		 * \return A smart pointer to the output estimated pose PDF.
		 * \sa CICP
		 */
		virtual mrpt::poses::CPosePDFPtr AlignPDF(
				const mrpt::maps::CMetricMap		*m1,
				const mrpt::maps::CMetricMap		*m2,
				const mrpt::poses::CPosePDFGaussian	&initialEstimationPDF,
				float					*runningTime = NULL,
				void					*info = NULL ) = 0;


		/** The method for aligning a pair of metric maps, aligning the full 6D pose.
		 *   The meaning of some parameters and the kind of the maps to be aligned are implementation dependant,
		 *    so look into the derived classes for instructions.
		 *  The target is to find a PDF for the pose displacement between
		 *   maps, <b>thus the pose of m2 relative to m1</b>. This pose
		 *   is returned as a PDF rather than a single value.
		 *
		 * \param m1			[IN] The first map
		 * \param m2			[IN] The second map. The pose of this map respect to m1 is to be estimated.
		 * \param grossEst		[IN] An initial gross estimation for the displacement. If a given algorithm doesn't need it, set to <code>CPose3D(0,0,0)</code> for example.
		 * \param runningTime	[OUT] A pointer to a container for obtaining the algorithm running time in seconds, or NULL if you don't need it.
		 * \param info			[OUT] See derived classes for details, or NULL if it isn't needed.
		 *
		 * \return A smart pointer to the output estimated pose PDF.
		 * \sa CICP
		 */
		mrpt::poses::CPose3DPDFPtr Align3D(
				const mrpt::maps::CMetricMap		*m1,
				const mrpt::maps::CMetricMap		*m2,
				const mrpt::poses::CPose3D			&grossEst,
				float					*runningTime = NULL,
				void					*info = NULL );

        /** The virtual method for aligning a pair of metric maps, aligning the full 6D pose.
		 *   The meaning of some parameters are implementation dependant,
		 *    so look at the derived classes for more details.
		 *  The goal is to find a PDF for the pose displacement between
		 *   maps, that is, <b>the pose of m2 relative to m1</b>. This pose
		 *   is returned as a PDF rather than a single value.
		 *
		 *  \note This method can be configurated with a "options" structure in the implementation classes.
		 *
		 * \param m1			[IN] The first map (MUST BE A COccupancyGridMap2D  derived class)
		 * \param m2			[IN] The second map. (MUST BE A CPointsMap derived class) The pose of this map respect to m1 is to be estimated.
		 * \param initialEstimationPDF	[IN] An initial gross estimation for the displacement.
		 * \param runningTime	[OUT] A pointer to a container for obtaining the algorithm running time in seconds, or NULL if you don't need it.
		 * \param info			[OUT] See derived classes for details, or NULL if it isn't needed.
		 *
		 * \return A smart pointer to the output estimated pose PDF.
		 * \sa CICP
		 */
		virtual mrpt::poses::CPose3DPDFPtr Align3DPDF(
				const mrpt::maps::CMetricMap		*m1,
				const mrpt::maps::CMetricMap		*m2,
				const mrpt::poses::CPose3DPDFGaussian	&initialEstimationPDF,
				float					*runningTime = NULL,
				void					*info = NULL ) = 0;


	};

	} // End of namespace
} // End of namespace

#endif

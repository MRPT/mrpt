/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
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
#ifndef CMetricMapsAlignmentAlgorithm_H
#define CMetricMapsAlignmentAlgorithm_H

#include <mrpt/slam/CPointsMap.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <mrpt/utils/CDebugOutputCapable.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
namespace slam
{
	/** A base class for any algorithm able of maps alignment. There are two methods
	 *   depending on an PDF or a single 2D Pose value is available as initial guess for the methods.
     *
	 * \sa CPointsMap, utils::CDebugOutputCapable  \ingroup mrpt_slam_grp 
	 */
	class SLAM_IMPEXP  CMetricMapsAlignmentAlgorithm : public mrpt::utils::CDebugOutputCapable
	{
	public:
        /** Destructor
          */
        virtual ~CMetricMapsAlignmentAlgorithm()
        {
        }

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
		CPosePDFPtr Align(
				const CMetricMap		*m1,
				const CMetricMap		*m2,
				const CPose2D			&grossEst,
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
		virtual CPosePDFPtr AlignPDF(
				const CMetricMap		*m1,
				const CMetricMap		*m2,
				const CPosePDFGaussian	&initialEstimationPDF,
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
		CPose3DPDFPtr Align3D(
				const CMetricMap		*m1,
				const CMetricMap		*m2,
				const CPose3D			&grossEst,
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
		virtual CPose3DPDFPtr Align3DPDF(
				const CMetricMap		*m1,
				const CMetricMap		*m2,
				const CPose3DPDFGaussian	&initialEstimationPDF,
				float					*runningTime = NULL,
				void					*info = NULL ) = 0;


	};

	} // End of namespace
} // End of namespace

#endif

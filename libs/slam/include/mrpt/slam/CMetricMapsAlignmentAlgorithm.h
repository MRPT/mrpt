/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

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
#ifndef  path_from_rtk_gps_H
#define  path_from_rtk_gps_H

#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/slam/CRawlog.h>

#include <mrpt/topography/link_pragmas.h>


namespace mrpt
{
	namespace topography
	{
		/** \addtogroup mrpt_topography_grp
		  *  @{ */

		/** Used to return optional information from mrpt::topography::path_from_rtk_gps */
		struct TOPO_IMPEXP TPathFromRTKInfo
		{
			std::map<mrpt::system::TTimeStamp,mrpt::math::TPoint3D> best_gps_path; //!< the path of the "best" GPS.
			std::map<mrpt::system::TTimeStamp, double> mahalabis_quality_measure; //!< A measure of the quality at each point (may be empty if not there is no enough information).
			mrpt::aligned_containers<mrpt::system::TTimeStamp, mrpt::math::CMatrixDouble66 >::map_t vehicle_uncertainty; //!< The 6x6 covariance matrix for the uncertainty of each vehicle pose (may be empty if there is no W_star info).
			mrpt::math::CMatrixDouble			W_star; //!< The reference covariance matrix used to compute vehicle_uncertainty.
		};

		/** Reconstruct the path of a vehicle equipped with 3 RTK GPSs.
		  *  \param robot_path [OUT] The reconstructed vehicle path
		  *  \param rawlog [IN] The dataset
		  *  \param rawlog_first [IN] The index of the first entry to process (first=0)
		  *  \param rawlog_last [IN] The index of the last entry to process
		  *  \param isGUI [IN] If set to true, some progress dialogs will be shown during the computation (requires MRPT built with support for wxWidgets).
		  *  \param disableGPSInterp [IN] Whether to interpolate missing GPS readings between very close datums.
		  *  \param path_smooth_filter_size [IN] Size of the window in the pitch & roll noise filtering.
		  *  \param outInfo [OUT] Optional output: additional information from the optimization
		  *
		  *  For more details on the method, refer to the paper: (...)
		  * \sa mrpt::topography
		  */
		void  TOPO_IMPEXP path_from_rtk_gps(
			mrpt::poses::CPose3DInterpolator	&robot_path,
			const mrpt::slam::CRawlog			&rawlog,
			size_t 								rawlog_first,
			size_t 								rawlog_last,
			bool								isGUI=false,
			bool								disableGPSInterp=false,
			int									path_smooth_filter_size=2,
			TPathFromRTKInfo					*outInfo = NULL
			);
			
			
		/** @} */ // end of grouping

	} // End of namespace

} // End of namespace

#endif

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

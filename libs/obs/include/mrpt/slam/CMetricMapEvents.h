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
#ifndef CMetricMapEvents_H
#define CMetricMapEvents_H

#include <mrpt/utils/mrptEvent.h>

namespace mrpt
{
	namespace poses { class CPose3D; }
	namespace slam
	{
		class CObservation;
		class CMetricMap;

		/** Event emitted by a metric up upon call of clear()
		  * \sa CMetricMap
	 	  * \ingroup mrpt_obs_grp
		  */
		class mrptEventMetricMapClear : public mrpt::utils::mrptEvent
		{
		protected:
			virtual void do_nothing() { } //!< Just to allow this class to be polymorphic
		public:
			inline mrptEventMetricMapClear(const CMetricMap   *smap) : source_map(smap) {}

			const CMetricMap  *source_map;
		};

		/** Event emitted by a metric up upon a succesful call to insertObservation()
		  * \sa CMetricMap
	 	  * \ingroup mrpt_obs_grp
		  */
		class mrptEventMetricMapInsert : public mrpt::utils::mrptEvent
		{
		protected:
			virtual void do_nothing() { } //!< Just to allow this class to be polymorphic
		public:
			inline mrptEventMetricMapInsert(const CMetricMap   *smap, const CObservation *obs,const CPose3D *robotPose ) : source_map(smap), inserted_obs(obs), inserted_robotPose(robotPose) { } 

			const CMetricMap   *source_map;
			const CObservation *inserted_obs;
			const CPose3D      *inserted_robotPose;

		};

	} // End of namespace
} // End of namespace

#endif

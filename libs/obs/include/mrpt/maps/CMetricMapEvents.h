/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CMetricMapEvents_H
#define CMetricMapEvents_H

#include <mrpt/utils/mrptEvent.h>
#include <mrpt/poses/poses_frwds.h>

namespace mrpt
{
namespace obs { class CObservation; }
	namespace maps
	{
		class CMetricMap;

		/** Event emitted by a metric up upon call of clear()
		  * \sa CMetricMap
	 	  * \ingroup mrpt_obs_grp
		  */
		class mrptEventMetricMapClear : public mrpt::utils::mrptEvent
		{
		protected:
			virtual void do_nothing() MRPT_OVERRIDE { } //!< Just to allow this class to be polymorphic
		public:
			inline mrptEventMetricMapClear(const mrpt::maps::CMetricMap   *smap) : source_map(smap) {}

			const mrpt::maps::CMetricMap  *source_map;
		};

		/** Event emitted by a metric up upon a succesful call to insertObservation()
		  * \sa CMetricMap
	 	  * \ingroup mrpt_obs_grp
		  */
		class mrptEventMetricMapInsert : public mrpt::utils::mrptEvent
		{
		protected:
			virtual void do_nothing() MRPT_OVERRIDE { } //!< Just to allow this class to be polymorphic
		public:
			inline mrptEventMetricMapInsert(const mrpt::maps::CMetricMap   *smap, const mrpt::obs::CObservation *obs,const mrpt::poses::CPose3D *robotPose ) : source_map(smap), inserted_obs(obs), inserted_robotPose(robotPose) { }

			const mrpt::maps::CMetricMap   *source_map;
			const mrpt::obs::CObservation *inserted_obs;
			const mrpt::poses::CPose3D *inserted_robotPose;
		};

	} // End of namespace
} // End of namespace

#endif

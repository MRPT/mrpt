/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#ifndef CObjectDetection_H
#define CObjectDetection_H

#include <mrpt/detectors/CDetectableObject.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CImage.h>

namespace mrpt
{
	namespace detectors
	{
		typedef std::vector<CDetectableObjectPtr> vector_detectable_object;

		/** \ingroup mrpt_detectors_grp */
		class DETECTORS_IMPEXP CObjectDetection	
		{
		public:
			/** Initialize the object with parameters loaded from the given config file. */
			inline void init(const std::string &configFile)
			{
				mrpt::utils::CConfigFile  cfg(configFile);
				init(cfg);
			}

			/** Initialize the object with parameters loaded from the given config source. */
			virtual void init(const mrpt::utils::CConfigFileBase &cfg )=0;

			inline void detectObjects(const mrpt::obs::CObservationPtr obs, vector_detectable_object &detected) 
			{ 
				detectObjects_Impl(obs.pointer(), detected); 
			};

			inline void detectObjects( const mrpt::obs::CObservation *obs, vector_detectable_object &detected)
			{
				detectObjects_Impl( obs, detected );
			};			

			void detectObjects(const mrpt::utils::CImage *img, vector_detectable_object &detected);

		protected:

			virtual void detectObjects_Impl( const mrpt::obs::CObservation *obs, vector_detectable_object &detected) = 0;			

		}; // End of class
	}

}

#endif



			

/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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


#ifndef CObjectDetection_H
#define CObjectDetection_H

#include <mrpt/vision/CDetectableObject.h>

namespace mrpt
{
	namespace vision
	{
		using namespace std;
		using namespace mrpt::slam;

		typedef vector<CDetectableObjectPtr> vector_detectable_object;

		class VISION_IMPEXP CObjectDetection	
		{
			//virtual ~CObjectDetection();

			/** Initialize the object with parameters loaded from the given config file. */
			inline void init(const std::string &configFile)
			{
				mrpt::utils::CConfigFile  cfg(configFile);
				init(cfg);
			}

			/** Initialize the object with parameters loaded from the given config source. */
			virtual void init(const mrpt::utils::CConfigFileBase &cfg )=0;

			inline void detectObjects(CObservationPtr obs, vector_detectable_object &detected) 
			{ 
				detectObjects(obs.pointer(), detected); 
			};

			virtual void detectObjects(CObservation *obs, vector_detectable_object &detected)=0;

		}; // End of class
	}

}

#endif



			
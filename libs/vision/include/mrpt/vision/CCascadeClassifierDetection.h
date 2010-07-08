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

#ifndef CCascadeClassifierDetection_H
#define CCascadeClassifierDetection_H

#include <mrpt/vision/CObjectDetection.h>

namespace mrpt
{
	namespace vision
	{
		class VISION_IMPEXP CCascadeClassifierDetection: virtual public CObjectDetection
		{
		public:

			CCascadeClassifierDetection( ) {};
			CCascadeClassifierDetection( std::string configFile );
			virtual ~CCascadeClassifierDetection(); 

			void init(std::string configFile);

			void detectObjects(CObservation *obs, vector_detectable_object &detected);

		protected:

			void * m_cascade;

			struct TOptions
			{
				std::string	cascadeFileName;
				double scaleFactor;
				int minNeighbors;
				int flags;
				int minSize;
			}m_options;

		}; // End of class
	}

}

#endif
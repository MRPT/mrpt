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

#ifndef CCascadeClassifierDetection_H
#define CCascadeClassifierDetection_H

#include <mrpt/detectors/CObjectDetection.h>

namespace mrpt
{
	namespace detectors
	{
		/** 
		  * \ingroup mrpt_detectors_grp
		  */
		class DETECTORS_IMPEXP  CCascadeClassifierDetection: virtual public CObjectDetection
		{
		public:
			
			CCascadeClassifierDetection( );
			
			virtual ~CCascadeClassifierDetection(); 

			/** Initialize cascade classifier detection */
			virtual void init(const mrpt::utils::CConfigFileBase &cfg );

		protected:

			/** Detect objects in a *CObservation
			 * \return A vector with detected objects 
			 */

			virtual void detectObjects_Impl(const CObservation *obs, vector_detectable_object &detected);

			void * m_cascade; //!< Cascade classifier object

			struct TOptions
			{
				std::string	cascadeFileName; 
				double scaleFactor;	
				int minNeighbors;
				int flags;
				int minSize;
			}m_options; //!< Cascade classifier options

		}; // End of class
	}
}

#endif

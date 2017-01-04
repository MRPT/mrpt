/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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

			virtual void detectObjects_Impl(const mrpt::obs::CObservation *obs, vector_detectable_object &detected);

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

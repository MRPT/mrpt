/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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


#ifndef CObjectDetection_H
#define CObjectDetection_H

#include <mrpt/detectors/CDetectableObject.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CImage.h>

namespace mrpt
{
	namespace detectors
	{
		using namespace std;
		using namespace mrpt::slam;
		using namespace mrpt::utils;

		typedef std::vector<CDetectableObjectPtr> vector_detectable_object;

		/** \ingroup mrpt_detectors_grp */
		class DETECTORS_IMPEXP CObjectDetection	
		{
		public:
			//virtual ~CObjectDetection();

			/** Initialize the object with parameters loaded from the given config file. */
			inline void init(const std::string &configFile)
			{
				mrpt::utils::CConfigFile  cfg(configFile);
				init(cfg);
			}

			/** Initialize the object with parameters loaded from the given config source. */
			virtual void init(const mrpt::utils::CConfigFileBase &cfg )=0;

			inline void detectObjects(const CObservationPtr obs, vector_detectable_object &detected) 
			{ 
				detectObjects_Impl(obs.pointer(), detected); 
			};

			inline void detectObjects( const CObservation *obs, vector_detectable_object &detected)
			{
				detectObjects_Impl( obs, detected );
			};			

			void detectObjects(const CImage *img, vector_detectable_object &detected);

		protected:

			virtual void detectObjects_Impl( const CObservation *obs, vector_detectable_object &detected) = 0;			

		}; // End of class
	}

}

#endif



			

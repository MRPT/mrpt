
#ifndef CObjectDetection_H
#define CObjectDetection_H

#include <mrpt\vision\CDetectableObject.h>

namespace mrpt
{
	namespace vision
	{
		typedef vector<CDetectableObject> vector_detectable_object;

		class VISION_IMPEXP CObjectDetection	
		{
			//virtual ~CObjectDetection();

			inline void detectObjects(CObservationPtr obs, vector_detectable_object &detected) 
			{ 
				detectObjects(obs.pointer(), detected); 
			};

			virtual void detectObjects(CObservation *obs, vector_detectable_object &detected)=0;

		}; // End of class
	}

}

#endif



			
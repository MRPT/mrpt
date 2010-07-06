#ifndef CFaceDetection_H
#define CFaceDetection_H

#include <mrpt/vision/CObjectDetection.h>

namespace mrpt
{
	namespace vision
	{
		class VISION_IMPEXP CFaceDetection: virtual public CObjectDetection
		{
			int hola;

			CFaceDetection(){};

			virtual void detectObjects(CObservation *obs);

		}; // End of class
	}

}

#endif
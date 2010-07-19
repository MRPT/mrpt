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

#ifndef CFaceDetection_H
#define CFaceDetection_H

#include <mrpt/detectors/CObjectDetection.h>
#include <mrpt/detectors/CCascadeClassifierDetection.h>

namespace mrpt
{
	namespace slam { class CObservation3DRangeScan; }

	namespace detectors
	{
		using namespace mrpt::slam;

		/** Specific class for face detection.
		  * Methods and variables labeled as experimentals are temporals (for debug or testing
		  * purposes) and may disappear in future versions.
		  */
		class DETECTORS_IMPEXP CFaceDetection: virtual public CObjectDetection
		{
		public:
		
			CCascadeClassifierDetection cascadeClassifier;

			CFaceDetection();

			virtual void init(const mrpt::utils::CConfigFileBase &cfg );

			virtual void detectObjects(CObservation *obs, vector_detectable_object &detected);
			
			inline void detectObjects(CObservationPtr obs, vector_detectable_object &detected)
			{
				detectObjects(obs.pointer(), detected); 
			}

			virtual void detectObjects(CImage *img, vector_detectable_object &detected);

			struct TOptions
			{
				int		confidenceThreshold;
				double	planeThreshold;
				double	regionsThreshold;
			}m_options;

			struct TExperimental
			{
				vector_double lessEigenVals;
			}m_exp;

			// Experimental methods
			void experimental_showMeasurements();

		private:

			bool checkIfFacePlane( const vector<TPoint3D> &points );

			bool checkIfFacePlaneCov( const vector<TPoint3D> &points );

			bool checkIfFaceRegions( CObservation3DRangeScan* face, const unsigned int &faceWidth, const unsigned int &faceHeight );

			bool checkRegionsConstrains( const double values[3][3] );

			// Experimental methods
			void experimental_viewFacePointsScanned( const vector_float &xs, const vector_float &ys, const vector_float &zs );

			void experimental_viewFacePointsScanned( const CObservation3DRangeScan &face );
			
			void experimental_viewFacePointsScanned( const vector<TPoint3D> &points );

			

		}; // End of class
	}

}

#endif
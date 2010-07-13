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


#include <mrpt/vision.h>  // Precompiled headers

#include <mrpt/vision/CFaceDetection.h>

using namespace mrpt::vision;

CFaceDetection::CFaceDetection()
{

}

void CFaceDetection::detectObjects(mrpt::slam::CObservation *obs, vector_detectable_object &detected)
{
	// Detect possible faces
	vector_detectable_object localDetected;
	cascadeClassifier.detectObjects( obs, localDetected );

	if ( (IS_CLASS(obs, CObservation3DRangeScan )) && ( localDetected.size() > 0 ) )
	{
		CObservation3DRangeScan* o = static_cast<CObservation3DRangeScan*>( obs );
	
		// Detected objects to delete if they aren't a face
		vector<size_t> deleteDetected;

		for ( unsigned int i = 0; i < localDetected.size(); i++ )
		{
			CDetectable2DPtr rec	= CDetectable2DPtr(localDetected[i]);
			bool confidence			= o->hasConfidenceImage;

			// First check if we can adjust a plane to detected region as face, if yes it isn't a face!

			CImage			conf;
			CMatrixFloat	conf2,conf3;
		
			if ( confidence )
			{
				conf = o->confidenceImage;
				conf.getAsMatrix(conf2);
				conf2.extractSubmatrix( rec->m_y, rec->m_y + rec->m_height, rec->m_x, rec->m_x + rec->m_width, conf3 );
			}

			vector<TPoint3D> points;

			// Submatrix size
			size_t imgWidth = conf2.getColCount();
			size_t imgHeight = conf2.getRowCount();

			for ( unsigned int j = 0; j < conf3.getRowCount(); j++ )
			{
				for ( unsigned int k = 0; k < conf3.getColCount(); k++ )
				{
					if ( ( confidence ) && ( conf3.get_unsafe( j, k ) > m_options.confidenceThreshold )) // TODO: Check if the point is valid
					{	
						int position = imgHeight*j + rec->m_x + k;
						points.push_back( TPoint3D(o->points3D_x[position],o->points3D_y[position],o->points3D_z[position]) );
					}
					else if ( !confidence )
					{
						int position = imgHeight*j + rec->m_x + k;
						points.push_back( TPoint3D(o->points3D_x[position],o->points3D_y[position],o->points3D_z[position]) );
					}
				}
			}
				
			TPlane plane;
			double estimation = getRegressionPlane(points,plane);	

			// TODO: Chose a estimation threshold and delete no-faces of detected vector!!
			if ( estimation > 0.9 )
				deleteDetected.push_back( i );
			
		}

		// Delete non faces
		for ( unsigned int i = deleteDetected.size(); i > 0; i-- )
			localDetected.erase( localDetected.begin() + deleteDetected[i-1] );

		// Convert 2d detected objects to 3d
		for ( unsigned int i = 0; i < localDetected.size(); i++ )
		{
			CDetectable3DPtr object3d = CDetectable3DPtr( new CDetectable3D((CDetectable2DPtr)localDetected[i]) );
			detected.push_back( object3d );
		}

	}
	else
	{
		detected = localDetected;
	}

	
}

void CFaceDetection::init(const mrpt::utils::CConfigFileBase &cfg )
{
	m_options.confidenceThreshold = cfg.read_double("FaceDetection","confidenceThreshold",0.9);

	cascadeClassifier.init( cfg );
}

void CFaceDetection::detectObjects(CImage *img, vector_detectable_object &detected)
{
	cascadeClassifier.detectObjects( img, detected );
}
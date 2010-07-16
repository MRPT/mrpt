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

//------------------------------------------------------------------------
//							CFaceDetection
//------------------------------------------------------------------------
CFaceDetection::CFaceDetection()
{

}


//------------------------------------------------------------------------
//								init
//------------------------------------------------------------------------
void CFaceDetection::init(const mrpt::utils::CConfigFileBase &cfg )
{
	m_options.confidenceThreshold	= cfg.read_int( "FaceDetection", "confidenceThreshold", 125 );
	m_options.planeThreshold		= cfg.read_double( "FaceDetection", "planeThreshold", 0.5 );
	m_options.regionsThreshold		= cfg.read_double( "FaceDetection", "regionsThreshold", 0.5 );

	cascadeClassifier.init( cfg );
}


//------------------------------------------------------------------------
//							detectObjects
//------------------------------------------------------------------------
void CFaceDetection::detectObjects(mrpt::slam::CObservation *obs, vector_detectable_object &detected)
{
	// Detect possible faces
	vector_detectable_object localDetected;
	cascadeClassifier.detectObjects( obs, localDetected );

	if ( (IS_CLASS(obs, CObservation3DRangeScan )) && ( localDetected.size() > 0 ) )
	{
		CObservation3DRangeScan* o = static_cast<CObservation3DRangeScan*>( obs );

		if ( o->hasPoints3D )
		{
			// Detected objects to delete if they aren't a face
			vector<size_t> deleteDetected;

			for ( unsigned int i = 0; i < localDetected.size(); i++ )
			{
				CDetectable2DPtr rec	= CDetectable2DPtr(localDetected[i]);
				bool confidence			= o->hasConfidenceImage;

				// First check if we can adjust a plane to detected region as face, if yes it isn't a face!

				// Calculate initial and final rows and columns
				unsigned int r1 = rec->m_y;
				unsigned int r2 = rec->m_y + rec->m_height;
				unsigned int c1 = rec->m_x;
				unsigned int c2 = rec->m_x + rec->m_width;

				// Image size
				size_t imgWidth		= o->cameraParams.ncols;
				//size_t imgHeight	= o->cameraParams.nrows;

				// Create a vector with points coordinates
				vector<TPoint3D> points;

				for ( unsigned int j = 0; j < r2-r1; j++ )
				{
					for ( unsigned int k = 0; k < c2-c1; k++ )
					{
						if ( ( confidence ) && ( *(o->confidenceImage.get_unsafe( j, k, 0 )) > m_options.confidenceThreshold )) // TODO: Check if the point is valid
						{
							int position = imgWidth*j + c1 + k;
							points.push_back( TPoint3D(o->points3D_x[position],o->points3D_y[position],o->points3D_z[position]) );
						}
						else if ( !confidence )
						{
							int position = imgWidth*j + c1 + k;
							points.push_back( TPoint3D(o->points3D_x[position],o->points3D_y[position],o->points3D_z[position]) );
						}
					}
				}

				// Check if it's really a face!
				if ( !checkIfFacePlane( points ) )
					deleteDetected.push_back( i );
				else
				{
					CObservation3DRangeScan face;
					o->getZoneAsObs( face, r1, r2, c1, c2 );
					if ( !checkIfFaceRegions( &face, c2-c1, r2-r1 ) )
						deleteDetected.push_back( i );
				}

			}

			// Delete non faces
			for ( unsigned int i = deleteDetected.size(); i > 0; i-- )
				localDetected.erase( localDetected.begin() + deleteDetected[i-1] );
		}

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


//------------------------------------------------------------------------
//							detectObjects
//------------------------------------------------------------------------
void CFaceDetection::detectObjects(CImage *img, vector_detectable_object &detected)
{
	cascadeClassifier.detectObjects( img, detected );
}


//------------------------------------------------------------------------
//  						checkIfFacePlane
//------------------------------------------------------------------------
bool CFaceDetection::checkIfFacePlane( const vector<TPoint3D> &points )
{
	// Try to ajust a plane
	TPlane plane;

	if ( getRegressionPlane(points,plane) < m_options.planeThreshold )
		return true;

	return false;
}

//------------------------------------------------------------------------
//							checkIfFaceSections
//------------------------------------------------------------------------
bool CFaceDetection::checkIfFaceRegions( CObservation3DRangeScan* face,
										 const unsigned int &faceWidth,
										 const unsigned int &faceHeight )
{
	unsigned int x1 = ceil(faceWidth*0.1);
	unsigned int x2 = floor(faceWidth*0.9);
	unsigned int y1 = ceil(faceHeight*0.1);
	unsigned int y2 = floor(faceHeight*0.9);

	unsigned int sectionVSize = floor((y2-y1)/3.0);
	unsigned int sectionHSize = floor((x2-x1)/3.0);

	vector<TPoint3D> points;
	unsigned int cont = 0;

	double meanDepth[3][3] = { {0,0,0}, {0,0,0}, {0,0,0} };
	double numPoints[3][3] = { {0,0,0}, {0,0,0}, {0,0,0} };

	for ( unsigned int i = y1; i <= y2; i++ )
		for ( unsigned int j = x1; j <= x2; j++, cont++ )
			if (*(face->confidenceImage.get_unsafe( j, i, 0 )) > m_options.confidenceThreshold )
			{
				unsigned int row, col;
				if ( i-y1 < sectionVSize - floor(sectionVSize*0.2) )
					row = 0;
				else if ( i-y1 < sectionVSize*2 + floor(sectionVSize*0.2) )
					row = 1;
				else
					row = 2;

				if ( j-x1 < sectionHSize - floor(sectionHSize*0.2) )
					col = 0;
				else if ( j-x1 < sectionHSize*2 + floor(sectionHSize*0.2) )
					col = 1;
				else
					col = 2;

				meanDepth[row][col]+=face->points3D_x[cont];
				++numPoints[row][col];
			}

	// Create 9 regions and calculate
	vector<vector<TPoint3D> > regions;

	for ( size_t i = 0; i < 3; i++ )
		for ( size_t j = 0; j < 3; j++ )
			if ( !numPoints[i][j] )
				meanDepth[i][j] = 0;
			else
				meanDepth[i][j] /= numPoints[i][j];

	/*ofstream f2;
	f2.open("fichero2.txt", ofstream::app);

	f2 << meanDepth[0][0] << "." << meanDepth[0][1] << "." << meanDepth[0][2] << endl;
	f2 << meanDepth[1][0] << "." << meanDepth[1][1] << "." << meanDepth[1][2] << endl;
	f2 << meanDepth[2][0] << "." << meanDepth[2][1] << "." << meanDepth[2][2] << endl;

	f2.close();*/

	return checkRegionsConstrains( meanDepth );
}

bool CFaceDetection::checkRegionsConstrains( const double values[3][3] )
{
	// This matrix put, for instance, in 0,0 an 1 if this region is farther that 0,1 region
	double satisfy[3][3] = { {0,0,0}, {0,0,0}, {0,0,0} };

	if ( values[0][0] < values[0][1] )
		satisfy[0][0] = 1;
	if ( values[0][2] < values[0][1] )
		satisfy[0][2] = 1;
	if ( values[1][0] < values[1][1] )
		satisfy[1][0] = 1;
	if ( values[1][2] < values[1][1] )
		satisfy[1][2] = 1;
	if ( values[2][0] < values[2][1] )
		satisfy[2][0] = 1;
	if ( values[2][2] < values[2][1] )
		satisfy[2][2] = 1;

	size_t sumCol0	= satisfy[0][0] + satisfy[1][0] + satisfy[2][0];
	size_t sumCol2	= satisfy[0][2] + satisfy[1][2] + satisfy[2][2];
	//size_t sumFil0 = satisfy[0][0] + satisfy[0][1] + satisfy[0][2];
	//size_t sumFil2 = satisfy[2][0] + satisfy[2][1] + satisfy[2][2];

	// We can choose any constrain to require to candidate face scan
	if ( ( sumCol0 >= 2 ) && ( sumCol2 >= 2 ) ) // Frontal faces
		return true;
	if ( ( sumCol0 == 3 ) || ( sumCol2 == 3 ) ) // Profile faces
		return true;

	return false;
}

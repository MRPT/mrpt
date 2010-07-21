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

#include <mrpt/obs.h>   // Precompiled headers

#include <mrpt/slam/CObservationDisparityImages.h>
//#include <mrpt/slam/CLandmarksMap.h>

using namespace mrpt::slam; 
using namespace mrpt::utils; 
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationDisparityImages, CObservation,mrpt::slam)

        /*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
        CObservationDisparityImages::CObservationDisparityImages( void *iplImageLeft,void *iplImageDisparity ) :
	//m_auxMap(),
	cameraPose(),
	leftCamera(),
	imageLeft( iplImageLeft ),
        imageDisparity( iplImageDisparity )

{
}

/*---------------------------------------------------------------
					Default Constructor
 ---------------------------------------------------------------*/
CObservationDisparityImages::CObservationDisparityImages( ) :
        //	m_auxMap(),
	cameraPose(),
	leftCamera(),
	imageLeft( NULL ),
        imageDisparity( NULL )

{
}

/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CObservationDisparityImages::~CObservationDisparityImages(  )
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationDisparityImages::writeToStream(CStream &out, int *version) const
{
    if (version)
        *version =0 ;
    else
    {
        // The data
        out << cameraPose << leftCamera << imageLeft << imageDisparity;
        out << timestamp;
        out << sensorLabel;
    }
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationDisparityImages::readFromStream(CStream &in, int version)
{
    switch(version)
    {
    case 0:

        {



            in >> cameraPose >> leftCamera;

            in >> imageLeft >> imageDisparity;														// For all the versions

            in >> timestamp;

            in >> sensorLabel; 							// For version 1 to 5

        } break;
    default:
        MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
            };
}


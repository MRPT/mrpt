/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
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

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/utils/TStereoCamera.h>
#include <mrpt/utils/CConfigFileMemory.h>

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( TStereoCamera, CSerializable, mrpt::utils )

TStereoCamera::TStereoCamera()
{
}


/**  Save as a config block:
  */
void TStereoCamera::saveToConfigFile(const std::string &section,  mrpt::utils::CConfigFileBase &cfg ) const
{
	// [<SECTION>_LEFT]
	//   ...
	// [<SECTION>_RIGHT]
	//   ...
	// [<SECTION>_LEFT2RIGHT_POSE]
	//  pose_quaternion = [x y z qr qx qy qz]

	leftCamera.saveToConfigFile(section+string("_LEFT"), cfg);
	rightCamera.saveToConfigFile(section+string("_RIGHT"), cfg);

	const CPose3DQuat q = CPose3DQuat(rightCameraPose); // Don't remove this line, it's a safe against future possible bugs if rightCameraPose changes!
	cfg.write(section+string("_LEFT2RIGHT_POSE"), "pose_quaternion",q.asString() );
}

/**  Load all the params from a config source, in the format described in saveToConfigFile()
  */
void TStereoCamera::loadFromConfigFile(const std::string &section,  const mrpt::utils::CConfigFileBase &cfg )
{
	// [<SECTION>_LEFT]
	//   ...
	// [<SECTION>_RIGHT]
	//   ...
	// [<SECTION>_LEFT2RIGHT_POSE]
	//  pose_quaternion = [x y z qr qx qy qz]

	leftCamera.loadFromConfigFile(section+string("_LEFT"), cfg);
	rightCamera.loadFromConfigFile(section+string("_RIGHT"), cfg);
	rightCameraPose.fromString( cfg.read_string(section+string("_LEFT2RIGHT_POSE"), "pose_quaternion","" ) );
}

// WriteToStream
void TStereoCamera::writeToStream( CStream &out, int *version ) const
{
	if( version )
		*version = 1;
	else
	{
	    out << leftCamera
            << rightCamera
            << rightCameraPose;
	} // end else
}

// ReadFromStream
void TStereoCamera::readFromStream( CStream &in, int version )
{
	switch( version )
	{
	case 0:
	case 1:
    {
		if (version==0)
		{
			uint8_t _model;
			in  >> _model;  // unused now
		}

        in  >> leftCamera
            >> rightCamera
            >> rightCameraPose;
    } break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION( version )
	}
}

std::string TStereoCamera::dumpAsText() const
{
	mrpt::utils::CConfigFileMemory cfg;
	saveToConfigFile("",cfg);
	return cfg.getContent();
}


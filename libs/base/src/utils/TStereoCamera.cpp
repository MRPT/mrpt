/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/TStereoCamera.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::utils;
using namespace mrpt::poses;
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


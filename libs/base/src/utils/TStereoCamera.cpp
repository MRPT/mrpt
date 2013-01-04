/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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


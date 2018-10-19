/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "img-precomp.h"  // Precompiled headers

#include <mrpt/img/TStereoCamera.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::img;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(TStereoCamera, CSerializable, mrpt::img)

/**  Save as a config block:
 */
void TStereoCamera::saveToConfigFile(
	const std::string& section, mrpt::config::CConfigFileBase& cfg) const
{
	// [<SECTION>_LEFT]
	//   ...
	// [<SECTION>_RIGHT]
	//   ...
	// [<SECTION>_LEFT2RIGHT_POSE]
	//  pose_quaternion = [x y z qr qx qy qz]

	leftCamera.saveToConfigFile(section + string("_LEFT"), cfg);
	rightCamera.saveToConfigFile(section + string("_RIGHT"), cfg);

	cfg.write(
		section + string("_LEFT2RIGHT_POSE"), "pose_quaternion",
		rightCameraPose.asString());
}

/**  Load all the params from a config source, in the format described in
 * saveToConfigFile()
 */
void TStereoCamera::loadFromConfigFile(
	const std::string& section, const mrpt::config::CConfigFileBase& cfg)
{
	// [<SECTION>_LEFT]
	//   ...
	// [<SECTION>_RIGHT]
	//   ...
	// [<SECTION>_LEFT2RIGHT_POSE]
	//  pose_quaternion = [x y z qr qx qy qz]

	leftCamera.loadFromConfigFile(section + string("_LEFT"), cfg);
	rightCamera.loadFromConfigFile(section + string("_RIGHT"), cfg);
	rightCameraPose.fromString(cfg.read_string(
		section + string("_LEFT2RIGHT_POSE"), "pose_quaternion", ""));
}

uint8_t TStereoCamera::serializeGetVersion() const { return 2; }
void TStereoCamera::serializeTo(mrpt::serialization::CArchive& out) const
{
	// v1->v2: rightCameraPose changed from mrpt::poses::CPose3DQuat to
	// mrpt::math::TPose3DQuat
	out << leftCamera << rightCamera << rightCameraPose;
}
void TStereoCamera::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		{
			if (version == 0)
			{
				uint8_t _model;
				in >> _model;  // unused now
			}
			if (version == 1)
			{
				// Emulate reading a CPose3DQuat object:
				THROW_EXCEPTION(
					"backwards compatibility de-serialization not implemented "
					"yet!");
			}
			in >> leftCamera >> rightCamera >> rightCameraPose;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	}
}

std::string TStereoCamera::dumpAsText() const
{
	mrpt::config::CConfigFileMemory cfg;
	saveToConfigFile("", cfg);
	return cfg.getContent();
}

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once
#include <string>


enum TypeOfConfig
{
	None = -1,
	General = 0,
	PointsMap = 1,
	Occupancy = 2,
	Landmarks = 3,
	Beacon = 4,
	GasGrid = 5
};

inline std::string typeToName(TypeOfConfig type)
{
	switch (type) {
	case PointsMap:
		return "pointsMap";
	case Occupancy:
		return "occupancyGrid";
	case Landmarks:
		return "landmarksMap";
	case Beacon:
		return "beaconMap";
	case GasGrid:
		return "gasGrid";
	default:
		break;
	}

	return "";
}



struct SType
{
	SType() : type(None), index(-1) {}
	SType(TypeOfConfig _type, int _index) : type(_type), index(_index) {}

	inline bool operator<(const SType &i) const
	{
		if (type != i.type) return type < i.type;
		return index < i.index;
	}

	TypeOfConfig type;
	int index;
};


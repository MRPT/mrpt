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

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/poses/CNetworkOfPoses.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;

IMPLEMENTS_SERIALIZABLE( CNetworkOfPoses2D, CSerializable, mrpt::poses )
IMPLEMENTS_SERIALIZABLE( CNetworkOfPoses3D, CSerializable, mrpt::poses )
IMPLEMENTS_SERIALIZABLE( CNetworkOfPoses2DInf, CSerializable, mrpt::poses )
IMPLEMENTS_SERIALIZABLE( CNetworkOfPoses3DInf, CSerializable, mrpt::poses )

// Use MRPT's STL-metaprogramming automatic serialization for the field "edges":
#define DO_IMPLEMENT_READ_WRITE(_CLASS) \
	void  _CLASS::writeToStream(CStream &out, int *version) const \
	{ \
		if (version) \
			*version = 0; \
		else \
		{ \
			out << edges;  \
		} \
	} \
	void  _CLASS::readFromStream(CStream &in, int version) \
	{ \
		switch(version) \
		{ \
		case 0: \
			{ \
				in >> edges; \
			} break; \
		default: \
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version) \
		}; \
	}


DO_IMPLEMENT_READ_WRITE(CNetworkOfPoses2D)
DO_IMPLEMENT_READ_WRITE(CNetworkOfPoses3D)
DO_IMPLEMENT_READ_WRITE(CNetworkOfPoses2DInf)
DO_IMPLEMENT_READ_WRITE(CNetworkOfPoses3DInf)


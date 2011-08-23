/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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

#include <mrpt/graphs.h>  // Precompiled headers

#include <mrpt/graphs/CNetworkOfPoses.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::graphs;

//   Implementation of serialization stuff
// --------------------------------------------------------------------------------
IMPLEMENTS_SERIALIZABLE( CNetworkOfPoses2D, CSerializable, mrpt::graphs )
IMPLEMENTS_SERIALIZABLE( CNetworkOfPoses3D, CSerializable, mrpt::graphs )
IMPLEMENTS_SERIALIZABLE( CNetworkOfPoses2DCov, CSerializable, mrpt::graphs )
IMPLEMENTS_SERIALIZABLE( CNetworkOfPoses3DCov, CSerializable, mrpt::graphs )
IMPLEMENTS_SERIALIZABLE( CNetworkOfPoses2DInf, CSerializable, mrpt::graphs )
IMPLEMENTS_SERIALIZABLE( CNetworkOfPoses3DInf, CSerializable, mrpt::graphs )


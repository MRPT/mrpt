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


#include <mrpt/vision/CDetectableObject.h>

using namespace mrpt;
using namespace mrpt::vision;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CDetectableObject, CSerializable, mrpt::vision)
IMPLEMENTS_SERIALIZABLE(CDetectable2D, CDetectableObject,mrpt::vision)
IMPLEMENTS_SERIALIZABLE(CDetectable3D, CDetectable2D,mrpt::vision)

void CDetectable2D::readFromStream(CStream &in, int version)
{

}
void CDetectable2D::writeToStream(CStream &out, int *version) const
{
}

void CDetectable3D::readFromStream(CStream &in, int version)
{

}
void CDetectable3D::writeToStream(CStream &out, int *version) const
{
}


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

#include <mrpt/opengl.h>  // Precompiled header



#include <mrpt/opengl/CCamera.h>

#include "opengl_internals.h"

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CCamera, CRenderizable, mrpt::opengl )


/*--------------------------------------------------------------
					CCamera
  ---------------------------------------------------------------*/
CCamera::CCamera()	:
	m_pointingX(0),m_pointingY(0),m_pointingZ(0),
	m_distanceZoom(10),
	m_azimuthDeg(45),m_elevationDeg(45),
	m_projectiveModel(true),
	m_projectiveFOVdeg(30),
	m_6DOFMode(false)
{
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CCamera::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 1;
	else
	{
		// Save data:
		out << m_pointingX << m_pointingY << m_pointingZ
			<< m_distanceZoom
			<< m_azimuthDeg << m_elevationDeg
			<< m_projectiveModel << m_projectiveFOVdeg;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CCamera::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 1:
		{
			// Load data:
			in  >> m_pointingX >> m_pointingY >> m_pointingZ
				>> m_distanceZoom
				>> m_azimuthDeg >> m_elevationDeg
                >> m_projectiveModel >> m_projectiveFOVdeg;
		}
		break;
	case 0:
		{
			in  >> m_pointingX >> m_pointingY >> m_pointingZ
				>> m_distanceZoom
				>> m_azimuthDeg >> m_elevationDeg;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"	 // Precompiled header
//
#include <mrpt/opengl/CCamera.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/optional_serialization.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CCamera, CRenderizable, mrpt::opengl)

uint8_t CCamera::serializeGetVersion() const { return 2; }
void CCamera::serializeTo(mrpt::serialization::CArchive& out) const
{
	// Save data:
	out << m_pointingX << m_pointingY << m_pointingZ << m_eyeDistance
		<< m_azimuthDeg << m_elevationDeg << m_projectiveModel
		<< m_projectiveFOVdeg;
	out << m_pinholeModel;	// v2
}

void CCamera::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 1:
		case 2:
		{
			// Load data:
			in >> m_pointingX >> m_pointingY >> m_pointingZ >> m_eyeDistance >>
				m_azimuthDeg >> m_elevationDeg >> m_projectiveModel >>
				m_projectiveFOVdeg;
			if (version >= 2) in >> m_pinholeModel;
			else
				m_pinholeModel.reset();
		}
		break;
		case 0:
		{
			in >> m_pointingX >> m_pointingY >> m_pointingZ >> m_eyeDistance >>
				m_azimuthDeg >> m_elevationDeg;
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

/** In this class, returns a fixed box (max,max,max), (-max,-max,-max). */
auto CCamera::getBoundingBox() const -> mrpt::math::TBoundingBox
{
	return mrpt::math::TBoundingBox::PlusMinusInfinity();
}

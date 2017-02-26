/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "detectors-precomp.h"  // Precompiled headers

#include <mrpt/detectors/CDetectableObject.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::detectors;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CDetectableObject, CSerializable, mrpt::detectors)
IMPLEMENTS_SERIALIZABLE(CDetectable2D, CDetectableObject,mrpt::detectors)
IMPLEMENTS_SERIALIZABLE(CDetectable3D, CDetectable2D,mrpt::detectors)


void CDetectable2D::readFromStream(mrpt::utils::CStream &, int )
{
}

void CDetectable2D::writeToStream(mrpt::utils::CStream &, int *) const
{
}

void CDetectable3D::readFromStream(mrpt::utils::CStream &, int )
{
}

void CDetectable3D::writeToStream(mrpt::utils::CStream &, int *) const
{
}

CDetectable3D::CDetectable3D( const CDetectable2DPtr &object2d )
	: CDetectable2D( object2d.pointer() ), m_z(0)
{ 
}

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/detectors.h>  // Precompiled headers

#include <mrpt/detectors/CDetectableObject.h>
#include <mrpt/utils/CStartUpClassesRegister.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::detectors;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CDetectableObject, CSerializable, mrpt::detectors)
IMPLEMENTS_SERIALIZABLE(CDetectable2D, CDetectableObject,mrpt::detectors)
IMPLEMENTS_SERIALIZABLE(CDetectable3D, CDetectable2D,mrpt::detectors)


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


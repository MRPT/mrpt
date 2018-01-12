/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "img-precomp.h"

#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/core/initializer.h>

using namespace mrpt::img;

MRPT_INITIALIZER(registerAllClasses_mrpt_imgs)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	//   Hack to enable compatibility with an older name of this class:
	registerClassCustomName("CMRPTImage", CLASS_ID(CImage));
	registerClass(CLASS_ID(CImage));
	registerClass(CLASS_ID(TCamera));
	registerClass(CLASS_ID(TStereoCamera));
#endif
}

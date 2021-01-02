/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "img-precomp.h"

#include <mrpt/core/initializer.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TStereoCamera.h>

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

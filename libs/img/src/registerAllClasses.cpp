/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"

#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/imgs.h>
#include <mrpt/utils/initializer.h>

#ifndef MRPT_ENABLE_PRECOMPILED_HDRS
#define MRPT_ALWAYS_INCLUDE_ALL_HEADERS
#undef mrpt_base_H
#include "imgs-precomp.h"
#endif

using namespace mrpt::imgs;

MRPT_INITIALIZER(registerAllClasses_mrpt_imgs)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	//   Hack to enable compatibility with an older name of this class:
	registerClassCustomName("CMRPTImage", CLASS_ID(CImage));
	registerClass(CLASS_ID(CImage));
	registerClass(CLASS_ID(TCamera));
	registerClass(CLASS_ID(TStereoCamera));

//	registerClass(CLASS_ID(CSimpleDatabase));
//	registerClass(CLASS_ID(CSimpleDatabaseTable));
//	registerClass(CLASS_ID(CPropertiesValuesList));
//	registerClass(CLASS_ID(CMHPropertiesValuesList));
//	registerClass(CLASS_ID(CTypeSelector));
//	registerClass(CLASS_ID(CMemoryChunk));
#endif
}

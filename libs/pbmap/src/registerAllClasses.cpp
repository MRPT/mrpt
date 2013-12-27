/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/pbmap.h>
#include <mrpt/utils.h>


using namespace mrpt::utils;
using namespace mrpt::pbmap;

void registerAllClasses_mrpt_pbmap();

CStartUpClassesRegister  mrpt_pbmap_class_reg(&registerAllClasses_mrpt_pbmap);

/*---------------------------------------------------------------
					registerAllClasses_mrpt_pbmap
  ---------------------------------------------------------------*/
void registerAllClasses_mrpt_pbmap()
{
	registerClass( CLASS_ID( Plane ) );
	registerClass( CLASS_ID( PbMap ) );
}


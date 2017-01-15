/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CAction.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::obs;
using namespace mrpt::utils;


IMPLEMENTS_VIRTUAL_SERIALIZABLE(CAction, CSerializable, mrpt::obs)


/*---------------------------------------------------------------
			Constructor
  ---------------------------------------------------------------*/
CAction::CAction() : timestamp( INVALID_TIMESTAMP )
{

}

/*---------------------------------------------------------------
			Destructor
  ---------------------------------------------------------------*/
CAction::~CAction()
{
}

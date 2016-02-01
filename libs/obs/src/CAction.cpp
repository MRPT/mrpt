/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CAction.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CStartUpClassesRegister.h>


using namespace mrpt::obs;
using namespace mrpt::utils;


IMPLEMENTS_VIRTUAL_SERIALIZABLE(CAction, CSerializable, mrpt::obs)


extern CStartUpClassesRegister  mrpt_obs_class_reg;

const int dumm = mrpt_obs_class_reg.do_nothing(); // Avoid compiler removing this class in static linking


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

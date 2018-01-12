/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

// Support building in old wx2.8.0 without changing my code:
#if !wxCHECK_VERSION(2, 9, 0)
#include <wx/font.h>
#define wxFONTWEIGHT_BOLD wxBOLD
#define wxFONTFAMILY_TELETYPE wxTELETYPE
#endif

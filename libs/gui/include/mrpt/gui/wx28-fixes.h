/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

// Support building in old wx2.8.0 without changing my code:
#if !wxCHECK_VERSION(2, 9, 0)
#include <wx/font.h>
#define wxFONTWEIGHT_BOLD wxBOLD
#define wxFONTFAMILY_TELETYPE wxTELETYPE
#endif

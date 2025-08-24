/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

// Support building in old wx2.8.0 without changing my code:
#if !wxCHECK_VERSION(2, 9, 0)
#include <wx/font.h>
#define wxFONTWEIGHT_BOLD     wxBOLD
#define wxFONTFAMILY_TELETYPE wxTELETYPE
#endif

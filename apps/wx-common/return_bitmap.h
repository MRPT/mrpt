/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once
#include <wx/bitmap.h>

#define RETURN_BITMAP(artid, xpm)                                              \
	if (id == artid)                                                           \
	{                                                                          \
		if (client == wxART_MENU)                                              \
		{                                                                      \
			wxBitmap b(xpm);                                                   \
			return wxBitmap(                                                   \
				b.ConvertToImage().Scale(16, 16, wxIMAGE_QUALITY_HIGH));       \
		}                                                                      \
		else                                                                   \
		{                                                                      \
			return wxBitmap(xpm);                                              \
		}                                                                      \
	}

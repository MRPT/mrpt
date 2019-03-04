/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef MYARTPROVIDER_H
#define MYARTPROVIDER_H

#include "CDlgLog.h"

#include <wx/artprov.h>

class CMyArtProvider : public wxArtProvider
{
   protected:
	wxBitmap CreateBitmap(
		const wxArtID& id, const wxArtClient& client,
		const wxSize& size) override;
};

#endif  // MYARTPROVIDER_H

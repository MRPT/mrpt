/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
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

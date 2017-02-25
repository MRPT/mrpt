/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "xRawLogViewerMain.h"


#include <wx/tooltip.h>

#include "imgs/Applications.xpm"
#include "imgs/ArrowLeft2.xpm"
#include "imgs/Exec.xpm"
#include "imgs/Folderdownloads.xpm"
#include "imgs/file_save.xpm"
#include "imgs/Sports-Car-2.xpm"
#include "imgs/Qmark.xpm"
#include "imgs/MAIN_ICON.xpm"
#include "imgs/icon_play.xpm"
#include "imgs/icon_animate_scans.xpm"
#include "imgs/icon_icp.xpm"

#include "../wx-common/mrpt_logo.xpm"


#define RETURN_BITMAP(artid,xpm) \
	if (id == artid) \
	{ \
		if (client==wxART_MENU) \
		{	wxBitmap	b(xpm); \
			return wxBitmap( b.ConvertToImage().Scale(16,16, wxIMAGE_QUALITY_HIGH ) ); \
		} else \
		{  return wxBitmap(xpm); } \
	} \


// CreateBitmap function
wxBitmap MyArtProvider::CreateBitmap(const wxArtID& id,
									 const wxArtClient& client,
									 const wxSize& size)
{
	RETURN_BITMAP( wxART_FOLDER,						Folderdownloads_xpm );
	RETURN_BITMAP( wxART_FILE_SAVE_AS,					file_save_xpm );
	RETURN_BITMAP( wxART_MAKE_ART_ID(ICON_MOTION),		Sports_Car_2_xpm );
	RETURN_BITMAP( wxART_MAKE_ART_ID(ICON_ABOUT),		Qmark_xpm );
	RETURN_BITMAP( wxART_MAKE_ART_ID(ICON_QUIT),		ArrowLeft2_xpm );
	RETURN_BITMAP( wxART_COPY,							Applications_xpm );
	RETURN_BITMAP( wxART_MAKE_ART_ID(ICON_RAWMAP),		Exec_xpm );
	RETURN_BITMAP( wxART_TIP,							icon_play_xpm );
	RETURN_BITMAP( wxART_MAKE_ART_ID(ICON_ICP),			icon_icp_xpm );
	RETURN_BITMAP( wxART_MAKE_ART_ID(ICON_ANIMATE_SCANS),icon_animate_scans_xpm );
	RETURN_BITMAP( wxART_MAKE_ART_ID(MAIN_ICON),		MAIN_ICON_xpm );
	RETURN_BITMAP( wxART_MAKE_ART_ID(IMG_MRPT_LOGO),	mrpt_logo_xpm );

	// Any wxWidgets icons not implemented here
	// will be provided by the default art provider.
	return wxNullBitmap;
}

/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <wx/tooltip.h>

#include "../wx-common/return_bitmap.h"
//
#include "../wx-common/Applications.xpm"
#include "../wx-common/ArrowLeft2.xpm"
#include "../wx-common/Exec.xpm"
#include "../wx-common/Folderdownloads.xpm"
#include "../wx-common/Qmark.xpm"
#include "../wx-common/Sports-Car-2.xpm"
#include "../wx-common/file_save.xpm"
#include "../wx-common/icon_animate_scans.xpm"
#include "../wx-common/icon_icp.xpm"
#include "../wx-common/icon_play.xpm"
#include "../wx-common/mrpt_logo.xpm"
#include "imgs/MAIN_ICON.xpm"
#include "xRawLogViewerMain.h"

// CreateBitmap function
wxBitmap MyArtProvider::CreateBitmap(
	const wxArtID& id, const wxArtClient& client, const wxSize& size)
{
	RETURN_BITMAP(wxART_FOLDER, Folderdownloads_xpm);
	RETURN_BITMAP(wxART_FILE_SAVE_AS, file_save_xpm);
	RETURN_BITMAP(wxART_MAKE_ART_ID(ICON_MOTION), Sports_Car_2_xpm);
	RETURN_BITMAP(wxART_MAKE_ART_ID(ICON_ABOUT), Qmark_xpm);
	RETURN_BITMAP(wxART_MAKE_ART_ID(ICON_QUIT), ArrowLeft2_xpm);
	RETURN_BITMAP(wxART_COPY, Applications_xpm);
	RETURN_BITMAP(wxART_MAKE_ART_ID(ICON_RAWMAP), Exec_xpm);
	RETURN_BITMAP(wxART_TIP, icon_play_xpm);
	RETURN_BITMAP(wxART_MAKE_ART_ID(ICON_ICP), icon_icp_xpm);
	RETURN_BITMAP(
		wxART_MAKE_ART_ID(ICON_ANIMATE_SCANS), icon_animate_scans_xpm);
	RETURN_BITMAP(wxART_MAKE_ART_ID(MAIN_ICON), MAIN_ICON_xpm);
	RETURN_BITMAP(wxART_MAKE_ART_ID(IMG_MRPT_LOGO), mrpt_logo_xpm);

	// Any wxWidgets icons not implemented here
	// will be provided by the default art provider.
	return wxNullBitmap;
}

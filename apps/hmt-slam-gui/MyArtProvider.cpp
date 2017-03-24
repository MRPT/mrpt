/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */



#include "hmt_slam_guiMain.h"  // Needed to avoid UNICODE WX build errors sometimes
#include "MyArtProvider.h"

#include "../wx-common/mrpt_logo.xpm"

#include "imgs/main_icon.xpm"

#include "imgs/icon_load.xpm"
#include "imgs/icon_log.xpm"
#include "imgs/icon_save.xpm"
#include "imgs/icon_reset.xpm"
#include "imgs/icon_step.xpm"
#include "imgs/icon_stop.xpm"
#include "imgs/icon_about.xpm"
#include "imgs/icon_exit.xpm"

// CreateBitmap function
wxBitmap CMyArtProvider::CreateBitmap(const wxArtID& id, const wxArtClient& client, const wxSize& size)
{
	// Icons:
	if (id == wxART_MAKE_ART_ID(IMG_MRPT_LOGO)) return wxBitmap(mrpt_logo_xpm);
	if (id == wxART_MAKE_ART_ID(MAIN_ICON)) return wxBitmap(main_icon_xpm);

	// toolbar buttons:
	if (id == wxART_MAKE_ART_ID(ICON_RESET)) return wxBitmap(icon_reset_xpm);
	if (id == wxART_MAKE_ART_ID(ICON_LOG)) return wxBitmap(icon_log_xpm);

	if (id == wxART_MAKE_ART_ID(ICON_LOAD)) return wxBitmap(icon_load_xpm);
	if (id == wxART_MAKE_ART_ID(ICON_SAVE)) return wxBitmap(icon_save_xpm);

	if (id == wxART_MAKE_ART_ID(ICON_PLAY)) return wxBitmap(icon_step_xpm);
	if (id == wxART_MAKE_ART_ID(ICON_STOP)) return wxBitmap(icon_stop_xpm);

	if (id == wxART_MAKE_ART_ID(ICON_ABOUT)) return wxBitmap(icon_about_xpm);

	if (id == wxART_MAKE_ART_ID(ICON_QUIT)) return wxBitmap(icon_exit_xpm);



    // Any wxWidgets icons not implemented here
    // will be provided by the default art provider.
    return wxNullBitmap;
}

/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

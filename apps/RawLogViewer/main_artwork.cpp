/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

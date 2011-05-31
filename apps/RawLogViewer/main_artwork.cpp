/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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

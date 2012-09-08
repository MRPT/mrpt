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
#include "CAboutBox.h"

//(*InternalHeaders(CAboutBox)
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/font.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/artprov.h>
//*)

#include "gridmapSimulMain.h"
#include "../wx-common/CMyRedirector.h"


#include <mrpt/utils.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;


//(*IdInit(CAboutBox)
const long CAboutBox::ID_STATICTEXT1 = wxNewId();
const long CAboutBox::ID_STATICTEXT2 = wxNewId();
const long CAboutBox::ID_STATICBITMAP1 = wxNewId();
const long CAboutBox::ID_STATICLINE1 = wxNewId();
const long CAboutBox::ID_TEXTCTRL1 = wxNewId();
const long CAboutBox::ID_TEXTCTRL2 = wxNewId();
const long CAboutBox::ID_TEXTCTRL3 = wxNewId();
const long CAboutBox::ID_NOTEBOOK1 = wxNewId();
const long CAboutBox::ID_BUTTON1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CAboutBox,wxDialog)
    //(*EventTable(CAboutBox)
    //*)
END_EVENT_TABLE()

CAboutBox::CAboutBox(wxWindow* parent,wxWindowID id)
{
    //(*Initialize(CAboutBox)
    wxFlexGridSizer* FlexGridSizer2;

    Create(parent, id, _("About box..."), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE|wxCLOSE_BOX, _T("id"));
    SetClientSize(wxSize(636,375));
    FlexGridSizer1 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer4 = new wxFlexGridSizer(0, 2, 0, 0);
    FlexGridSizer4->AddGrowableCol(1);
    FlexGridSizer4->AddGrowableRow(0);
    FlexGridSizer2 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer2->AddGrowableCol(0);
    FlexGridSizer2->AddGrowableRow(0);
    FlexGridSizer2->AddGrowableRow(1);
    lbProgName = new wxStaticText(this, ID_STATICTEXT1, _("RawLog Viewer v2.0"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    wxFont lbProgNameFont(22,wxSWISS,wxFONTSTYLE_NORMAL,wxBOLD,false,_T("Times New Roman"),wxFONTENCODING_DEFAULT);
    lbProgName->SetFont(lbProgNameFont);
    FlexGridSizer2->Add(lbProgName, 1, wxALL|wxALIGN_BOTTOM|wxALIGN_CENTER_HORIZONTAL, 5);
    lbBuild = new wxStaticText(this, ID_STATICTEXT2, _("Label"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer2->Add(lbBuild, 1, wxALL|wxALIGN_TOP|wxALIGN_CENTER_HORIZONTAL, 5);
    FlexGridSizer4->Add(FlexGridSizer2, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    StaticBitmap1 = new wxStaticBitmap(this, ID_STATICBITMAP1, wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("IMG_MRPT_LOGO")),wxART_OTHER), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICBITMAP1"));
    FlexGridSizer4->Add(StaticBitmap1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer1->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    StaticLine1 = new wxStaticLine(this, ID_STATICLINE1, wxPoint(3,86), wxSize(627,2), wxLI_HORIZONTAL, _T("ID_STATICLINE1"));
    FlexGridSizer1->Add(StaticLine1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Notebook1 = new wxNotebook(this, ID_NOTEBOOK1, wxPoint(6,91), wxSize(625,250), 0, _T("ID_NOTEBOOK1"));
    lbInfo = new wxTextCtrl(Notebook1, ID_TEXTCTRL1, wxEmptyString, wxPoint(4,24), wxSize(545,222), wxTE_MULTILINE|wxTE_READONLY|wxTE_AUTO_URL, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    wxFont lbInfoFont(10,wxSWISS,wxFONTSTYLE_NORMAL,wxNORMAL,false,_T("Courier New"),wxFONTENCODING_DEFAULT);
    lbInfo->SetFont(lbInfoFont);
    lbLicense = new wxTextCtrl(Notebook1, ID_TEXTCTRL2, _("License for the Mobile Robot Programming Toolkit (MRPT)\n\nCopyright (C) 2005-2012  University of Malaga                        \n                                                       \nThis software was written by the Perception and Robotics            \nresearch group, University of Malaga (Spain).                     \nContact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                  \n                                                               \n\nMRPT is free software: you can redistribute it and/or modify       \nit under the terms of the GNU General Public License as published by\nthe Free Software Foundation, either version 3 of the License, or  \n(at your option) any later version.                                \n                                                               \nMRPT is distributed in the hope that it will be useful,              \nbut WITHOUT ANY WARRANTY; without even the implied warranty of     \nMERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the      \nGNU General Public License for more details.                       \n                                                               \nYou should have received a copy of the GNU General Public License  \nalong with MRPT.  If not, see <http://www.gnu.org/licenses/>.      \n"), wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY|wxTE_AUTO_URL, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    wxFont lbLicenseFont(10,wxSWISS,wxFONTSTYLE_NORMAL,wxNORMAL,false,_T("Courier New"),wxFONTENCODING_DEFAULT);
    lbLicense->SetFont(lbLicenseFont);
    TextCtrl1 = new wxTextCtrl(Notebook1, ID_TEXTCTRL3, _("Up to date documentation and tutorials are maintained at the MRPT website:\n\nhttp://www.mrpt.org/\nor\nhttp://mrpt.sourceforge.net\n\n\n\n\n"), wxPoint(4,24), wxSize(545,222), wxTE_MULTILINE|wxTE_READONLY|wxTE_AUTO_URL, wxDefaultValidator, _T("ID_TEXTCTRL3"));
    wxFont TextCtrl1Font(10,wxSWISS,wxFONTSTYLE_NORMAL,wxNORMAL,false,_T("Courier New"),wxFONTENCODING_DEFAULT);
    TextCtrl1->SetFont(TextCtrl1Font);
    Notebook1->AddPage(lbInfo, _("Information"), false);
    Notebook1->AddPage(lbLicense, _("License"), false);
    Notebook1->AddPage(TextCtrl1, _("Tutorial"), false);
    FlexGridSizer1->Add(Notebook1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Button1 = new wxButton(this, ID_BUTTON1, _("OK"), wxPoint(250,345), wxSize(76,26), 0, wxDefaultValidator, _T("ID_BUTTON1"));
    FlexGridSizer1->Add(Button1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    SetSizer(FlexGridSizer1);
    FlexGridSizer1->SetSizeHints(this);
    Center();

    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CAboutBox::OnButton1Click);
    Connect(wxID_ANY,wxEVT_INIT_DIALOG,(wxObjectEventFunction)&CAboutBox::OnInit);
    Connect(wxEVT_CHAR,(wxObjectEventFunction)&CAboutBox::OnChar);
    //*)
}

CAboutBox::~CAboutBox()
{
    //(*Destroy(CAboutBox)
    //*)
}


void CAboutBox::OnInit(wxInitDialogEvent& event)
{
    // Build strings:
    wxString MRPTver( MRPT_getVersion().c_str(), wxConvLibc);
    wxString wxVer( wxVERSION_STRING );

#if defined(__WXMSW__)
    wxVer << _T("-Windows");
#elif defined(__UNIX__)
    wxVer << _T("-Linux");
#endif
#if wxUSE_UNICODE
    wxVer << _T("-Unicode build");
#else
    wxVer << _T("-ANSI build");
#endif // wxUSE_UNICODE

    // Set the label with MRPT version:
    wxString    s(_("Build: "));
    s << wxString(__DATE__,wxConvLibc);
    s << _(" - ") << MRPTver;

    lbBuild->SetLabel( s );

    // Info:
    lbInfo->Clear();

    {
		CMyRedirector myRedirector( lbInfo );

		cout << "  GridmapSimul Application\n";
		cout << "--------------------------------\n";
		cout << "Jose Luis Blanco (C) 2005-2012\n";
		cout << "For bug reports or to collaborate: <jlblanco@ctima.uma.es>\n";
		cout << "http://www.isa.uma.es/jlblanco\n\n";

		cout << "Version:                " << GRIDMAPSIMUL_VERSION << endl;
		cout << "MRPT version:           " << MRPT_getVersion() << endl;
		cout << "MRPT compilation date:  " << MRPT_getCompilationDate() << endl;
 		cout << "Eigen version:          " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << endl;
		cout << "wxWidgets version:      " << wxVer.mb_str() << endl;
    }

    lbProgName->SetLabel( _U( format("GridmapSimul %s",GRIDMAPSIMUL_VERSION).c_str() ) );
	lbProgName->SetForegroundColour( wxColour(0,0,128) );

    FlexGridSizer1->RecalcSizes();
}

void CAboutBox::OnButton1Click(wxCommandEvent& event)
{
    Close();
}

void CAboutBox::OnChar(wxKeyEvent& event)
{
}

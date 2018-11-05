/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/gui/keycodes.h>

#if MRPT_HAS_WXWIDGETS

#include <wx/sizer.h>
#include <wx/statbmp.h>
#include <wx/menu.h>
#include <wx/toolbar.h>
#include <wx/frame.h>
#include <wx/timer.h>
#include <wx/statusbr.h>
#include <wx/msgdlg.h>
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
#include <wx/msgdlg.h>
#include <wx/panel.h>
#include <wx/filedlg.h>
#include <wx/progdlg.h>
#include <wx/imaglist.h>
#include <wx/busyinfo.h>
#include <wx/log.h>
#include <wx/textdlg.h>
#include <wx/dirdlg.h>
#include <wx/colordlg.h>
#include <wx/dcmemory.h>
#include <wx/app.h>
#include <wx/pen.h>
#include <wx/spinctrl.h>
#include <wx/sizer.h>
#include <wx/notebook.h>
#include <wx/button.h>
#include <wx/panel.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/choice.h>
#include <wx/radiobox.h>
#include <wx/settings.h>
#include <wx/checkbox.h>
#include <wx/dc.h>
#include <wx/dcclient.h>

#endif

#include <mutex>

namespace mrpt
{
namespace gui
{
/** \addtogroup mrpt_gui_wxutils Utilities for MRPT-wxWidgets interfacing (in
  #include <mrpt/gui/WxUtils.h>)
  * \ingroup mrpt_gui_grp

	* @{ */
#if MRPT_HAS_WXWIDGETS

#ifndef WX_START_TRY

#define WX_START_TRY \
	try              \
	{
#define WX_END_TRY                                                            \
	}                                                                         \
	catch (std::exception & e)                                                \
	{                                                                         \
		wxMessageBox(                                                         \
			mrpt::exception_to_str(e), wxT("Exception"), wxOK, nullptr);      \
	}                                                                         \
	catch (...)                                                               \
	{                                                                         \
		wxMessageBox(_("Untyped exception!"), _("Exception"), wxOK, nullptr); \
	}

#endif

/** Create a wxImage from a MRPT image. The new object must be freed by the user
 * when not required anymore.
 * \sa MRPTImage2wxImage
 */
wxImage* MRPTImage2wxImage(const mrpt::img::CImage& img);

/** Create a wxBitmap from a MRPT image. The new object must be freed by the
 * user when not required anymore.
 * \sa MRPTImage2wxImage
 */
wxBitmap* MRPTImage2wxBitmap(const mrpt::img::CImage& img);

#if MRPT_HAS_OPENCV
/** Create a wxImage from a IPL image. The new object must be freed by the user
 * when not required anymore.
 * \sa IplImage2wxImage
 */
wxImage* IplImage2wxImage(void* img);
#endif

/** Create a MRPT image from a wxImage. The new object must be freed by the user
 * when not required anymore.
 *  It is recommended to use wxImage2MRPTImagePtr instead since smart pointers
 * are safer to manage.
 * \sa wxImage2MRPTImage, wxImage2MRPTImagePtr
 */
mrpt::img::CImage* wxImage2MRPTImage(const wxImage& img);

/** Create a MRPT image from a wxImage. The new object is returned as a smart
 * pointer to a CImage object.
 * \sa wxImage2MRPTImage
 */
mrpt::img::CImage::Ptr wxImage2MRPTImagePtr(const wxImage& img);

/** Extracts the key modifiers from a wxKeyEvent */
mrptKeyModifier keyEventToMrptKeyModifier(const wxKeyEvent& ev);

/** A custom control to display the bitmap and avoid flicker
 */
class wxMRPTImageControl : public wxPanel
{
   protected:
	wxBitmap* m_img;
	std::mutex m_img_cs;

	wxPoint m_last_mouse_point, m_last_mouse_click;
	std::mutex m_mouse_cs;

   public:
	wxMRPTImageControl(
		wxWindow* parent, wxWindowID winID, int x, int y, int width,
		int height);
	~wxMRPTImageControl() override;

	/** Assigns this image. This object has the ownship of the image and will
	 * delete it when appropriate. Remember to call Refresh to display the
	 * image. */
	void AssignImage(wxBitmap* img);
	/** Assigns this image. Remember to call Refresh to display the image. */
	void AssignImage(const mrpt::img::CImage& img);
	void GetBitmap(wxBitmap& bmp);

	void OnPaint(wxPaintEvent& ev);
	void OnMouseMove(wxMouseEvent& ev);
	void OnMouseClick(wxMouseEvent& ev);

	void OnEraseBackground(wxEraseEvent&)
	{ /* Do nothing */
	}
};
// end wxMRPTImageControl  -----------

/** A panel to select the camera input from all the formats supported by MRPT */
class CPanelCameraSelection : public wxPanel
{
   public:
	CPanelCameraSelection(wxWindow* parent, wxWindowID id = wxID_ANY);
	~CPanelCameraSelection() override;

	void readConfigIntoVideoSourcePanel(
		const std::string& sect,
		const mrpt::config::CConfigFileBase* cfg) const;

	void writeConfigFromVideoSourcePanel(
		const std::string& sect, mrpt::config::CConfigFileBase* cfg) const;

	//(*Declarations(CPanelCameraSelection)
	wxTextCtrl* edRawlogLabel;
	wxStaticText* StaticText10;
	wxStaticText* StaticText9;
	wxPanel* Panel5;
	wxButton* btnBrowseRawlogDir;
	wxRadioBox* rbBumblebeeSel;
	wxButton* btnBrowseVideo;
	wxStaticText* StaticText2;
	wxPanel* Panel4;
	wxCheckBox* cbKinect_3D;
	wxRadioBox* rbKinect_int;
	wxCheckBox* cbSR_chConf;
	wxStaticText* StaticText6;
	wxSpinCtrl* opencvCamIndex;
	wxTextCtrl* edIPcamURL;
	wxStaticText* StaticText8;
	wxStaticText* StaticText11;
	wxTextCtrl* edCustomCamConfig;
	wxTextCtrl* edSR_IP;
	wxPanel* Panel1;
	wxChoice* cbOpencvCamType;
	wxStaticText* StaticText1;
	wxStaticText* StaticText3;
	wxRadioBox* rbSR_usb;
	wxPanel* Panel6;
	wxButton* btnBrowseRawlog;
	wxPanel* Panel3;
	wxCheckBox* cbGrayscale;
	wxCheckBox* cbSR_chRange;
	wxStaticText* StaticText5;
	wxStaticText* StaticText7;
	wxPanel* pnKinect;
	wxTextCtrl* edVideoFile;
	wxCheckBox* cbBumblebeeRectif;
	wxCheckBox* cbKinect_Int;
	wxCheckBox* cbSR_chIntensity;
	wxCheckBox* cbKinect_Depth;
	wxNotebook* pagesCameras;
	wxPanel* pnSwissRanger;
	wxTextCtrl* edRawlogFile;
	wxTextCtrl* edRawlogImgDir;
	wxPanel* Panel2;
	wxCheckBox* cbSR_ch3D;
	wxStaticText* StaticText4;
	wxChoice* cbOpencvResolution;
	//*)

   protected:
	//(*Identifiers(CPanelCameraSelection)
	static const long ID_STATICTEXT1;
	static const long ID_SPINCTRL1;
	static const long ID_STATICTEXT3;
	static const long ID_CHOICE1;
	static const long ID_STATICTEXT6;
	static const long ID_CHOICE2;
	static const long ID_PANEL2;
	static const long ID_STATICTEXT7;
	static const long ID_TEXTCTRL1;
	static const long ID_PANEL3;
	static const long ID_TEXTCTRL6;
	static const long ID_PANEL4;
	static const long ID_STATICTEXT8;
	static const long ID_TEXTCTRL2;
	static const long ID_BUTTON7;
	static const long ID_PANEL5;
	static const long ID_STATICTEXT9;
	static const long ID_TEXTCTRL3;
	static const long ID_BUTTON8;
	static const long ID_STATICTEXT5;
	static const long ID_TEXTCTRL7;
	static const long ID_BUTTON9;
	static const long ID_STATICTEXT10;
	static const long ID_TEXTCTRL8;
	static const long ID_STATICTEXT11;
	static const long ID_PANEL6;
	static const long ID_RADIOBOX1;
	static const long ID_CHECKBOX1;
	static const long ID_STATICTEXT2;
	static const long ID_PANEL7;
	static const long ID_RADIOBOX2;
	static const long ID_STATICTEXT4;
	static const long ID_TEXTCTRL4;
	static const long ID_CHECKBOX3;
	static const long ID_CHECKBOX4;
	static const long ID_CHECKBOX5;
	static const long ID_CHECKBOX6;
	static const long ID_PANEL1;
	static const long ID_CHECKBOX7;
	static const long ID_CHECKBOX8;
	static const long ID_CHECKBOX9;
	static const long ID_RADIOBOX3;
	static const long ID_PANEL8;
	static const long ID_NOTEBOOK1;
	static const long ID_CHECKBOX2;
	//*)

   private:
	//(*Handlers(CPanelCameraSelection)
	//*)
	void OnbtnBrowseVideoClick(wxCommandEvent& event);
	void OnbtnBrowseRawlogClick(wxCommandEvent& event);
	void OnbtnBrowseRawlogDirClick(wxCommandEvent& event);

	DECLARE_EVENT_TABLE()
};
// end   -----------

/** Auxiliary structures used internally to mrpt */
namespace detail
{
struct TReturnAskUserOpenCamera
{
	mrpt::config::CConfigFileMemory selectedConfig;
	bool accepted_by_user;
};
}  // namespace detail

#endif
/** @} */
}  // namespace gui
}  // namespace mrpt

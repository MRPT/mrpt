/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/round.h>
#include <mrpt/gui/WxUtils.h>
#include <mrpt/img/CImage.h>
#include <mrpt/system/filesystem.h>

#if MRPT_HAS_WXWIDGETS

using namespace mrpt;
using namespace mrpt::gui;
using namespace std;

wxImage* mrpt::gui::MRPTImage2wxImage(const mrpt::img::CImage& img)
{
  using namespace std::string_literals;

  mrpt::img::CImage new_image(img, mrpt::img::SHALLOW_COPY);

  // If the image is GRAYSCALE, we need to convert it into RGB, so do it
  // manually:
  if (!new_image.isColor())
  {
    new_image = new_image.colorImage();
  }

  if (new_image.getChannelsOrder() == "BGR"s)
  {
    auto im = new_image.makeDeepCopy();
    im.swapRB();
    new_image = im;
  }

  const int row_in_bytes =
      new_image.getWidth() * (new_image.channels() == mrpt::img::CH_RGB ? 3 : 1);
  uint8_t* data = static_cast<uint8_t*>(malloc(row_in_bytes * new_image.getHeight()));

  const int w = new_image.getWidth();
  const int h = new_image.getHeight();
  const auto rs = new_image.getRowStride();

  // Copy row by row only if necessary:
  if (row_in_bytes != rs)
  {
    auto* trg = data;
    const auto* src = new_image.ptrLine<uint8_t>(0);
    for (int y = 0; y < h; y++, src += rs, trg += row_in_bytes)
    {
      memcpy(trg, src, row_in_bytes);
    }
  }
  else
  {
    memcpy(data, new_image.ptrLine<uint8_t>(0), row_in_bytes * h);
  }

  // create and return the object
  return new wxImage(w, h, data, false /* false=transfer mem ownership */);
}

wxBitmap* mrpt::gui::MRPTImage2wxBitmap(const mrpt::img::CImage& img)
{
  auto* i = MRPTImage2wxImage(img);
  auto* ret = new wxBitmap(*i);
  delete i;
  return ret;
}

//------------------------------------------------------------------------
// Convert wxImage -> MRPTImage
//------------------------------------------------------------------------
mrpt::img::CImage* mrpt::gui::wxImage2MRPTImage(const wxImage& img)
{
  auto* newImg = new mrpt::img::CImage();

  const auto lx = img.GetWidth();
  const auto ly = img.GetHeight();

  newImg->loadFromMemoryBuffer(lx, ly, mrpt::img::CH_RGB, img.GetData(), true /* swap RB */);

  return newImg;
}

//------------------------------------------------------------------------
// Convert wxImage -> MRPTImagePtr
//------------------------------------------------------------------------
mrpt::img::CImage::Ptr mrpt::gui::wxImage2MRPTImagePtr(const wxImage& img)
{
  return mrpt::img::CImage::Ptr(wxImage2MRPTImage(img));
}

//------------------------------------------------------------------------
// 				wxMRPTImageControl
//------------------------------------------------------------------------
wxMRPTImageControl::wxMRPTImageControl(
    wxWindow* parent, wxWindowID winID, int x, int y, int width, int height) :
    m_img(nullptr)
{
  this->Create(parent, winID, wxPoint(x, y), wxSize(width, height));

  Bind(wxEVT_PAINT, &wxMRPTImageControl::OnPaint, this);
  Bind(wxEVT_MOTION, &wxMRPTImageControl::OnMouseMove, this);
  Bind(wxEVT_LEFT_DOWN, &wxMRPTImageControl::OnMouseClick, this);
}

wxMRPTImageControl::~wxMRPTImageControl()
{
  std::lock_guard<std::mutex> lock(m_img_cs);
  if (m_img != nullptr)
  {
    delete m_img;
    m_img = nullptr;
  }
}

void wxMRPTImageControl::OnMouseMove(wxMouseEvent& ev)
{
  std::lock_guard<std::mutex> lock(m_mouse_cs);
  m_last_mouse_point = ev.GetPosition();
}

void wxMRPTImageControl::OnMouseClick(wxMouseEvent& ev)
{
  std::lock_guard<std::mutex> lock(m_mouse_cs);
  m_last_mouse_click = ev.GetPosition();
}

void wxMRPTImageControl::AssignImage(wxBitmap* img)
{
  std::lock_guard<std::mutex> lock(m_img_cs);
  if (m_img != nullptr)
  {
    delete m_img;
    m_img = nullptr;
  }

  m_img = img;
}

void wxMRPTImageControl::AssignImage(const mrpt::img::CImage& img)
{
  wxBitmap* wxImg = MRPTImage2wxBitmap(img);

  std::lock_guard<std::mutex> lock(m_img_cs);
  if (m_img != nullptr)
  {
    delete m_img;
    m_img = nullptr;
  }

  m_img = wxImg;
}

void wxMRPTImageControl::OnPaint(wxPaintEvent& ev)
{
  wxPaintDC dc(this);

  std::lock_guard<std::mutex> lock(m_img_cs);
  if (m_img == nullptr)
  {
    // Erase background:
    return;
  }

  dc.DrawBitmap(*m_img, 0, 0);
}

void wxMRPTImageControl::GetBitmap(wxBitmap& bmp)
{
  std::lock_guard<std::mutex> lock(m_img_cs);
  if (m_img == nullptr)
  {
    return;
  }
  bmp = *m_img;
}

//  *********************************************************************
//  CPanelCameraSelection
//  *********************************************************************

//(*IdInit(CPanelCameraSelection)
const long CPanelCameraSelection::ID_STATICTEXT1 = wxNewId();
const long CPanelCameraSelection::ID_SPINCTRL1 = wxNewId();
const long CPanelCameraSelection::ID_STATICTEXT3 = wxNewId();
const long CPanelCameraSelection::ID_CHOICE1 = wxNewId();
const long CPanelCameraSelection::ID_STATICTEXT6 = wxNewId();
const long CPanelCameraSelection::ID_CHOICE2 = wxNewId();
const long CPanelCameraSelection::ID_PANEL2 = wxNewId();
const long CPanelCameraSelection::ID_STATICTEXT7 = wxNewId();
const long CPanelCameraSelection::ID_TEXTCTRL1 = wxNewId();
const long CPanelCameraSelection::ID_PANEL3 = wxNewId();
const long CPanelCameraSelection::ID_TEXTCTRL6 = wxNewId();
const long CPanelCameraSelection::ID_PANEL4 = wxNewId();
const long CPanelCameraSelection::ID_STATICTEXT8 = wxNewId();
const long CPanelCameraSelection::ID_TEXTCTRL2 = wxNewId();
const long CPanelCameraSelection::ID_BUTTON7 = wxNewId();
const long CPanelCameraSelection::ID_PANEL5 = wxNewId();
const long CPanelCameraSelection::ID_STATICTEXT9 = wxNewId();
const long CPanelCameraSelection::ID_TEXTCTRL3 = wxNewId();
const long CPanelCameraSelection::ID_BUTTON8 = wxNewId();
const long CPanelCameraSelection::ID_STATICTEXT5 = wxNewId();
const long CPanelCameraSelection::ID_TEXTCTRL7 = wxNewId();
const long CPanelCameraSelection::ID_BUTTON9 = wxNewId();
const long CPanelCameraSelection::ID_STATICTEXT10 = wxNewId();
const long CPanelCameraSelection::ID_TEXTCTRL8 = wxNewId();
const long CPanelCameraSelection::ID_STATICTEXT11 = wxNewId();
const long CPanelCameraSelection::ID_PANEL6 = wxNewId();
const long CPanelCameraSelection::ID_RADIOBOX1 = wxNewId();
const long CPanelCameraSelection::ID_CHECKBOX1 = wxNewId();
const long CPanelCameraSelection::ID_STATICTEXT2 = wxNewId();
const long CPanelCameraSelection::ID_PANEL7 = wxNewId();
const long CPanelCameraSelection::ID_RADIOBOX2 = wxNewId();
const long CPanelCameraSelection::ID_STATICTEXT4 = wxNewId();
const long CPanelCameraSelection::ID_TEXTCTRL4 = wxNewId();
const long CPanelCameraSelection::ID_CHECKBOX3 = wxNewId();
const long CPanelCameraSelection::ID_CHECKBOX4 = wxNewId();
const long CPanelCameraSelection::ID_CHECKBOX5 = wxNewId();
const long CPanelCameraSelection::ID_CHECKBOX6 = wxNewId();
const long CPanelCameraSelection::ID_PANEL1 = wxNewId();
const long CPanelCameraSelection::ID_CHECKBOX7 = wxNewId();
const long CPanelCameraSelection::ID_CHECKBOX8 = wxNewId();
const long CPanelCameraSelection::ID_CHECKBOX9 = wxNewId();
const long CPanelCameraSelection::ID_RADIOBOX3 = wxNewId();
const long CPanelCameraSelection::ID_PANEL8 = wxNewId();
const long CPanelCameraSelection::ID_NOTEBOOK1 = wxNewId();
const long CPanelCameraSelection::ID_CHECKBOX2 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CPanelCameraSelection, wxPanel)
//(*EventTable(CPanelCameraSelection)
//*)
END_EVENT_TABLE()

CPanelCameraSelection::CPanelCameraSelection(wxWindow* parent, wxWindowID id)
{
  //(*Initialize(CPanelCameraSelection)
  wxStaticBoxSizer* StaticBoxSizer2;
  wxFlexGridSizer* FlexGridSizer4;
  wxFlexGridSizer* FlexGridSizer16;
  wxFlexGridSizer* FlexGridSizer10;
  wxFlexGridSizer* FlexGridSizer3;
  wxFlexGridSizer* FlexGridSizer5;
  wxFlexGridSizer* FlexGridSizer2;
  wxFlexGridSizer* FlexGridSizer18;
  wxFlexGridSizer* FlexGridSizer13;
  wxFlexGridSizer* FlexGridSizer12;
  wxStaticBoxSizer* StaticBoxSizer1;
  wxFlexGridSizer* FlexGridSizer1;
  wxFlexGridSizer* FlexGridSizer11;

  Create(parent, id, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("id"));
  FlexGridSizer1 = new wxFlexGridSizer(0, 1, 0, 0);
  FlexGridSizer1->AddGrowableCol(0);
  FlexGridSizer1->AddGrowableRow(0);
  pagesCameras =
      new wxNotebook(this, ID_NOTEBOOK1, wxDefaultPosition, wxDefaultSize, 0, _T("ID_NOTEBOOK1"));
  Panel2 = new wxPanel(
      pagesCameras, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
  FlexGridSizer10 = new wxFlexGridSizer(0, 2, 0, 0);
  FlexGridSizer10->AddGrowableCol(1);
  StaticText1 = new wxStaticText(
      Panel2, ID_STATICTEXT1, _("Camera index:"), wxDefaultPosition, wxDefaultSize, 0,
      _T("ID_STATICTEXT1"));
  FlexGridSizer10->Add(StaticText1, 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
  opencvCamIndex = new wxSpinCtrl(
      Panel2, ID_SPINCTRL1, _T("0"), wxDefaultPosition, wxDefaultSize, 0, 0, 100, 0,
      _T("ID_SPINCTRL1"));
  opencvCamIndex->SetValue(_T("0"));
  FlexGridSizer10->Add(opencvCamIndex, 1, wxALL | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
  StaticText3 = new wxStaticText(
      Panel2, ID_STATICTEXT3, _("Camera type:"), wxDefaultPosition, wxDefaultSize, 0,
      _T("ID_STATICTEXT3"));
  FlexGridSizer10->Add(
      StaticText3, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
  cbOpencvCamType = new wxChoice(
      Panel2, ID_CHOICE1, wxDefaultPosition, wxDefaultSize, 0, nullptr, 0, wxDefaultValidator,
      _T("ID_CHOICE1"));
  cbOpencvCamType->SetSelection(cbOpencvCamType->Append(_("CAMERA_CV_AUTODETECT")));
  cbOpencvCamType->Append(_("CAMERA_CV_DC1394"));
  cbOpencvCamType->Append(_("CAMERA_CV_VFL"));
  cbOpencvCamType->Append(_("CAMERA_CV_VFW"));
  cbOpencvCamType->Append(_("CAMERA_CV_MIL"));
  cbOpencvCamType->Append(_("CAMERA_CV_DSHOW"));
  FlexGridSizer10->Add(cbOpencvCamType, 1, wxALL | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
  StaticText6 = new wxStaticText(
      Panel2, ID_STATICTEXT6, _("Resolution:"), wxDefaultPosition, wxDefaultSize, 0,
      _T("ID_STATICTEXT6"));
  FlexGridSizer10->Add(StaticText6, 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
  cbOpencvResolution = new wxChoice(
      Panel2, ID_CHOICE2, wxDefaultPosition, wxDefaultSize, 0, nullptr, 0, wxDefaultValidator,
      _T("ID_CHOICE2"));
  cbOpencvResolution->SetSelection(cbOpencvResolution->Append(_("default")));
  cbOpencvResolution->Append(_("320x240"));
  cbOpencvResolution->Append(_("640x480"));
  FlexGridSizer10->Add(cbOpencvResolution, 1, wxALL | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
  FlexGridSizer10->Add(-1, -1, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 0);
  Panel2->SetSizer(FlexGridSizer10);
  FlexGridSizer10->Fit(Panel2);
  FlexGridSizer10->SetSizeHints(Panel2);
  Panel3 = new wxPanel(
      pagesCameras, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
  FlexGridSizer11 = new wxFlexGridSizer(0, 1, 0, 0);
  FlexGridSizer11->AddGrowableCol(0);
  StaticText7 = new wxStaticText(
      Panel3, ID_STATICTEXT7, _("IP Camera URL:"), wxDefaultPosition, wxDefaultSize, 0,
      _T("ID_STATICTEXT7"));
  FlexGridSizer11->Add(StaticText7, 1, wxALL | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
  edIPcamURL = new wxTextCtrl(
      Panel3, ID_TEXTCTRL1, _("rtsp://192.168.0.1/live.sdp"), wxDefaultPosition, wxDefaultSize, 0,
      wxDefaultValidator, _T("ID_TEXTCTRL1"));
  FlexGridSizer11->Add(edIPcamURL, 1, wxEXPAND, 5);
  Panel3->SetSizer(FlexGridSizer11);
  FlexGridSizer11->Fit(Panel3);
  FlexGridSizer11->SetSizeHints(Panel3);
  Panel4 = new wxPanel(
      pagesCameras, ID_PANEL4, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL4"));
  FlexGridSizer16 = new wxFlexGridSizer(0, 1, 0, 0);
  FlexGridSizer16->AddGrowableCol(0);
  FlexGridSizer16->AddGrowableRow(0);
  edCustomCamConfig = new wxTextCtrl(
      Panel4, ID_TEXTCTRL6,
      _("// Configuration block for the CCameraSensor object.\n// Check out "
        "its documentation at:\n// "
        "http://reference.mrpt.org/devel/"
        "classmrpt_1_1hwdrivers_1_1_c_camera_sensor.html\n\n[CONFIG]"
        "\ngrabber_type = opencv \ncv_camera_index = 0\ncv_camera_type = "
        "CAMERA_CV_AUTODETECT\n\n"),
      wxDefaultPosition, wxDefaultSize,
      wxTE_MULTILINE | wxHSCROLL | wxTE_DONTWRAP | wxVSCROLL | wxALWAYS_SHOW_SB, wxDefaultValidator,
      _T("ID_TEXTCTRL6"));
  wxFont edCustomCamConfigFont = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
  if (!edCustomCamConfigFont.Ok())
    edCustomCamConfigFont = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
  edCustomCamConfigFont.SetPointSize(7);
  edCustomCamConfig->SetFont(edCustomCamConfigFont);
  FlexGridSizer16->Add(edCustomCamConfig, 1, wxEXPAND, 5);
  Panel4->SetSizer(FlexGridSizer16);
  FlexGridSizer16->Fit(Panel4);
  FlexGridSizer16->SetSizeHints(Panel4);
  Panel5 = new wxPanel(
      pagesCameras, ID_PANEL5, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL5"));
  FlexGridSizer12 = new wxFlexGridSizer(0, 1, 0, 0);
  FlexGridSizer12->AddGrowableCol(0);
  StaticText8 = new wxStaticText(
      Panel5, ID_STATICTEXT8, _("Video file to open:"), wxDefaultPosition, wxDefaultSize, 0,
      _T("ID_STATICTEXT8"));
  FlexGridSizer12->Add(StaticText8, 1, wxALL | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
  edVideoFile = new wxTextCtrl(
      Panel5, ID_TEXTCTRL2, _("test.avi"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator,
      _T("ID_TEXTCTRL2"));
  FlexGridSizer12->Add(edVideoFile, 1, wxEXPAND, 5);
  btnBrowseVideo = new wxButton(
      Panel5, ID_BUTTON7, _("Browse..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator,
      _T("ID_BUTTON7"));
  FlexGridSizer12->Add(
      btnBrowseVideo, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
  Panel5->SetSizer(FlexGridSizer12);
  FlexGridSizer12->Fit(Panel5);
  FlexGridSizer12->SetSizeHints(Panel5);
  Panel6 = new wxPanel(
      pagesCameras, ID_PANEL6, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL6"));
  FlexGridSizer13 = new wxFlexGridSizer(0, 3, 0, 0);
  FlexGridSizer13->AddGrowableCol(1);
  StaticText9 = new wxStaticText(
      Panel6, ID_STATICTEXT9, _("Rawlog \nfile:"), wxDefaultPosition, wxDefaultSize, 0,
      _T("ID_STATICTEXT9"));
  FlexGridSizer13->Add(StaticText9, 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
  edRawlogFile = new wxTextCtrl(
      Panel6, ID_TEXTCTRL3, _("test.rawlog"), wxDefaultPosition, wxDefaultSize, 0,
      wxDefaultValidator, _T("ID_TEXTCTRL3"));
  FlexGridSizer13->Add(edRawlogFile, 1, wxEXPAND, 5);
  btnBrowseRawlog = new wxButton(
      Panel6, ID_BUTTON8, _("Browse..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator,
      _T("ID_BUTTON8"));
  FlexGridSizer13->Add(
      btnBrowseRawlog, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
  StaticText5 = new wxStaticText(
      Panel6, ID_STATICTEXT5, _("External \nimages:"), wxDefaultPosition, wxDefaultSize, 0,
      _T("ID_STATICTEXT5"));
  FlexGridSizer13->Add(StaticText5, 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
  edRawlogImgDir = new wxTextCtrl(
      Panel6, ID_TEXTCTRL7, _("./Images"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator,
      _T("ID_TEXTCTRL7"));
  FlexGridSizer13->Add(edRawlogImgDir, 1, wxEXPAND, 5);
  btnBrowseRawlogDir = new wxButton(
      Panel6, ID_BUTTON9, _("Browse..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator,
      _T("ID_BUTTON9"));
  FlexGridSizer13->Add(
      btnBrowseRawlogDir, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
  StaticText10 = new wxStaticText(
      Panel6, ID_STATICTEXT10, _("Sensor\nlabel:"), wxDefaultPosition, wxDefaultSize, 0,
      _T("ID_STATICTEXT10"));
  FlexGridSizer13->Add(StaticText10, 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
  edRawlogLabel = new wxTextCtrl(
      Panel6, ID_TEXTCTRL8, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator,
      _T("ID_TEXTCTRL8"));
  FlexGridSizer13->Add(edRawlogLabel, 1, wxEXPAND, 5);
  StaticText11 = new wxStaticText(
      Panel6, ID_STATICTEXT11, _("(empty=any)"), wxDefaultPosition, wxDefaultSize, 0,
      _T("ID_STATICTEXT11"));
  FlexGridSizer13->Add(
      StaticText11, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
  Panel6->SetSizer(FlexGridSizer13);
  FlexGridSizer13->Fit(Panel6);
  FlexGridSizer13->SetSizeHints(Panel6);
  Panel1 = new wxPanel(
      pagesCameras, ID_PANEL7, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL7"));
  FlexGridSizer18 = new wxFlexGridSizer(2, 2, 0, 0);
  wxString __wxRadioBoxChoices_1[2] = {_("Left"), _("Right")};
  rbBumblebeeSel = new wxRadioBox(
      Panel1, ID_RADIOBOX1, _("Select monocular input"), wxDefaultPosition, wxDefaultSize, 2,
      __wxRadioBoxChoices_1, 1, 0, wxDefaultValidator, _T("ID_RADIOBOX1"));
  rbBumblebeeSel->SetSelection(0);
  FlexGridSizer18->Add(
      rbBumblebeeSel, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
  cbBumblebeeRectif = new wxCheckBox(
      Panel1, ID_CHECKBOX1, _("Use vendor\'s rectify"), wxDefaultPosition, wxDefaultSize, 0,
      wxDefaultValidator, _T("ID_CHECKBOX1"));
  cbBumblebeeRectif->SetValue(false);
  FlexGridSizer18->Add(cbBumblebeeRectif, 1, wxALL | wxALIGN_TOP | wxALIGN_CENTER_HORIZONTAL, 10);
  FlexGridSizer18->Add(-1, -1, 1, wxALL | wxALIGN_LEFT | wxALIGN_BOTTOM, 5);
  StaticText2 = new wxStaticText(
      Panel1, ID_STATICTEXT2, _("(Unchecked = raw images)"), wxDefaultPosition, wxDefaultSize, 0,
      _T("ID_STATICTEXT2"));
  FlexGridSizer18->Add(StaticText2, 1, wxALL | wxALIGN_LEFT | wxALIGN_BOTTOM, 5);
  Panel1->SetSizer(FlexGridSizer18);
  FlexGridSizer18->Fit(Panel1);
  FlexGridSizer18->SetSizeHints(Panel1);

  pnKinect = new wxPanel(
      pagesCameras, ID_PANEL8, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL8"));
  FlexGridSizer4 = new wxFlexGridSizer(2, 3, 0, 0);
  StaticBoxSizer2 = new wxStaticBoxSizer(wxHORIZONTAL, pnKinect, _("Channels to grab: "));
  FlexGridSizer5 = new wxFlexGridSizer(4, 1, 0, 0);
  cbKinect_Int = new wxCheckBox(
      pnKinect, ID_CHECKBOX7, _("Intensity"), wxDefaultPosition, wxDefaultSize, 0,
      wxDefaultValidator, _T("ID_CHECKBOX7"));
  cbKinect_Int->SetValue(true);
  cbKinect_Int->Disable();
  FlexGridSizer5->Add(cbKinect_Int, 1, wxALL | wxALIGN_LEFT | wxALIGN_BOTTOM, 5);
  cbKinect_3D = new wxCheckBox(
      pnKinect, ID_CHECKBOX8, _("3D point cloud"), wxDefaultPosition, wxDefaultSize, 0,
      wxDefaultValidator, _T("ID_CHECKBOX8"));
  cbKinect_3D->SetValue(false);
  FlexGridSizer5->Add(cbKinect_3D, 1, wxALL | wxALIGN_LEFT | wxALIGN_BOTTOM, 5);
  cbKinect_Depth = new wxCheckBox(
      pnKinect, ID_CHECKBOX9, _("Depth image"), wxDefaultPosition, wxDefaultSize, 0,
      wxDefaultValidator, _T("ID_CHECKBOX9"));
  cbKinect_Depth->SetValue(false);
  FlexGridSizer5->Add(cbKinect_Depth, 1, wxALL | wxALIGN_LEFT | wxALIGN_BOTTOM, 5);
  StaticBoxSizer2->Add(FlexGridSizer5, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_BOTTOM, 0);
  FlexGridSizer4->Add(StaticBoxSizer2, 1, wxEXPAND | wxALIGN_LEFT | wxALIGN_BOTTOM, 5);
  wxString __wxRadioBoxChoices_3[2] = {_("RGB camera"), _("IR camera")};
  rbKinect_int = new wxRadioBox(
      pnKinect, ID_RADIOBOX3, _("Intensity channel:"), wxDefaultPosition, wxDefaultSize, 2,
      __wxRadioBoxChoices_3, 1, 0, wxDefaultValidator, _T("ID_RADIOBOX3"));
  rbKinect_int->SetSelection(0);
  FlexGridSizer4->Add(rbKinect_int, 1, wxALL | wxALIGN_TOP | wxALIGN_CENTER_HORIZONTAL, 5);
  pnKinect->SetSizer(FlexGridSizer4);
  FlexGridSizer4->Fit(pnKinect);
  FlexGridSizer4->SetSizeHints(pnKinect);
  pagesCameras->AddPage(Panel2, _("Camera (opencv)"), false);
  pagesCameras->AddPage(Panel3, _("Camera (FFmpeg)"), false);
  pagesCameras->AddPage(Panel4, _("Camera (custom)"), false);
  pagesCameras->AddPage(Panel5, _("Video file"), false);
  pagesCameras->AddPage(Panel6, _("Rawlog file"), false);
  pagesCameras->AddPage(Panel1, _("Bumblebee"), false);
  pagesCameras->AddPage(pnKinect, _("Kinect"), false);
  FlexGridSizer1->Add(pagesCameras, 1, wxALL | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
  cbGrayscale = new wxCheckBox(
      this, ID_CHECKBOX2, _("Capture in grayscale"), wxDefaultPosition, wxDefaultSize, 0,
      wxDefaultValidator, _T("ID_CHECKBOX2"));
  cbGrayscale->SetValue(true);
  FlexGridSizer1->Add(cbGrayscale, 1, wxEXPAND, 5);
  SetSizer(FlexGridSizer1);
  FlexGridSizer1->Fit(this);
  FlexGridSizer1->SetSizeHints(this);

  Bind(wxEVT_BUTTON, &CPanelCameraSelection::OnbtnBrowseVideoClick, this, ID_BUTTON7);
  Bind(wxEVT_BUTTON, &CPanelCameraSelection::OnbtnBrowseRawlogClick, this, ID_BUTTON8);
  Bind(wxEVT_BUTTON, &CPanelCameraSelection::OnbtnBrowseRawlogDirClick, this, ID_BUTTON9);
  //*)

  // end of automatically-generated code above:
  cbOpencvResolution->Clear();
  cbOpencvResolution->SetSelection(cbOpencvResolution->Append(_("default")));

  cbOpencvResolution->Append(_("320x240"));
  cbOpencvResolution->Append(_("640x480"));
  cbOpencvResolution->Append(_("800x600"));
  cbOpencvResolution->Append(_("1024x768"));
  cbOpencvResolution->Append(_("1280x1024"));
}

void CPanelCameraSelection::OnbtnBrowseVideoClick(wxCommandEvent& event)
{
  wxFileDialog dialog(
      this, wxT("Choose a video to open"), wxT("."), wxT(""),
      wxT("Video files (*.avi;*.mpg;*.mov)|*.avi;*.mpg;*.mov|All files "
          "(*.*)|*.*"),
      wxFD_OPEN | wxFD_FILE_MUST_EXIST);

  if (dialog.ShowModal() == wxID_OK) edVideoFile->SetValue(dialog.GetPath());
}

void CPanelCameraSelection::OnbtnBrowseRawlogClick(wxCommandEvent& event)
{
  wxFileDialog dialog(
      this, wxT("Choose a rawlog to open"), wxT("."), wxT(""),
      wxT("Rawlog files (*.rawlog;*.rawlog.gz)|*.rawlog;*.rawlog.gz|All "
          "files (*.*)|*.*"),
      wxFD_OPEN | wxFD_FILE_MUST_EXIST);

  if (dialog.ShowModal() == wxID_OK)
  {
    edRawlogFile->SetValue(dialog.GetPath());

    if (!mrpt::system::directoryExists(string(edRawlogImgDir->GetValue().mb_str())))
    {
      string fil = string(dialog.GetPath().mb_str());
      string fil_path = mrpt::system::extractFileDirectory(fil);
      fil_path += "/Images";
      edRawlogImgDir->SetValue(fil_path.c_str());
    }
  }
}

void CPanelCameraSelection::OnbtnBrowseRawlogDirClick(wxCommandEvent& event)
{
  wxDirDialog dialog(
      this, wxT("Choose the rawlog directory with external images"), edRawlogImgDir->GetValue(),
      wxDD_DEFAULT_STYLE);

  if (dialog.ShowModal() == wxID_OK) edRawlogImgDir->SetValue(dialog.GetPath());
}

CPanelCameraSelection::~CPanelCameraSelection()
{
  //(*Destroy(CPanelCameraSelection)
  //*)
}

/* ------------------------------------------------------------------------
            writeConfigFromVideoSourcePanel
   ------------------------------------------------------------------------ */
void CPanelCameraSelection::writeConfigFromVideoSourcePanel(
    const std::string& sect, mrpt::config::CConfigFileBase* cfg) const
{
  MRPT_START

  switch (this->pagesCameras->GetSelection())
  {
    // OpenCV:
    // -----------------------------------
    case 0:
    {
      cfg->write(sect, "grabber_type", "opencv");
      cfg->write(sect, "cv_camera_index", format("%i", this->opencvCamIndex->GetValue()));
      cfg->write(
          sect, "cv_camera_type", string(this->cbOpencvCamType->GetStringSelection().mb_str()));

      const std::string sRes = std::string(cbOpencvResolution->GetStringSelection().mb_str());

      if (!sRes.empty())
      {
        const size_t p = sRes.find("x");
        if (p != std::string::npos)
        {
          const std::string sW = sRes.substr(0, p);
          const std::string sH = sRes.substr(p + 1);

          cfg->write(sect, "cv_frame_width", sW);
          cfg->write(sect, "cv_frame_height", sH);
        }
      }
    }
    break;

    // Camera FFmpeg
    // -----------------------------------
    case 1:
    {
      cfg->write(sect, "grabber_type", "ffmpeg");
      cfg->write(sect, "ffmpeg_url", string(this->edIPcamURL->GetValue().mb_str()));
    }
    break;

    // Camera custom
    // -----------------------------------
    case 2:
    {
      // Replicate the config sections in "edCustomCamConfig" into "cfg":
      mrpt::config::CConfigFileMemory cfgIn(string(edCustomCamConfig->GetValue().mb_str()));
      std::vector<std::string> allSects;
      cfgIn.getAllSections(allSects);
      for (const auto& allSect : allSects)
      {
        std::vector<std::string> keys;
        cfgIn.getAllKeys(allSect, keys);
        for (const auto& key : keys) cfg->write(allSect, key, cfgIn.read_string(allSect, key, ""));
      }
    }
    break;

    // Video file
    // -----------------------------------
    case 3:
    {
      cfg->write(sect, "grabber_type", "ffmpeg");
      cfg->write(sect, "ffmpeg_url", string(this->edVideoFile->GetValue().mb_str()));
    }
    break;

    // Rawlog
    // -----------------------------------
    case 4:
    {
      cfg->write(sect, "grabber_type", "rawlog");
      cfg->write(sect, "rawlog_file", string(this->edRawlogFile->GetValue().mb_str()));

      const string rawlog_lb = string(this->edRawlogLabel->GetValue().mb_str());
      if (!rawlog_lb.empty()) cfg->write(sect, "rawlog_camera_sensor_label", rawlog_lb);

      mrpt::img::CImage::setImagesPathBase(string(this->edRawlogImgDir->GetValue().mb_str()));
    }
    break;

    // Bumblebee
    // -----------------------------------
    case 5:
    {
      cfg->write(sect, "grabber_type", "bumblebee");
      if (this->rbBumblebeeSel->GetSelection() < 2)
        cfg->write(
            sect, "bumblebee_mono", format("%i\n", (int)this->rbBumblebeeSel->GetSelection()));
      else
      { /* Use stereo capture */
      }
      cfg->write(sect, "bumblebee_fps", 15);
      cfg->write(sect, "bumblebee_get_rectified", this->cbBumblebeeRectif->GetValue());
    }
    break;

    // Kinect
    // -----------------------------------
    case 6:
    {
      cfg->write(sect, "grabber_type", "kinect");

      cfg->write(sect, "kinect_grab_intensity", cbKinect_Int->GetValue());
      cfg->write(sect, "kinect_grab_3d", cbKinect_3D->GetValue());
      cfg->write(sect, "kinect_grab_range", cbKinect_Depth->GetValue());

      cfg->write(sect, "kinect_video_rgb", rbKinect_int->GetSelection() == 0 ? 1 : 0);
    }
    break;

    default:
    {
      cerr << "[MRPT CPanelCameraSelection] ERROR: Unknown camera "
              "selection tab!\n";
      THROW_EXCEPTION("Unknown camera selection tab!");
    }
  }

  // Add grayscale option:
  cfg->write(sect, "capture_grayscale", this->cbGrayscale->GetValue());

  MRPT_END
}

/* ------------------------------------------------------------------------
            readConfigIntoVideoSourcePanel
   ------------------------------------------------------------------------ */
void CPanelCameraSelection::readConfigIntoVideoSourcePanel(
    const std::string& sect, const mrpt::config::CConfigFileBase* cfg) const
{
  MRPT_START

  const std::string grab_type = cfg->read_string(sect, "grabber_type", "opencv");

  if (grab_type == "opencv")
  {
    this->pagesCameras->SetSelection(0);

    this->opencvCamIndex->SetValue(cfg->read_int(sect, "cv_camera_index", 0));
    this->cbOpencvCamType->SetStringSelection(cfg->read_string(sect, "cv_camera_type", "").c_str());

    const int w = cfg->read_int(sect, "cv_frame_width", 0);

    if (w == 320)
      this->cbOpencvResolution->SetSelection(1);
    else if (w == 640)
      this->cbOpencvResolution->SetSelection(2);
    else
      this->cbOpencvResolution->SetSelection(0);
  }
  else if (grab_type == "ffmpeg")
  {
    // Page: 1 (IP), 3 (file)
    const string url = cfg->read_string(sect, "ffmpeg_url", "rtsp://192.168.0.1/live.sdp");

    if (url.substr(0, 5) == "rtsp:")
    {
      this->pagesCameras->SetSelection(1);
      this->edIPcamURL->SetValue(url.c_str());
    }
    else
    {
      this->pagesCameras->SetSelection(3);
      this->edVideoFile->SetValue(url.c_str());
    }
  }
  else if (grab_type == "rawlog")
  {
    this->pagesCameras->SetSelection(4);

    this->edRawlogFile->SetValue(cfg->read_string(sect, "rawlog_file", "").c_str());

    const string lb = cfg->read_string(sect, "rawlog_camera_sensor_label", "");
    this->edRawlogLabel->SetValue(lb.c_str());
  }
  else if (grab_type == "bumblebee")
  {
    this->pagesCameras->SetSelection(5);

    this->rbBumblebeeSel->SetSelection(cfg->read_int(sect, "bumblebee_mono", 0));
    this->cbBumblebeeRectif->SetValue(cfg->read_bool(sect, "bumblebee_get_rectified", false));
  }
  else
    THROW_EXCEPTION_FMT("Error: Unknown choice in 'grabber_type': '%s'", grab_type.c_str());

  // Grayscale option:
  this->cbGrayscale->SetValue(cfg->read_bool(sect, "capture_grayscale", false));

  MRPT_END
}

mrptKeyModifier mrpt::gui::keyEventToMrptKeyModifier(const wxKeyEvent& ev)
{
  int mod = MRPTKMOD_NONE;
  if (ev.AltDown()) mod |= MRPTKMOD_ALT;
  if (ev.CmdDown()) mod |= MRPTKMOD_CMD;
  if (ev.ControlDown()) mod |= MRPTKMOD_CONTROL;
  if (ev.MetaDown()) mod |= MRPTKMOD_META;
  if (ev.ShiftDown()) mod |= MRPTKMOD_SHIFT;
  return mrptKeyModifier(mod);
}

wxSize mrpt::gui::GetScaledClientSize(const wxWindow* w)
{
  int width, height;
  w->GetClientSize(&width, &height);
  const double s = w->GetContentScaleFactor();
  return wxSize(mrpt::round(s * width), mrpt::round(s * height));
}

#endif  // MRPT_HAS_WXWIDGETS

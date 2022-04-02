/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "CRawlogTreeView.h"

#include <mrpt/gui/wx28-fixes.h>
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/dcbuffer.h>
#include <wx/dcclient.h>
#include <wx/font.h>
#include <wx/image.h>
#include <wx/imaglist.h>
#include <wx/intl.h>
#include <wx/string.h>
#include <wx/window.h>

IMPLEMENT_DYNAMIC_CLASS(CRawlogTreeView, wxScrolledWindow)

BEGIN_EVENT_TABLE(CRawlogTreeView, wxScrolledWindow)
EVT_LEFT_DOWN(CRawlogTreeView::OnLeftDown)
EVT_RIGHT_DOWN(CRawlogTreeView::OnRightDown)
EVT_MOUSEWHEEL(CRawlogTreeView::OnMouseWheel)
EVT_CHAR(CRawlogTreeView::OnKey)
EVT_SCROLLWIN_THUMBTRACK(CRawlogTreeView::onScrollThumbTrack)
EVT_SCROLLWIN_THUMBRELEASE(CRawlogTreeView::onScrollThumbRelease)
END_EVENT_TABLE()

#include <mrpt/system/datetime.h>

std::atomic_bool CRawlogTreeView::RAWLOG_UNDERGOING_CHANGES{false};

#define MRPT_NO_WARN_BIG_HDR  // It's ok here
#include <mrpt/config/CConfigFile.h>
#include <mrpt/gui/WxUtils.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/obs.h>
#include <mrpt/obs/CObservationPointCloud.h>  // this one is in mrpt-maps
#include <mrpt/serialization/CArchive.h>

#include <regex>

extern std::unique_ptr<mrpt::config::CConfigFile> iniFile;
extern std::string iniFileSect;

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::serialization;
using namespace mrpt::obs;
using namespace mrpt::rtti;
using namespace std;

const int CRawlogTreeView::ROW_HEIGHT = 17;
const int CRawlogTreeView::TREE_HORZ_STEPS = 25;

const long ID_MNU_EXPORT_ANOTHER_FILE = 1001;
const long ID_MNU_EXPORT_LAST_FILE = 1002;

static std::string shortenClassName(const std::string& s)
{
	auto ret = std::regex_replace(
		s, std::regex("mrpt\\:\\:obs\\:\\:CObservation"), "");
	ret = std::regex_replace(ret, std::regex("mrpt\\:\\:obs\\:\\:"), "");
	return ret;
}

/* ------------------------------------------------------------
						CRawlogTreeView
   ------------------------------------------------------------ */
CRawlogTreeView::CRawlogTreeView(
	wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size,
	long style, const wxString& name)
	: wxScrolledWindow(
		  parent, id, pos, size, style | wxVSCROLL | wxFULL_REPAINT_ON_RESIZE,
		  name),
	  m_rawlog_start(INVALID_TIMESTAMP),
	  m_rawlog_last(INVALID_TIMESTAMP),
	  m_tree_nodes()
{
	m_contextMenu.Append(
		ID_MNU_EXPORT_ANOTHER_FILE, "Export to another .rawlog file...");

	m_contextMenu.Append(
		ID_MNU_EXPORT_LAST_FILE, "Append to same previous file");

	Bind(
		wxEVT_MENU, &CRawlogTreeView::onMnuExportToOtherFile, this,
		ID_MNU_EXPORT_ANOTHER_FILE);

	Bind(
		wxEVT_MENU, &CRawlogTreeView::onMnuAppendSaveFile, this,
		ID_MNU_EXPORT_LAST_FILE);
}

/* ------------------------------------------------------------
						setRawlogSource
   ------------------------------------------------------------ */
void CRawlogTreeView::setRawlogSource(CRawlog* rawlog)
{
	m_rawlog = rawlog;
	reloadFromRawlog();
}

/* ------------------------------------------------------------
						reloadFromRawlog
   ------------------------------------------------------------ */
void CRawlogTreeView::reloadFromRawlog(int hint_rawlog_items)
{
	// Recompute the total height of the scroll area:
	//  We also compute a list for each index with:
	//    - Pointer to data
	//	  - level in the hierarchy (0,1,2)
	// --------------------------------------------------------

	if (m_rawlog)
	{
		if (hint_rawlog_items < 0) m_tree_nodes.reserve(m_rawlog->size() + 100);
		else
			m_tree_nodes.reserve(hint_rawlog_items + 100);
	}

	// Create a "tree node" for each element in the rawlog:
	// ---------------------------------------------------------
	m_tree_nodes.clear();

	m_rawlog_start = INVALID_TIMESTAMP;
	m_rawlog_last = INVALID_TIMESTAMP;

	// Root:
	m_tree_nodes.emplace_back();
	m_tree_nodes.back().level = 0;

	auto lambdaCheckTimestamp = [this](const mrpt::Clock::time_point& t) {
		if (t == INVALID_TIMESTAMP) return;
		m_rawlog_last = t;
		if (m_rawlog_start == INVALID_TIMESTAMP) m_rawlog_start = t;
	};

	if (m_rawlog)
	{
		CRawlog::iterator end_it = m_rawlog->end();
		size_t rawlog_index = 0;
		for (const auto& entry : *m_rawlog)
		{
			m_tree_nodes.emplace_back();
			TNodeData& dEntry = m_tree_nodes.back();
			dEntry.level = 1;
			dEntry.data = entry;
			dEntry.index = rawlog_index;

			// For containers, go recursively:
			if (auto sf = std::dynamic_pointer_cast<CSensoryFrame>(entry); sf)
			{
				for (auto& o : *sf)
				{
					m_tree_nodes.emplace_back();
					TNodeData& dSF = m_tree_nodes.back();
					dSF.level = 2;
					dSF.data = o;
					lambdaCheckTimestamp(o->timestamp);
				}
			}
			else if (auto acts =
						 std::dynamic_pointer_cast<CActionCollection>(entry);
					 acts)
			{
				for (auto& a : *acts)
				{
					m_tree_nodes.emplace_back();
					TNodeData& dAC = m_tree_nodes.back();
					dAC.level = 2;
					dAC.data = a.get_ptr();

					lambdaCheckTimestamp(a->timestamp);
				}
			}
			else if (auto o = std::dynamic_pointer_cast<CObservation>(entry); o)
			{
				lambdaCheckTimestamp(o->timestamp);
			}

			rawlog_index++;
		}
	}

	// Set new size:
	int ly = m_tree_nodes.size();
	SetScrollbars(ROW_HEIGHT, ROW_HEIGHT, 50, ly);
}

/* ------------------------------------------------------------
						reloadFromRawlog
   ------------------------------------------------------------ */
void CRawlogTreeView::OnDraw(wxDC& dc)
{
	if (RAWLOG_UNDERGOING_CHANGES) return;
	try
	{
		OnDrawImpl(dc);
	}
	catch (...)
	{
	}
}

void CRawlogTreeView::OnDrawImpl(wxDC& dc)
{
	// The origin of the window:
	int xc0, y0;
	GetViewStart(&xc0, &y0);  // Not pixels, but **lines**

	int w, h;
	GetSize(&w, &h);

	wxRect visibleArea(
		xc0 * ROW_HEIGHT, y0 * ROW_HEIGHT, w * ROW_HEIGHT, h * ROW_HEIGHT);

	dc.SetPen(*wxTRANSPARENT_PEN);
	dc.SetBrush(*wxWHITE_BRUSH);
	dc.DrawRectangle(xc0 * ROW_HEIGHT, y0 * ROW_HEIGHT, w, h);

	// The first time only, create fonts:
	static wxFont font_normal(
		11, wxFONTFAMILY_ROMAN, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
	static wxFont font_bold(
		11, wxFONTFAMILY_ROMAN, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD);

	static wxColor wxColorGray(40, 40, 40);
	static wxColor brush_SELECTED_COLOR(0, 200, 200);
	static wxPen gray_thick(brush_SELECTED_COLOR, 3);
	static wxPen gray_thin(brush_SELECTED_COLOR, 1);

	size_t first_item = y0;

	size_t n_visible_items = (h / ROW_HEIGHT) + 1;
	if (n_visible_items < 1) n_visible_items = 1;

	size_t last_item = first_item + n_visible_items - 1;

	last_item = min(last_item, m_tree_nodes.size());

	// Draw icons?
	// ----------------------------
	int x0 = 30;
	int Ax0_img = 0;  // Increment in x for the icons, if any.

	if (m_imageList && m_imageList->GetImageCount() > 0)
	{
		int imgw, imgh;
		m_imageList->GetSize(0, imgw, imgh);
		Ax0_img = imgw + 4;
	}

	// Draw only those elements that are visible:
	// -----------------------------------------------
	const int first_tim_y = ROW_HEIGHT + 1;
	const int last_tim_y = (m_tree_nodes.size() - 1) * ROW_HEIGHT - 1;

	for (size_t i = first_item; i < last_item; i++)
	{
		int y = i * ROW_HEIGHT;	 // y= bottom of that row
		TNodeData& d = m_tree_nodes[i];

		switch (d.level)
		{
			case 0:
				dc.SetFont(font_bold);
				dc.SetTextForeground(*wxBLACK);
				break;

			case 1:
				dc.SetFont(font_normal);
				dc.SetTextForeground(*wxBLACK);
				break;

			default:
			case 2:
				dc.SetFont(font_normal);
				dc.SetTextForeground(wxColorGray);
				break;
		};

		// Select icon & text according to type of object
		// ------------------------------------------------
		int icon = -1;
		wxString s;

		if (i == 0)
		{
			// The root node:
			s = (format("Rawlog: %s", m_rawlog_name.c_str()).c_str());
			icon = 3;
		}
		else
		{
			// According to class ID:
			m_rawlog->getAsGeneric(d.index);  // Just to assure it's on memory

			if (d.data)
			{
				// Icon:
				icon = iconIndexFromClass(d.data->GetRuntimeClass());

				// Text:
				if (d.level == 1) s << "[" << std::to_string(d.index) << "] ";

				s << shortenClassName(d.data->GetRuntimeClass()->className);

				// Sensor label:
				if (d.data->GetRuntimeClass()->derivedFrom(
						CLASS_ID(CObservation)))
				{
					CObservation::Ptr obs =
						std::dynamic_pointer_cast<CObservation>(d.data);

					/*if (first_tim==INVALID_TIMESTAMP)
					{
						first_tim = obs->timestamp;
						first_tim_y = y+1;
					}

					last_tim = obs->timestamp;
					last_tim_y = y+1;*/

					if (!obs->sensorLabel.empty())
						s << wxT(" : ") << obs->sensorLabel.c_str();
				}
			}
		}

		// Draw text and icon:
		// ------------------------------------
		if (m_selectedItem == int(i))
		{
			dc.SetBrush(brush_SELECTED_COLOR);
			dc.DrawRectangle(
				x0 + TREE_HORZ_STEPS * d.level, y, w - x0, ROW_HEIGHT);
			dc.SetTextBackground(brush_SELECTED_COLOR);

			dc.SetPen(gray_thick);
			dc.DrawLine(x0, y + 1, x0 + TREE_HORZ_STEPS * d.level, y + 1);
		}
		else
		{
			dc.SetTextBackground(GetBackgroundColour());

			dc.SetPen(gray_thin);
			dc.DrawLine(x0, y + 1, x0 + TREE_HORZ_STEPS * d.level, y + 1);
		}

		dc.DrawText(s, Ax0_img + x0 + TREE_HORZ_STEPS * d.level, y);
		if (m_imageList && icon >= 0)
			m_imageList->Draw(icon, dc, x0 + TREE_HORZ_STEPS * d.level + 1, y);
	}

	// Draw time-line:
	// -----------------------------------------------
	dc.SetPen(*wxLIGHT_GREY_PEN);
	dc.DrawLine(15, visibleArea.GetTop(), 15, visibleArea.GetBottom());

	dc.SetTextForeground(wxColorGray);
	dc.SetFont(font_normal);
	dc.DrawRotatedText(_("Time"), 17, 10, -90);

	// timestamps in time-line:
	// -----------------------------------------------
	if (m_rawlog_start == INVALID_TIMESTAMP ||
		m_rawlog_last == INVALID_TIMESTAMP || last_tim_y <= first_tim_y)
		return;

	dc.SetFont(font_normal);

	const double len_tim =
		mrpt::system::timeDifference(m_rawlog_start, m_rawlog_last);

	std::optional<TTimeStamp> firstTim;

	for (size_t i = first_item; i < last_item; i++)
	{
		const int y = i * ROW_HEIGHT;  // y= bottom of that row
		TNodeData& d = m_tree_nodes[i];

		if (d.data)
		{
			TTimeStamp t_this = INVALID_TIMESTAMP;
			if (d.data->GetRuntimeClass()->derivedFrom(CLASS_ID(CObservation)))
			{
				CObservation::Ptr obs =
					std::dynamic_pointer_cast<CObservation>(d.data);
				t_this = obs->timestamp;
			}

			if (t_this == INVALID_TIMESTAMP) continue;

			if (!firstTim) firstTim = t_this;

			// Draw line:
			if (m_selectedItem == int(i)) dc.SetPen(gray_thick);
			else
				dc.SetPen(gray_thin);

			// Calc. the "y" coordinate for this time:
			const double At =
				mrpt::system::timeDifference(m_rawlog_start, t_this);
			double rat = At / len_tim;
			int ty = first_tim_y + (last_tim_y - first_tim_y) * rat;

			dc.DrawLine(15, ty, x0, y + 1);

			// Draw some text labels with times:
			if ((i % 5) == 0)
			{
				dc.DrawLine(10, ty, 15, ty);

				// Seconds from start of rawlog:
				if (m_rawlog_start != INVALID_TIMESTAMP)
				{
					wxString s = (format("%.03fs", At).c_str());
					dc.DrawRotatedText(s, 17, ty + 3, -90);
				}
			}
		}
	}  // end for i

	// If thumb tracking, show time as text too:
	if (!m_is_thumb_tracking || !firstTim) return;

	using namespace std::string_literals;

	const auto t_this_d = mrpt::Clock::toDouble(*firstTim);
	const auto yb = y0 * ROW_HEIGHT;

	// White background:
	dc.SetPen(*wxBLACK_PEN);
	const auto colorDarkGray = wxColor(30, 30, 30);
	dc.SetBrush(wxBrush(colorDarkGray));
	dc.DrawRoundedRectangle(
		xc0 * ROW_HEIGHT + 5, y0 * ROW_HEIGHT + 5, w - 30, 4 * ROW_HEIGHT, 4);

	dc.SetTextForeground(*wxWHITE);
	dc.SetTextBackground(colorDarkGray);
	dc.SetFont(font_normal);

	dc.DrawText(wxString::Format("t=%.03f", t_this_d), 10, yb + 5);
	dc.DrawText(
		mrpt::system::dateTimeLocalToString(*firstTim) + " (Local)"s, 10,
		yb + 20);
	dc.DrawText(
		mrpt::system::dateTimeToString(*firstTim) + " (UTC)"s, 10, yb + 35);
	dc.DrawText(
		wxString::Format(
			"%.03f / %.03f [s]",
			mrpt::system::timeDifference(m_rawlog_start, *firstTim), len_tim),
		10, yb + 50);
}

// Return an icon index depending on the class of the object in the tree view:
int CRawlogTreeView::iconIndexFromClass(const TRuntimeClassId* class_ID)
{
	int iconIndex = -1;

	if (class_ID == CLASS_ID(CObservation2DRangeScan)) iconIndex = 6;
	else if (class_ID == CLASS_ID(CObservationImage))
		iconIndex = 4;
	else if (class_ID == CLASS_ID(CObservationStereoImages))
		iconIndex = 5;
	else if (class_ID == CLASS_ID(CObservationGPS))
		iconIndex = 7;
	else if (class_ID == CLASS_ID(CObservationGasSensors))
		iconIndex = 8;
	else if (class_ID == CLASS_ID(CObservationWirelessPower))
		iconIndex = 8;
	else if (class_ID == CLASS_ID(CObservationRFID))
		iconIndex = 8;
	else if (class_ID == CLASS_ID(CObservationOdometry))
		iconIndex = 9;
	else if (
		class_ID == CLASS_ID(CObservation3DRangeScan) ||
		class_ID == CLASS_ID(CObservationVelodyneScan) ||
		class_ID == CLASS_ID(CObservationPointCloud))
		iconIndex = 10;
	else if (class_ID == CLASS_ID(CObservationIMU))
		iconIndex = 11;
	else if (class_ID->derivedFrom(CLASS_ID(CObservation)))
		iconIndex = 2;	// Default observation
	else if (class_ID == CLASS_ID(CActionCollection))
		iconIndex = 0;
	else if (class_ID == CLASS_ID(CSensoryFrame))
		iconIndex = 1;
	else if (class_ID->derivedFrom(CLASS_ID(CAction)))
		iconIndex = 2;

	return iconIndex;
}

/* ------------------------------------------------------------
						OnLeftDown
   ------------------------------------------------------------ */
void CRawlogTreeView::OnLeftDown(wxMouseEvent& e)
{
	// The origin of the window:
	int xc0, y0;
	GetViewStart(&xc0, &y0);  // Not pixels, but **lines**

	// Determine the clicked row:
	int nLineThisView = e.GetY() / ROW_HEIGHT;

	size_t sel_item = y0 + nLineThisView;

	SetSelectedItem(sel_item);
}

void CRawlogTreeView::OnRightDown(wxMouseEvent& event)
{
	OnLeftDown(event);

	if (m_selectedItem < 0) return;

	this->PopupMenu(&m_contextMenu);
}

/* ------------------------------------------------------------
						ConnectSelectedItemChange
   ------------------------------------------------------------ */
void CRawlogTreeView::ConnectSelectedItemChange(
	const wxRawlogTreeEventFunction& func)
{
	m_event_select_change = func;
}

/* ------------------------------------------------------------
						SetSelectedItem
   ------------------------------------------------------------ */
void CRawlogTreeView::SetSelectedItem(int sel_item, bool force_refresh)
{
	if (sel_item < (int)m_tree_nodes.size())
	{
		if (sel_item != m_selectedItem || force_refresh)
		{
			m_selectedItem = sel_item;
			Refresh();

			if (m_event_select_change && m_win_parent)
			{
				m_event_select_change(
					m_win_parent, this, evSelected, sel_item,
					sel_item >= 0 ? m_tree_nodes[sel_item].data
								  : CSerializable::Ptr());
			}
		}
	}
}

/* ------------------------------------------------------------
						OnMouseWheel
   ------------------------------------------------------------ */
void CRawlogTreeView::OnMouseWheel(wxMouseEvent& event)
{
	int x, y;
	GetViewStart(&x, &y);

	if (event.GetWheelRotation() > 0) y--;
	else
		y++;

	int ly = m_tree_nodes.size();

	if (y >= 0 && y < ly) SetScrollbars(ROW_HEIGHT, ROW_HEIGHT, 50, ly, x, y);
}

/* ------------------------------------------------------------
						OnKey
   ------------------------------------------------------------ */
void CRawlogTreeView::OnKey(wxKeyEvent& event)
{
	int x, y, y0;
	GetViewStart(&x, &y);

	int w, h;
	GetSize(&w, &h);

	int nLinesPerPage = h / ROW_HEIGHT;

	y0 = y;
	int ly = m_tree_nodes.size();

	if (event.GetKeyCode() == WXK_UP)
	{
		int sel = m_selectedItem - 1;
		if (sel >= 0) SetSelectedItem(sel);
	}
	else if (event.GetKeyCode() == WXK_DOWN)
	{
		int sel = m_selectedItem + 1;
		SetSelectedItem(sel);
	}
	else if (event.GetKeyCode() == WXK_PAGEDOWN)
	{
		SetSelectedItem(m_selectedItem + nLinesPerPage);
		y += nLinesPerPage;
		if (y > ly) y = ly;
	}
	else if (event.GetKeyCode() == WXK_PAGEUP)
	{
		int sel = m_selectedItem - nLinesPerPage;
		if (sel <= 0) sel = 0;

		SetSelectedItem(sel);
		y -= nLinesPerPage;
		if (y < 0) y = 0;
	}
	else if (event.GetKeyCode() == WXK_END)
	{
		SetSelectedItem(ly - 1);
		y = ly - 1;
	}
	else if (event.GetKeyCode() == WXK_HOME)
	{
		SetSelectedItem(0);
		y = 0;
	}
	else
		event.Skip();

	if (y >= 0 && y < ly && y != y0)
		SetScrollbars(ROW_HEIGHT, ROW_HEIGHT, 50, ly, x, y);
}

void CRawlogTreeView::onScrollThumbTrack(wxScrollWinEvent& ev)
{
	m_is_thumb_tracking = true;
	ev.Skip();	// keep processing in base class
}
void CRawlogTreeView::onScrollThumbRelease(wxScrollWinEvent& ev)
{
	m_is_thumb_tracking = false;
	ev.Skip();	// keep processing in base class
}

void CRawlogTreeView::onMnuExportToOtherFile(wxCommandEvent&)
{
	if (m_selectedItem < 0) return;

	const auto& obj = m_tree_nodes[m_selectedItem].data;

	const wxString wildcard =
		"RawLog files (*.rawlog,*.rawlog.gz)|*.rawlog;*.rawlog.gz|All "
		"files (*.*)|*.*";

	const wxString defaultDir =
		iniFile->read_string(iniFileSect, "LastDir", ".");

	wxFileDialog dialog(
		this, "Save file...", defaultDir, {}, wildcard,
		wxFD_SAVE | wxFD_OVERWRITE_PROMPT);

	if (dialog.ShowModal() != wxID_OK) return;

	mrpt::io::CFileGZOutputStream fs(dialog.GetPath().ToStdString());
	auto a = mrpt::serialization::archiveFrom(fs);

	a << obj;

	m_last_exported_rawlog_file = fs.filePathAtUse();
}

void CRawlogTreeView::onMnuAppendSaveFile(wxCommandEvent&)
{
	if (m_selectedItem < 0) return;
	if (m_last_exported_rawlog_file.empty()) return;

	const auto& obj = m_tree_nodes[m_selectedItem].data;

	// open GZ file for append:
	mrpt::io::CFileGZOutputStream fs(
		m_last_exported_rawlog_file, mrpt::io::OpenMode::APPEND);

	auto a = mrpt::serialization::archiveFrom(fs);

	a << obj;
}

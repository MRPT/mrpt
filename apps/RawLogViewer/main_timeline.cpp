/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/containers/find_closest.h>
//#include <mrpt/system/CTimeLogger.h>

#include <wx/tooltip.h>

#include "xRawLogViewerMain.h"

constexpr int TL_BORDER = 2;  // pixels
constexpr int TL_BORDER_BOTTOM = 14;  // pixels
constexpr int TL_X_TICK_COUNT = 7;
constexpr int CURSOR_WIDTH_PIXELS = 2;

constexpr int XTICKS_FONT_SIZE = 20;  // => 6x10 actual char, 9x10 incl spacing
constexpr double XTICKS_FONT_WIDTH = 65.0 / 7.0;

constexpr double TL_CLICK_SENSOR_LABEL_VERTICAL_TOLERANCE = 0.1;
constexpr int TL_SEARCH_CLICK_RANGE = 200;

constexpr double TL_MAXIMUM_TIME_ZOOM = 0.001;	// in [0,1] ratio

void xRawLogViewerFrame::createTimeLineObjects(wxFlexGridSizer* fgzMain)
{
	using This = xRawLogViewerFrame;  // shortcut!

	static const long ID_TIMELINE_SCROLL_BAR = wxNewId();

	wxFlexGridSizer* fgzBottomTimeLine;

	fgzBottomTimeLine = new wxFlexGridSizer(1, 1, 0, 0);
	fgzBottomTimeLine->AddGrowableRow(0);
	fgzBottomTimeLine->AddGrowableCol(0);

	// bottom-right/main panel 3D view:
	wxPanel* pnTimeLine;
	{
		static const long ID_PANEL_TIMELINE = wxNewId();
		static const long ID_TIMELINE_GLCANVAS = wxNewId();

		pnTimeLine = new wxPanel(
			this, ID_PANEL_TIMELINE, wxDefaultPosition, wxDefaultSize,
			wxTAB_TRAVERSAL, _T("ID_PANEL_TIMELINE"));
		auto fgs = new wxFlexGridSizer(2, 1, 0, 0);
		fgs->AddGrowableCol(0);
		fgs->AddGrowableRow(1);

		m_timeline.sbTimeLineRange =
			new wxScrollBar(pnTimeLine, ID_TIMELINE_SCROLL_BAR);

		m_timeline.sbTimeLineRange->Bind(
			wxEVT_SCROLL_CHANGED, &This::OnTimelineZoomScroolBar, this);
		m_timeline.sbTimeLineRange->Bind(
			wxEVT_SCROLL_THUMBTRACK, &This::OnTimelineZoomScroolBar, this);

		// scrollbar properties set in rebuildBottomTimeLine().

		fgs->Add(m_timeline.sbTimeLineRange, 1, wxALL | wxEXPAND, 0 /*border*/);

		m_glTimeLine = new CMyGLCanvas(pnTimeLine, ID_TIMELINE_GLCANVAS);
		pnTimeLine->SetMinSize(wxSize(-1, 125));
		m_glTimeLine->SetMinSize(wxSize(-1, 125));
		fgs->Add(m_glTimeLine, 1, wxALL | wxEXPAND, 2 /*border*/);

		pnTimeLine->SetSizer(fgs);
		fgs->SetSizeHints(pnTimeLine);

		fgzBottomTimeLine->Add(pnTimeLine, 1, wxALL | wxEXPAND, 0);

		// timeline opengl area events:
		m_glTimeLine->Bind(wxEVT_MOTION, &This::OnTimeLineMouseMove, this);
		m_glTimeLine->Bind(wxEVT_LEFT_DOWN, &This::OnTimeLineMouseMove, this);
		m_glTimeLine->Bind(wxEVT_LEFT_UP, &This::OnTimeLineMouseLeftUp, this);
		m_glTimeLine->Bind(
			wxEVT_RIGHT_DOWN, &This::OnTimeLineMouseRightDown, this);
		m_glTimeLine->Bind(wxEVT_RIGHT_UP, &This::OnTimeLineMouseRightUp, this);

		m_glTimeLine->Bind(wxEVT_MOUSEWHEEL, &This::OnTimeLineMouseWheel, this);
	}

	fgzMain->Add(fgzBottomTimeLine, 1, wxALL | wxEXPAND, 0);

	// Set-up bottom timeline view opengl objects:
	{
		mrpt::opengl::COpenGLScene::Ptr& scene =
			m_glTimeLine->getOpenGLSceneRef();
		scene->clear();

		// Enable no-projection mode in this viewport:
		scene->getViewport()->setCustomBackgroundColor({1.0f, 1.0f, 1.0f});

		m_glTimeLine->setUseCameraFromScene(true);

		{
			auto glCam = mrpt::opengl::CCamera::Create();
			glCam->setNoProjection();  // work with pixel coordinates
			scene->insert(glCam);
		}

		m_timeline.borderBox = mrpt::opengl::CBox::Create(
			mrpt::math::TPoint3D(-1.0, 0., 0.),
			mrpt::math::TPoint3D(0.9, 1., 0.), true);
		scene->insert(m_timeline.borderBox);

		m_timeline.xTicks = mrpt::opengl::CSetOfObjects::Create();
		scene->insert(m_timeline.xTicks);

		m_timeline.ySensorLabels = mrpt::opengl::CSetOfObjects::Create();
		scene->insert(m_timeline.ySensorLabels);

		m_timeline.allSensorDots = mrpt::opengl::CPointCloud::Create();
		scene->insert(m_timeline.allSensorDots);

		m_timeline.cursor = mrpt::opengl::CBox::Create();
		scene->insert(m_timeline.cursor);

		m_timeline.horizontalCursor = mrpt::opengl::CBox::Create();
		scene->insert(m_timeline.horizontalCursor);

		m_timeline.visiblePage = mrpt::opengl::CBox::Create();
		scene->insert(m_timeline.visiblePage);
	}

	// Bind(wxEVT_IDLE, &This::OnIdle, this);
	Bind(wxEVT_SIZE, &This::OnSize, this);
	Bind(wxEVT_MAXIMIZE, &This::OnMaximize, this);
}

// Resize and rebuild timeline view objects:
void xRawLogViewerFrame::rebuildBottomTimeLine()
{
#if 0
	static mrpt::system::CTimeLogger profiler;
	mrpt::system::CTimeLoggerEntry tle(profiler, "rebuildBottomTimeLine");
#endif

	auto& tl = m_timeline;
	tl.clearStats();

	const auto clsz = m_glTimeLine->GetClientSize();

	auto px2x = [clsz](int u) { return -1.0 + (2.0 / clsz.GetWidth()) * u; };
	auto px2y = [clsz](int v) { return +1.0 - (2.0 / clsz.GetHeight()) * v; };

	auto px2width = [clsz](int u) { return (2.0 / clsz.GetWidth()) * u; };

	const double widthOf1Px = px2width(1);

	// make room for sensor labels on the left side:
	size_t maxSensorLabelLength = 10;
	for (const auto& e : listOfSensorLabels)
		mrpt::keep_max(maxSensorLabelLength, e.first.size());

	tl.actualLeftBorderPixels =
		mrpt::round(maxSensorLabelLength * XTICKS_FONT_WIDTH + 10);

	const double xLeft = px2x(tl.actualLeftBorderPixels);
	const double xRight = px2x(clsz.GetWidth() - TL_BORDER);
	const double yLowerBorder = px2y(clsz.GetHeight() - TL_BORDER_BOTTOM);
	const double yTopBorder = px2y(TL_BORDER);

	const double xLeft1 = px2x(tl.actualLeftBorderPixels + 1);
	const double xRight1 = px2x(clsz.GetWidth() - TL_BORDER - 1);
	const double yLowerBorder1 = px2y(clsz.GetHeight() - TL_BORDER_BOTTOM - 1);
	const double yTopBorder1 = px2y(TL_BORDER + 1);

	const double yLowerBorder2 = px2y(clsz.GetHeight() - TL_BORDER_BOTTOM - 5);
	const double yTopBorder2 = px2y(TL_BORDER + 5);

	// outer border box:
	tl.borderBox->setBoxCorners(
		mrpt::math::TPoint3D(xLeft, yLowerBorder, 0),
		mrpt::math::TPoint3D(xRight, yTopBorder, 0));

	// find time limits:
	auto& min_t = tl.min_t;
	auto& max_t = tl.max_t;

	for (const auto& e : listOfSensorLabels)
	{
		if (e.second.timOccurs.empty()) continue;

		for (const auto& t : e.second.timOccurs)
		{
			if (min_t == INVALID_TIMESTAMP || t < min_t) { min_t = t; }
			if (max_t == INVALID_TIMESTAMP || t > max_t) { max_t = t; }
		}
	}

	// x ticks:
	tl.xTicks->clear();

	// absolute limits of dataset:
	const double abs_min_t_d = mrpt::Clock::toDouble(min_t);
	const double abs_max_t_d = mrpt::Clock::toDouble(max_t);
	const double absDatasetDuration = std::max(1.0, abs_max_t_d - abs_min_t_d);

	// Zoomed-in time range:
	const double min_t_d =
		abs_min_t_d + m_timeline.scrollBarStartPercent * absDatasetDuration;
	const double max_t_d =
		min_t_d + m_timeline.scrollBarZoomVisiblePercent * absDatasetDuration;

	// Scrollbar properties:
	m_timeline.sbTimeLineRange->SetScrollbar(
		// thumb position:
		mrpt::round(absDatasetDuration * m_timeline.scrollBarStartPercent),
		// thumbSize: visible part (seconds).
		mrpt::round(
			absDatasetDuration * m_timeline.scrollBarZoomVisiblePercent),
		// range:
		mrpt::round(absDatasetDuration),
		// page size
		1);

	for (int i = 0; i < (TL_X_TICK_COUNT - 1); i++)
	{
		// tick label:
		auto glLb = mrpt::opengl::CText::Create();
		glLb->setFont("mono", XTICKS_FONT_SIZE);
		glLb->setString(mrpt::system::formatTimeInterval(
			i * (max_t_d - min_t_d) / (TL_X_TICK_COUNT - 1) + min_t_d -
			abs_min_t_d));
		glLb->setColor_u8(0x00, 0x00, 0x00, 0xff);

		const double ptX =
			xLeft1 + i * (xRight1 - xLeft1) / (TL_X_TICK_COUNT - 1);
		glLb->setLocation(ptX, px2y(clsz.GetHeight() - 1), 0);

		tl.xTicks->insert(glLb);

		// tick line:
		auto glTick = mrpt::opengl::CSimpleLine::Create();
		glTick->setColor_u8(0xa0, 0xa0, 0xa0, 0x80);
		glTick->setLineCoords(	//
			ptX, yLowerBorder1, 0,	//
			ptX, yTopBorder1, 0	 //
		);
		tl.xTicks->insert(glTick);
	}

	// Main per-sensor points:
	tl.allSensorDots->clear();
	tl.allSensorDots->setColor_u8(0x00, 0x00, 0xff, 0xff);
	tl.allSensorDots->setPointSize(1.0f);
	tl.allSensorDots->enableVariablePointSize(false);

	tl.yCoordToSensorLabel.clear();
	tl.ySensorLabels->clear();

	if (!listOfSensorLabels.empty())
	{
		const double dy = (yTopBorder2 - yLowerBorder2) /
			(listOfSensorLabels.size() > 1 ? listOfSensorLabels.size() : 1.0);
		double y0 = yLowerBorder2 + 0.1 * dy;

		for (const auto& e : listOfSensorLabels)
		{
			if (e.second.timOccurs.empty()) continue;

			double lastX = -2;	// actual coords go in [-1,1]
			for (const auto& tim : e.second.timOccurs)
			{
				const double t = mrpt::Clock::toDouble(tim);

				const double dx = (t - min_t_d) / (max_t_d - min_t_d);
				if (dx < 0 || dx > 1) continue;	 // out of zoomed-in area

				const double x = xLeft1 + dx * (xRight1 - xLeft1);

				if (x - lastX < widthOf1Px)
					continue;  // no worth adding so many points

				lastX = x;
				tl.allSensorDots->insertPoint(x, y0, 0);
			}

			// Keep a map between vertical coords and sensor labels:
			tl.yCoordToSensorLabel.insert(y0, e.first);

			// and add its visualization:
			if (!e.first.empty())
			{
				auto glLb = mrpt::opengl::CText::Create();
				glLb->setFont("mono", XTICKS_FONT_SIZE);
				glLb->setString(e.first);
				glLb->setColor_u8(0x00, 0x00, 0x00, 0xff);

#if 0
				// right-aligned text:
				const double ptX = xLeft - 4 * widthOf1Px -
					px2width(XTICKS_FONT_WIDTH * e.first.size() - 1);
#endif
				const double ptX = -1.0 + 4 * widthOf1Px;
				glLb->setLocation(ptX, y0, 0);

				glLb->setName(e.first);

				tl.ySensorLabels->insert(glLb);
			}

			y0 += dy;
		}
	}

	// Build x <-> treeIndex map:
	tl.xs2treeIndices.clear();
	tl.treeIndices2xs.clear();
	{
		double lastX = -2;	// actual coords go in [-1,1]
		for (size_t idx = 0; idx < m_treeView->getTotalTreeNodes(); idx++)
		{
			const auto& tim = m_treeView->treeNodes()[idx].timestamp;
			if (!tim.has_value()) continue;

			const double t = mrpt::Clock::toDouble(*tim);

			const double dx = (t - min_t_d) / (max_t_d - min_t_d);
			if (dx < 0 || dx > 1) continue;	 // out of zoomed-in area

			const double x = xLeft1 + dx * (xRight1 - xLeft1);

			// Insert all indices, without the decimation below:
			tl.treeIndices2xs[idx] = x;

			if (x - lastX < widthOf1Px)
				continue;  // decimation: no worth adding so many points

			lastX = x;

			tl.xs2treeIndices.insert(tl.xs2treeIndices.end(), {x, idx});
		}
	}

	// current time position page:
	tl.visiblePage->setColor_u8(0xff, 0x00, 0x00, 0x20);
	tl.visiblePage->setBoxBorderColor({0xff, 0x00, 0x00, 0x20});

	tl.horizontalCursor->setColor_u8(0xc0, 0x00, 0x00, 0x20);
	tl.horizontalCursor->setBoxBorderColor({0xc0, 0x00, 0x00, 0x20});

	tl.cursor->setColor_u8(0x30, 0x30, 0x30, 0x50);
	tl.cursor->setBoxBorderColor({0x30, 0x30, 0x30, 0x50});

	bottomTimeLineUpdateCursorFromTreeScrollPos();	// this does a Refresh()
}

void xRawLogViewerFrame::bottomTimeLineUpdateCursorFromTreeScrollPos()
{
	if (rawlog.empty())
	{
		m_timeline.visiblePage->setVisibility(false);
		m_timeline.cursor->setVisibility(false);
		m_timeline.horizontalCursor->setVisibility(false);
		return;
	}

	const auto clsz = m_glTimeLine->GetClientSize();

	auto px2x = [clsz](int u) { return -1.0 + (2.0 / clsz.GetWidth()) * u; };
	auto px2y = [clsz](int v) { return +1.0 - (2.0 / clsz.GetHeight()) * v; };

	auto px2width = [clsz](int u) { return (2.0 / clsz.GetWidth()) * u; };

	const double widthOf1Px = px2width(1);

	const double xLeft1 = px2x(m_timeline.actualLeftBorderPixels + 1);
	const double xRight1 = px2x(clsz.GetWidth() - TL_BORDER - 1);
	const double yLowerBorder1 = px2y(clsz.GetHeight() - TL_BORDER_BOTTOM - 1);
	const double yTopBorder1 = px2y(TL_BORDER + 1);

	// percent of view:
	double pc0 = 0, pc1 = 0;
	if (const double nItems = m_treeView->getTotalTreeNodes(); nItems > 2)
	{
		pc0 = m_treeView->m_firstVisibleItem / static_cast<double>(nItems - 1);
		pc1 = m_treeView->m_lastVisibleItem / static_cast<double>(nItems - 1);
	}

	// visible page shaded area:
	if (auto itIdx = m_timeline.treeIndices2xs.lower_bound(
			m_treeView->m_firstVisibleItem);
		itIdx != m_timeline.treeIndices2xs.end())
	{
		double xVisPage0 = itIdx->second;  // the "x"
		double xVisPage1 = xVisPage0 +
			(xRight1 - xLeft1) * (pc1 - pc0) /
				m_timeline.scrollBarZoomVisiblePercent;

		mrpt::keep_max(xVisPage1, xVisPage0 + 2 * widthOf1Px);	// Minimum width

		// borders for zoomed-in views.
		mrpt::keep_max(xVisPage1, xLeft1);
		mrpt::keep_min(xVisPage1, xRight1);

		mrpt::keep_max(xVisPage0, xLeft1);
		mrpt::keep_min(xVisPage0, xRight1);

		m_timeline.visiblePage->setVisibility(true);
		m_timeline.visiblePage->setBoxCorners(
			mrpt::math::TPoint3D(xVisPage0, yLowerBorder1, 0),	//
			mrpt::math::TPoint3D(xVisPage1, yTopBorder1, 0));
	}
	else
	{
		m_timeline.visiblePage->setVisibility(false);
	}

	// selected cursor line:
	if (auto itIdxCursor =
			m_timeline.treeIndices2xs.find(m_treeView->GetSelectedItem());
		itIdxCursor != m_timeline.treeIndices2xs.end())
	{
		const double cursorWidth = px2width(CURSOR_WIDTH_PIXELS);

		double xCursor0 = itIdxCursor->second;	// the "x"
		double xCursor1 = xCursor0 + cursorWidth;

		m_timeline.cursor->setVisibility(true);
		m_timeline.cursor->setBoxCorners(
			mrpt::math::TPoint3D(xCursor0, yLowerBorder1, 0),  //
			mrpt::math::TPoint3D(xCursor1, yTopBorder1, 0));
	}
	else
	{
		m_timeline.cursor->setVisibility(false);
	}

	m_glTimeLine->Refresh();
	// DONT: wxTheApp->Yield();
}

void xRawLogViewerFrame::OnTimeLineDoScrollToMouseX(
	const std::optional<std::pair<double, size_t>>& selPt)
{
	if (selPt.has_value())
	{
		auto treeIndex = selPt->second;

		m_treeView->m_is_thumb_tracking = true;
		m_treeView->ScrollToPercent(
			treeIndex /
			static_cast<double>(m_treeView->getTotalTreeNodes() - 1));
	}
}

void xRawLogViewerFrame::OnTimeLineMouseMove(wxMouseEvent& e)
{
	if (e.RightIsDown())
	{
		std::optional<std::pair<double, size_t>> selPt =
			timeLineMouseXYToTreeIndex(e);
		OnTimeLineDoScrollToMouseX(selPt);
	}
	else if (e.LeftIsDown())
	{
		std::optional<std::pair<double, size_t>> selPt;

		if (!m_timeline.currentTrackedSensorLabel.has_value())
		{
			// select on first click:
			std::string sel;
			selPt = timeLineMouseXYToTreeIndex(e, true, std::nullopt, sel);
			if (!sel.empty()) m_timeline.currentTrackedSensorLabel = sel;
		}
		else
		{
			// continue tracking the same sensor:
			selPt = timeLineMouseXYToTreeIndex(
				e, true, *m_timeline.currentTrackedSensorLabel);
		}

		OnTimeLineMouseLeftDown(selPt);
	}
	else
	{
		static double lastUpdate = .0;
		const double t = mrpt::Clock::nowDouble();
		if (t - lastUpdate > 0.05)
		{
			lastUpdate = t;
			std::string sel;
			timeLineMouseXYToTreeIndex(e, true, std::nullopt, sel);
			bottomTimeLineUpdateCursorFromTreeScrollPos();	// Refresh()
		}
	}
}
void xRawLogViewerFrame::OnTimeLineMouseLeftDown(
	const std::optional<std::pair<double, size_t>>& selPt)
{
	if (selPt.has_value())
	{
		auto treeIndex = selPt->second;

		m_treeView->SetSelectedItem(treeIndex);
		if (!m_treeView->isItemIndexVisible(treeIndex))
		{
			m_treeView->ScrollToPercent(
				treeIndex /
				static_cast<double>(m_treeView->getTotalTreeNodes() - 1));
		}
	}
}
void xRawLogViewerFrame::OnTimeLineMouseLeftUp(wxMouseEvent&)
{
	m_timeline.currentTrackedSensorLabel.reset();

	m_treeView->m_is_thumb_tracking = false;
	m_treeView->Refresh();
}
void xRawLogViewerFrame::OnTimeLineMouseRightDown(wxMouseEvent& e)
{
	OnTimeLineMouseMove(e);
}
void xRawLogViewerFrame::OnTimeLineMouseRightUp(wxMouseEvent& e)
{
	OnTimeLineMouseLeftUp(e);
}

void xRawLogViewerFrame::OnTimeLineMouseWheel(wxMouseEvent& e)
{
	const auto wheelIncr = e.GetWheelRotation();

	if (!e.ShiftDown())
	{
		// Zoom in/out:
		const double incrRatio = 1.0 -
			0.2 * (std::max(1.1, std::abs(wheelIncr) / 100.0) - 1.0) *
				mrpt::sign(wheelIncr);

		const double R = m_timeline.scrollBarZoomVisiblePercent;

		m_timeline.scrollBarZoomVisiblePercent *= incrRatio;
		mrpt::keep_min(m_timeline.scrollBarZoomVisiblePercent, 1.0);
		mrpt::keep_max(
			m_timeline.scrollBarZoomVisiblePercent, TL_MAXIMUM_TIME_ZOOM);

		const double Rnew = m_timeline.scrollBarZoomVisiblePercent;

		// Try to keep the hover-on point in the same position before vs after
		// zoom:
		if (std::optional<std::pair<double, size_t>> selPt =
				timeLineMouseXYToTreeIndex(e, false);
			selPt.has_value())
		{
			double xFocus = selPt->first;
			mrpt::keep_max(xFocus, -1.0);
			mrpt::keep_min(xFocus, 1.0);

			const auto clsz = m_glTimeLine->GetClientSize();

			auto px2x = [clsz](int u) {
				return -1.0 + (2.0 / clsz.GetWidth()) * u;
			};
			const double xLeft1 = px2x(m_timeline.actualLeftBorderPixels + 1);
			const double xRight1 = px2x(clsz.GetWidth() - TL_BORDER - 1);

			const double c = (xFocus - xLeft1) / (xRight1 - xLeft1);

			// -------------------------------------------------------
			// Equation for keeping the same focused point:
			// s'+c*R' = s+c*R
			// -------------------------------------------------------
			const double s = m_timeline.scrollBarStartPercent;

			const double sNew = s + c * (R - Rnew);

			m_timeline.scrollBarStartPercent = sNew;
		}
	}
	else
	{
		// Pan left/right:
		const double incrRatio = -0.2 *
			(std::max(1.1, std::abs(wheelIncr) / 100.0) - 1.0) *
			mrpt::sign(wheelIncr) * m_timeline.scrollBarZoomVisiblePercent;

		m_timeline.scrollBarStartPercent += incrRatio;
	}

	// limits:
	mrpt::keep_max(m_timeline.scrollBarStartPercent, 0.0);
	mrpt::keep_min(
		m_timeline.scrollBarStartPercent,
		1.0 - m_timeline.scrollBarZoomVisiblePercent);

	// Update
	rebuildBottomTimeLine();
}

void xRawLogViewerFrame::OnTimelineZoomScroolBar(const wxScrollEvent&)
{
	int pos = m_timeline.sbTimeLineRange->GetThumbPosition();
	int maxRange = m_timeline.sbTimeLineRange->GetRange();
	m_timeline.scrollBarStartPercent = static_cast<double>(pos) / maxRange;

	rebuildBottomTimeLine();
}

std::optional<std::pair<double, size_t>>
	xRawLogViewerFrame::timeLineMouseXYToTreeIndex(
		const wxMouseEvent& e, bool refineBySensorLabelVerticalMatch,
		const std::optional<std::string>& forceThisSensorLabel,
		const mrpt::optional_ref<std::string>& outSelectedSensorLabel) const
{
	const auto clsz = m_glTimeLine->GetClientSize();

	auto& tl = m_timeline;

	const int mouseX = e.GetX();
	if (mouseX < 0 || mouseX >= clsz.GetWidth()) return {};

	const int mouseY = e.GetY();
	if (mouseY < 0 || mouseY >= clsz.GetHeight()) return {};

	auto px2x = [clsz](int u) { return -1.0 + (2.0 / clsz.GetWidth()) * u; };
	auto px2y = [clsz](int v) { return +1.0 - (2.0 / clsz.GetHeight()) * v; };

	double clickedX = px2x(mouseX);
	double clickedY = px2y(mouseY);

	// find an approximate sensor type by vertical position:
	std::optional<std::string> trackedSensorLabel;

	if (forceThisSensorLabel.has_value())
	{  //
		trackedSensorLabel = *forceThisSensorLabel;
	}
	else
	{
		auto closestSensorLabel = mrpt::containers::find_closest_with_tolerance(
			tl.yCoordToSensorLabel.getDirectMap(), clickedY,
			TL_CLICK_SENSOR_LABEL_VERTICAL_TOLERANCE);

		if (closestSensorLabel.has_value())
		{
			trackedSensorLabel = closestSensorLabel->second;

			if (outSelectedSensorLabel)
				outSelectedSensorLabel.value().get() = *trackedSensorLabel;
		}
	}

	// vertical line highlight:
	for (size_t i = 0;; i++)
	{
		auto glLb = tl.ySensorLabels->getByClass<mrpt::opengl::CText>(i);
		if (!glLb) break;

		if (trackedSensorLabel.has_value() &&
			glLb->getString() == *trackedSensorLabel)
			glLb->setColor_u8(0xff, 0x00, 0x00, 0xff);
		else
			glLb->setColor_u8(0x00, 0x00, 0x00, 0xff);
	}

	m_timeline.horizontalCursor->setVisibility(false);
	if (trackedSensorLabel.has_value())
	{
		const double xLeft1 = px2x(tl.actualLeftBorderPixels + 1);
		const double xRight1 = px2x(clsz.GetWidth() - TL_BORDER - 1);

		auto px2height = [clsz](int v) { return (2.0 / clsz.GetHeight()) * v; };

		const double heightOf1Px = px2height(1);

		const auto& im = tl.yCoordToSensorLabel.getInverseMap();
		if (auto it = im.find(*trackedSensorLabel); it != im.end())
		{
			const double y = it->second;
			const double ym1 = y - heightOf1Px;
			const double yp1 = y + heightOf1Px;

			m_timeline.horizontalCursor->setVisibility(true);
			m_timeline.horizontalCursor->setBoxCorners(
				mrpt::math::TPoint3D(xLeft1, ym1, 0),
				mrpt::math::TPoint3D(xRight1, yp1, 0));
		}
	}

	// ---
	std::string matchedClickSensorLabel;

	if (auto closestXIdx =
			mrpt::containers::find_closest(m_timeline.xs2treeIndices, clickedX);
		closestXIdx.has_value() &&
		m_timeline.treeIndices2xs.count(closestXIdx->second) != 0)
	{
		// Closest sensor by "x":
		auto treeIndex = closestXIdx->second;

		mrpt::keep_max(treeIndex, 0U);
		mrpt::keep_min(treeIndex, m_treeView->getTotalTreeNodes() - 1);

		// Look in the neighbors for the exact sensorLabel:
		if (trackedSensorLabel.has_value() && refineBySensorLabelVerticalMatch)
		{
			const std::string& clickedSensorLabel = *trackedSensorLabel;

			const int lastIdx = m_treeView->getTotalTreeNodes() - 1;

			const auto lambdaTest = [&treeIndex, this, clickedSensorLabel,
									 lastIdx, &matchedClickSensorLabel](int i) {
				if (i < 0) return false;
				if (i > lastIdx) return false;

				const auto& sl = m_treeView->treeNodes().at(i).sensorLabel;

				if (!sl.has_value()) return false;

				bool found = sl.value() == clickedSensorLabel;

				if (found && m_timeline.treeIndices2xs.count(i) != 0)
				{
					treeIndex = i;	// keep best match.
					matchedClickSensorLabel = clickedSensorLabel;
				}

				return found;
			};

			const int ti0 = static_cast<int>(treeIndex);

			for (int dist = 0; dist < TL_SEARCH_CLICK_RANGE; dist++)
			{
				if (bool found = lambdaTest(ti0 + dist); found) break;
				if (bool found = lambdaTest(ti0 - dist); found) break;
			}
		}

		if (!matchedClickSensorLabel.empty())
		{
			m_glTimeLine->SetToolTip(matchedClickSensorLabel);
			m_glTimeLine->GetToolTip()->SetReshow(10);
			m_glTimeLine->GetToolTip()->SetDelay(10);
		}
		else
		{
			m_glTimeLine->UnsetToolTip();
		}

		// Search "x" in the treeIndices2xs again for the case where treeIndex
		// has been modified by the search-by-sensorLabel loop above
		return {{m_timeline.treeIndices2xs.at(treeIndex), treeIndex}};
	}
	else
	{
		return {};
	}
}

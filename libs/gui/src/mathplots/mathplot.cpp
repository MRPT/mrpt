/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
/////////////////////////////////////////////////////////////////////////////
// Name:            mathplot.cpp
// Purpose:         Framework for plotting in wxWindows
// Original Author: David Schalig
// Maintainer:      Davide Rondini
// Contributors:    Jose Luis Blanco, Val Greene
// Created:         21/07/2003
// Last edit:       09/09/2007
// Copyright:       (c) David Schalig, Davide Rondini
// Licence:         wxWindows licence
/////////////////////////////////////////////////////////////////////////////

#if defined(__GNUG__) && !defined(__clang__)
#pragma implementation "mathplot.h"
#endif

// For compilers that support precompilation, includes "wx.h".
#include <wx/window.h>

// Comment out for release operation:
// (Added by J.L.Blanco, Aug 2007)
// #define MATHPLOT_DO_LOGGING

const int INVALID_CLICK_COORDS = -99999;

#ifdef __BORLANDC__
#pragma hdrstop
#endif

#ifndef WX_PRECOMP
#include "wx/object.h"
#include "wx/font.h"
#include "wx/colour.h"
#include "wx/settings.h"
#include "wx/sizer.h"
#include "wx/log.h"
#include "wx/intl.h"
#include "wx/dcclient.h"
#include "wx/cursor.h"
#endif

#include <mrpt/otherlibs/mathplot/mathplot.h>

#include <wx/bmpbuttn.h>
#include <wx/module.h>
#include <wx/msgdlg.h>
#include <wx/image.h>
#include <wx/tipwin.h>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES  // (For VS to define M_PI, etc. in cmath)
#endif
#include <cmath>
#include <cstdio>  // used only for debug
#include <ctime>  // used for representation of x axes involving date
#include <algorithm>  // For std::min()/max()

// #include "pixel.xpm"

// Memory leak debugging
//#ifdef _DEBUG
//#define new DEBUG_NEW
//#endif

// Legend margins
#define mpLEGEND_MARGIN 5
#define mpLEGEND_LINEWIDTH 10

// Minimum axis label separation
#define mpMIN_X_AXIS_LABEL_SEPARATION 64
#define mpMIN_Y_AXIS_LABEL_SEPARATION 32

// Number of pixels to scroll when scrolling by a line
#define mpSCROLL_NUM_PIXELS_PER_LINE 10

// See doxygen comments.
double mpWindow::zoomIncrementalFactor = 1.5;

//-----------------------------------------------------------------------------
// mpLayer
//-----------------------------------------------------------------------------

IMPLEMENT_ABSTRACT_CLASS(mpLayer, wxObject)

mpLayer::mpLayer()
{
	SetPen((wxPen&)*wxBLACK_PEN);
	SetFont((wxFont&)*wxNORMAL_FONT);
	m_continuous = FALSE;  // Default
	m_showName = TRUE;  // Default
	m_drawOutsideMargins = TRUE;
	m_visible = true;
}

wxBitmap mpLayer::GetColourSquare(int side)
{
	wxBitmap square(side, side, -1);
	wxColour filler = m_pen.GetColour();
	wxBrush brush(filler, wxSOLID);
	wxMemoryDC dc;
	dc.SelectObject(square);
	dc.SetBackground(brush);
	dc.Clear();
	dc.SelectObject(wxNullBitmap);
	return square;
}

//-----------------------------------------------------------------------------
// mpInfoLayer
//-----------------------------------------------------------------------------
IMPLEMENT_DYNAMIC_CLASS(mpInfoLayer, mpLayer)

mpInfoLayer::mpInfoLayer()
{
	m_dim = wxRect(0, 0, 1, 1);
	m_brush = *wxTRANSPARENT_BRUSH;
	m_reference.x = 0;
	m_reference.y = 0;
	m_winX = 1;  // parent->GetScrX();
	m_winY = 1;  // parent->GetScrY();
	m_type = mpLAYER_INFO;
}

mpInfoLayer::mpInfoLayer(wxRect rect, const wxBrush* brush) : m_dim(rect)
{
	m_brush = *brush;
	m_reference.x = rect.x;
	m_reference.y = rect.y;
	m_winX = 1;  // parent->GetScrX();
	m_winY = 1;  // parent->GetScrY();
	m_type = mpLAYER_INFO;
}

mpInfoLayer::~mpInfoLayer() = default;
void mpInfoLayer::UpdateInfo(mpWindow& w, wxEvent& event) {}
bool mpInfoLayer::Inside(wxPoint& point) { return m_dim.Contains(point); }
void mpInfoLayer::Move(wxPoint delta)
{
	m_dim.SetX(m_reference.x + delta.x);
	m_dim.SetY(m_reference.y + delta.y);
}

void mpInfoLayer::UpdateReference()
{
	m_reference.x = m_dim.x;
	m_reference.y = m_dim.y;
}

void mpInfoLayer::Plot(wxDC& dc, mpWindow& w)
{
	if (m_visible)
	{
		// Adjust relative position inside the window
		int scrx = w.GetScrX();
		int scry = w.GetScrY();
		// Avoid dividing by 0
		if (scrx == 0) scrx = 1;
		if (scry == 0) scry = 1;

		if ((m_winX != scrx) || (m_winY != scry))
		{
#ifdef MATHPLOT_DO_LOGGING
// wxLogMessage(_("mpInfoLayer::Plot() screen size has changed from %d x %d to
// %d x %d"), m_winX, m_winY, scrx, scry);
#endif
			if (m_winX != 1)
				m_dim.x = (int)floor((double)(m_dim.x * scrx / m_winX));
			if (m_winY != 1)
			{
				m_dim.y = (int)floor((double)(m_dim.y * scry / m_winY));
				UpdateReference();
			}
			// Finally update window size
			m_winX = scrx;
			m_winY = scry;
		}
		dc.SetPen(m_pen);
		//     wxImage image0(wxT("pixel.png"), wxBITMAP_TYPE_PNG);
		//     wxBitmap image1(image0);
		//     wxBrush semiWhite(image1);
		dc.SetBrush(m_brush);
		dc.DrawRectangle(m_dim.x, m_dim.y, m_dim.width, m_dim.height);
	}
}

wxPoint mpInfoLayer::GetPosition() const { return m_dim.GetPosition(); }
wxSize mpInfoLayer::GetSize() { return m_dim.GetSize(); }
mpInfoCoords::mpInfoCoords() : mpInfoLayer() {}
mpInfoCoords::mpInfoCoords(wxRect rect, const wxBrush* brush)
	: mpInfoLayer(rect, brush)
{
}

mpInfoCoords::~mpInfoCoords() = default;
void mpInfoCoords::UpdateInfo(mpWindow& w, wxEvent& event)
{
	if (event.GetEventType() == wxEVT_MOTION)
	{
		int mouseX = ((wxMouseEvent&)event).GetX();
		int mouseY = ((wxMouseEvent&)event).GetY();
/* It seems that Windows port of wxWidgets don't support multi-line test to be
   drawn in a wxDC.
   wxGTK instead works perfectly with it.
   Info on wxForum:
   http://wxforum.shadonet.com/viewtopic.php?t=3451&highlight=drawtext+eol */
#ifdef _WINDOWS
		m_content.Printf(wxT("x = %f y = %f"), w.p2x(mouseX), w.p2y(mouseY));
#else
		m_content.Printf(wxT("x = %f\ny = %f"), w.p2x(mouseX), w.p2y(mouseY));
#endif
	}
}

void mpInfoCoords::Plot(wxDC& dc, mpWindow& w)
{
	if (m_visible)
	{
		// Adjust relative position inside the window
		int scrx = w.GetScrX();
		int scry = w.GetScrY();
		if ((m_winX != scrx) || (m_winY != scry))
		{
#ifdef MATHPLOT_DO_LOGGING
// wxLogMessage(_("mpInfoLayer::Plot() screen size has changed from %d x %d to
// %d x %d"), m_winX, m_winY, scrx, scry);
#endif
			if (m_winX != 1)
				m_dim.x = (int)floor((double)(m_dim.x * scrx / m_winX));
			if (m_winY != 1)
			{
				m_dim.y = (int)floor((double)(m_dim.y * scry / m_winY));
				UpdateReference();
			}
			// Finally update window size
			m_winX = scrx;
			m_winY = scry;
		}
		dc.SetPen(m_pen);
		//     wxImage image0(wxT("pixel.png"), wxBITMAP_TYPE_PNG);
		//     wxBitmap image1(image0);
		//     wxBrush semiWhite(image1);
		dc.SetBrush(m_brush);
		dc.SetFont(m_font);
		int textX, textY;
		dc.GetTextExtent(m_content, &textX, &textY);
		if (m_dim.width < textX + 10) m_dim.width = textX + 10;
		if (m_dim.height < textY + 10) m_dim.height = textY + 10;
		dc.DrawRectangle(m_dim.x, m_dim.y, m_dim.width, m_dim.height);
		dc.DrawText(m_content, m_dim.x + 5, m_dim.y + 5);
	}
}

mpInfoLegend::mpInfoLegend() : mpInfoLayer() {}
mpInfoLegend::mpInfoLegend(wxRect rect, const wxBrush* brush)
	: mpInfoLayer(rect, brush)
{
}

mpInfoLegend::~mpInfoLegend() = default;
void mpInfoLegend::UpdateInfo(mpWindow& w, wxEvent& event) {}
void mpInfoLegend::Plot(wxDC& dc, mpWindow& w)
{
	if (m_visible)
	{
		// Adjust relative position inside the window
		int scrx = w.GetScrX();
		int scry = w.GetScrY();
		if ((m_winX != scrx) || (m_winY != scry))
		{
#ifdef MATHPLOT_DO_LOGGING
// wxLogMessage(_("mpInfoLayer::Plot() screen size has changed from %d x %d to
// %d x %d"), m_winX, m_winY, scrx, scry);
#endif
			if (m_winX != 1)
				m_dim.x = (int)floor((double)(m_dim.x * scrx / m_winX));
			if (m_winY != 1)
			{
				m_dim.y = (int)floor((double)(m_dim.y * scry / m_winY));
				UpdateReference();
			}
			// Finally update window size
			m_winX = scrx;
			m_winY = scry;
		}
		//     wxImage image0(wxT("pixel.png"), wxBITMAP_TYPE_PNG);
		//     wxBitmap image1(image0);
		//     wxBrush semiWhite(image1);
		dc.SetBrush(m_brush);
		dc.SetFont(m_font);
		const int baseWidth = (mpLEGEND_MARGIN * 2 + mpLEGEND_LINEWIDTH);
		int textX = baseWidth, textY = mpLEGEND_MARGIN;
		int plotCount = 0;
		int posY = 0;
		int tmpX = 0, tmpY = 0;
		mpLayer* ly = nullptr;
		wxPen lpen;
		wxString label;
		for (unsigned int p = 0; p < w.CountAllLayers(); p++)
		{
			ly = w.GetLayer(p);
			if ((ly->GetLayerType() == mpLAYER_PLOT) && (ly->IsVisible()))
			{
				label = ly->GetName();
				dc.GetTextExtent(label, &tmpX, &tmpY);
				textX = (textX > (tmpX + baseWidth))
							? textX
							: (tmpX + baseWidth + mpLEGEND_MARGIN);
				textY += (tmpY);
#ifdef MATHPLOT_DO_LOGGING
// wxLogMessage(_("mpInfoLegend::Plot() Adding layer %d: %s"), p,
// label.c_str());
#endif
			}
		}
		dc.SetPen(m_pen);
		dc.SetBrush(m_brush);
		m_dim.width = textX;
		if (textY != mpLEGEND_MARGIN)
		{  // Don't draw any thing if there are no visible layers
			textY += mpLEGEND_MARGIN;
			m_dim.height = textY;
			dc.DrawRectangle(m_dim.x, m_dim.y, m_dim.width, m_dim.height);
			for (unsigned int p2 = 0; p2 < w.CountAllLayers(); p2++)
			{
				ly = w.GetLayer(p2);
				if ((ly->GetLayerType() == mpLAYER_PLOT) && (ly->IsVisible()))
				{
					label = ly->GetName();
					lpen = ly->GetPen();
					dc.GetTextExtent(label, &tmpX, &tmpY);
					dc.SetPen(lpen);
					// textX = (textX > (tmpX + baseWidth)) ? textX : (tmpX +
					// baseWidth);
					// textY += (tmpY + mpLEGEND_MARGIN);
					posY = m_dim.y + mpLEGEND_MARGIN + plotCount * tmpY +
						   (tmpY >> 1);
					dc.DrawLine(
						m_dim.x + mpLEGEND_MARGIN,  // X start coord
						posY,  // Y start coord
						m_dim.x + mpLEGEND_LINEWIDTH +
							mpLEGEND_MARGIN,  // X end coord
						posY);
					// dc.DrawRectangle(m_dim.x + 5, m_dim.y + 5 +
					// plotCount*tmpY, 5, 5);
					dc.DrawText(
						label, m_dim.x + baseWidth,
						m_dim.y + mpLEGEND_MARGIN + plotCount * tmpY);
					plotCount++;
				}
			}
		}
	}
}

//-----------------------------------------------------------------------------
// mpLayer implementations - functions
//-----------------------------------------------------------------------------

IMPLEMENT_ABSTRACT_CLASS(mpFX, mpLayer)

mpFX::mpFX(wxString name, int flags)
{
	SetName(name);
	m_flags = flags;
	m_type = mpLAYER_PLOT;
}

void mpFX::Plot(wxDC& dc, mpWindow& w)
{
	if (m_visible)
	{
		dc.SetPen(m_pen);

		wxCoord startPx = m_drawOutsideMargins ? 0 : w.GetMarginLeft();
		wxCoord endPx = m_drawOutsideMargins ? w.GetScrX()
											 : w.GetScrX() - w.GetMarginRight();
		wxCoord minYpx = m_drawOutsideMargins ? 0 : w.GetMarginTop();
		wxCoord maxYpx = m_drawOutsideMargins
							 ? w.GetScrY()
							 : w.GetScrY() - w.GetMarginBottom();

		wxCoord iy = 0;
		if (m_pen.GetWidth() <= 1)
		{
			for (wxCoord i = startPx; i < endPx; ++i)
			{
				iy = w.y2p(GetY(w.p2x(i)));
				// Draw the point only if you can draw outside margins or if the
				// point is inside margins
				if (m_drawOutsideMargins || ((iy >= minYpx) && (iy <= maxYpx)))
					dc.DrawPoint(i, iy);  // (wxCoord) ((w.GetPosY() - GetY(
				// (double)i / w.GetScaleX() +
				// w.GetPosX()) ) * w.GetScaleY()));
			}
		}
		else
		{
			for (wxCoord i = startPx; i < endPx; ++i)
			{
				iy = w.y2p(GetY(w.p2x(i)));
				// Draw the point only if you can draw outside margins or if the
				// point is inside margins
				if (m_drawOutsideMargins || ((iy >= minYpx) && (iy <= maxYpx)))
					dc.DrawLine(i, iy, i, iy);
				//             wxCoord c = w.y2p( GetY(w.p2x(i)) ); //(wxCoord)
				//             ((w.GetPosY() - GetY( (double)i / w.GetScaleX() +
				//             w.GetPosX()) ) * w.GetScaleY());
			}
		}

		if (!m_name.IsEmpty() && m_showName)
		{
			dc.SetFont(m_font);

			wxCoord tx, ty;
			dc.GetTextExtent(m_name, &tx, &ty);

			/*if ((m_flags & mpALIGNMASK) == mpALIGN_RIGHT)
				tx = (w.GetScrX()>>1) - tx - 8;
			else if ((m_flags & mpALIGNMASK) == mpALIGN_CENTER)
				tx = -tx/2;
			else
				tx = -(w.GetScrX()>>1) + 8;
			*/
			if ((m_flags & mpALIGNMASK) == mpALIGN_RIGHT)
				tx = (w.GetScrX() - tx) - w.GetMarginRight() - 8;
			else if ((m_flags & mpALIGNMASK) == mpALIGN_CENTER)
				tx = ((w.GetScrX() - w.GetMarginRight() - w.GetMarginLeft() -
					   tx) /
					  2) +
					 w.GetMarginLeft();
			else
				tx = w.GetMarginLeft() + 8;
			dc.DrawText(
				m_name, tx, w.y2p(GetY(w.p2x(tx))));  // (wxCoord) ((w.GetPosY()
			// - GetY( (double)tx /
			// w.GetScaleX() +
			// w.GetPosX())) *
			// w.GetScaleY()) );
		}
	}
}

IMPLEMENT_ABSTRACT_CLASS(mpFY, mpLayer)

mpFY::mpFY(wxString name, int flags)
{
	SetName(name);
	m_flags = flags;
	m_type = mpLAYER_PLOT;
}

void mpFY::Plot(wxDC& dc, mpWindow& w)
{
	if (m_visible)
	{
		dc.SetPen(m_pen);

		wxCoord i, ix;

		wxCoord startPx = m_drawOutsideMargins ? 0 : w.GetMarginLeft();
		wxCoord endPx = m_drawOutsideMargins ? w.GetScrX()
											 : w.GetScrX() - w.GetMarginRight();
		wxCoord minYpx = m_drawOutsideMargins ? 0 : w.GetMarginTop();
		wxCoord maxYpx = m_drawOutsideMargins
							 ? w.GetScrY()
							 : w.GetScrY() - w.GetMarginBottom();

		if (m_pen.GetWidth() <= 1)
		{
			for (i = minYpx; i < maxYpx; ++i)
			{
				ix = w.x2p(GetX(w.p2y(i)));
				if (m_drawOutsideMargins || ((ix >= startPx) && (ix <= endPx)))
					dc.DrawPoint(ix, i);
			}
		}
		else
		{
			for (i = 0; i < w.GetScrY(); ++i)
			{
				ix = w.x2p(GetX(w.p2y(i)));
				if (m_drawOutsideMargins || ((ix >= startPx) && (ix <= endPx)))
					dc.DrawLine(ix, i, ix, i);
				//             wxCoord c =  w.x2p(GetX(w.p2y(i))); //(wxCoord)
				//             ((GetX( (double)i / w.GetScaleY() + w.GetPosY())
				//             - w.GetPosX()) * w.GetScaleX());
				//             dc.DrawLine(c, i, c, i);
			}
		}

		if (!m_name.IsEmpty() && m_showName)
		{
			dc.SetFont(m_font);

			wxCoord tx, ty;
			dc.GetTextExtent(m_name, &tx, &ty);

			if ((m_flags & mpALIGNMASK) == mpALIGN_TOP)
				ty = w.GetMarginTop() + 8;
			else if ((m_flags & mpALIGNMASK) == mpALIGN_CENTER)
				ty = ((w.GetScrY() - w.GetMarginTop() - w.GetMarginBottom() -
					   ty) /
					  2) +
					 w.GetMarginTop();
			else
				ty = w.GetScrY() - 8 - ty - w.GetMarginBottom();

			dc.DrawText(
				m_name, w.x2p(GetX(w.p2y(ty))),
				ty);  // (wxCoord) ((GetX( (double)i / w.GetScaleY() +
			// w.GetPosY()) - w.GetPosX()) * w.GetScaleX()), -ty);
		}
	}
}

IMPLEMENT_ABSTRACT_CLASS(mpFXY, mpLayer)

mpFXY::mpFXY(wxString name, int flags)
{
	SetName(name);
	m_flags = flags;
	m_type = mpLAYER_PLOT;
}

void mpFXY::UpdateViewBoundary(wxCoord xnew, wxCoord ynew)
{
	// Keep track of how many points have been drawn and the bouding box
	maxDrawX = (xnew > maxDrawX) ? xnew : maxDrawX;
	minDrawX = (xnew < minDrawX) ? xnew : minDrawX;
	maxDrawY = (maxDrawY > ynew) ? maxDrawY : ynew;
	minDrawY = (minDrawY < ynew) ? minDrawY : ynew;
	// drawnPoints++;
}

void mpFXY::Plot(wxDC& dc, mpWindow& w)
{
	if (m_visible)
	{
		dc.SetPen(m_pen);

		double x, y;
		// Do this to reset the counters to evaluate bounding box for label
		// positioning
		Rewind();
		GetNextXY(x, y);
		maxDrawX = x;
		minDrawX = x;
		maxDrawY = y;
		minDrawY = y;
		// drawnPoints = 0;
		Rewind();

		wxCoord startPx = m_drawOutsideMargins ? 0 : w.GetMarginLeft();
		wxCoord endPx = m_drawOutsideMargins ? w.GetScrX()
											 : w.GetScrX() - w.GetMarginRight();
		wxCoord minYpx = m_drawOutsideMargins ? 0 : w.GetMarginTop();
		wxCoord maxYpx = m_drawOutsideMargins
							 ? w.GetScrY()
							 : w.GetScrY() - w.GetMarginBottom();

		wxCoord ix = 0, iy = 0;

		if (!m_continuous)
		{
			// for some reason DrawPoint does not use the current pen,
			// so we use DrawLine for fat pens
			if (m_pen.GetWidth() <= 1)
			{
				while (GetNextXY(x, y))
				{
					ix = w.x2p(x);
					iy = w.y2p(y);
					if (m_drawOutsideMargins ||
						((ix >= startPx) && (ix <= endPx) && (iy >= minYpx) &&
						 (iy <= maxYpx)))
					{
						dc.DrawPoint(ix, iy);
						UpdateViewBoundary(ix, iy);
					};
				}
			}
			else
			{
				while (GetNextXY(x, y))
				{
					ix = w.x2p(x);
					iy = w.y2p(y);
					if (m_drawOutsideMargins ||
						((ix >= startPx) && (ix <= endPx) && (iy >= minYpx) &&
						 (iy <= maxYpx)))
					{
						dc.DrawLine(ix, iy, ix, iy);
						UpdateViewBoundary(ix, iy);
					}
					//                dc.DrawLine(cx, cy, cx, cy);
				}
			}
		}
		else
		{
			// Old code
			wxCoord x0 = 0, c0 = 0;
			bool first = TRUE;
			while (GetNextXY(x, y))
			{
				wxCoord x1 =
					w.x2p(x);  // (wxCoord) ((x - w.GetPosX()) * w.GetScaleX());
				wxCoord c1 =
					w.y2p(y);  // (wxCoord) ((w.GetPosY() - y) * w.GetScaleY());
				if (first)
				{
					first = FALSE;
					x0 = x1;
					c0 = c1;
				}
				bool outUp, outDown;
				if ((x1 >= startPx) && (x0 <= endPx))
				{
					outDown = (c0 > maxYpx) && (c1 > maxYpx);
					outUp = (c0 < minYpx) && (c1 < minYpx);
					if (!outUp && !outDown)
					{
						if (c1 != c0)
						{
							if (c0 < minYpx)
							{
								x0 =
									(int)(((float)(minYpx - c0)) / ((float)(c1 - c0)) * (x1 - x0)) +
									x0;
								c0 = minYpx;
							}
							if (c0 > maxYpx)
							{
								x0 =
									(int)(((float)(maxYpx - c0)) / ((float)(c1 - c0)) * (x1 - x0)) +
									x0;
								// wxLogDebug(wxT("old x0 = %d, new x0 = %d"),
								// x0, newX0);
								// x0 = newX0;
								c0 = maxYpx;
							}
							if (c1 < minYpx)
							{
								x1 =
									(int)(((float)(minYpx - c0)) / ((float)(c1 - c0)) * (x1 - x0)) +
									x0;
								c1 = minYpx;
							}
							if (c1 > maxYpx)
							{
								x1 =
									(int)(((float)(maxYpx - c0)) / ((float)(c1 - c0)) * (x1 - x0)) +
									x0;
								// wxLogDebug(wxT("old x0 = %d, old x1 = %d, new
								// x1 = %d, c0 = %d, c1 = %d, maxYpx = %d"), x0,
								// x1, newX1, c0, c1, maxYpx);
								// x1 = newX1;
								c1 = maxYpx;
							}
						}
						if (x1 != x0)
						{
							if (x0 < startPx)
							{
								c0 =
									(int)(((float)(startPx - x0)) / ((float)(x1 - x0)) * (c1 - c0)) +
									c0;
								x0 = startPx;
							}
							if (x1 > endPx)
							{
								c1 =
									(int)(((float)(endPx - x0)) / ((float)(x1 - x0)) * (c1 - c0)) +
									c0;
								x1 = endPx;
							}
						}
						dc.DrawLine(x0, c0, x1, c1);
						UpdateViewBoundary(x1, c1);
					}
				}
				x0 = x1;
				c0 = c1;
			}
		}

		if (!m_name.IsEmpty() && m_showName)
		{
			dc.SetFont(m_font);

			wxCoord tx, ty;
			dc.GetTextExtent(m_name, &tx, &ty);

			// xxx implement else ... if (!HasBBox())
			{
				// const int sx = w.GetScrX();
				// const int sy = w.GetScrY();

				if ((m_flags & mpALIGNMASK) == mpALIGN_NW)
				{
					tx = minDrawX + 8;
					ty = maxDrawY + 8;
				}
				else if ((m_flags & mpALIGNMASK) == mpALIGN_NE)
				{
					tx = maxDrawX - tx - 8;
					ty = maxDrawY + 8;
				}
				else if ((m_flags & mpALIGNMASK) == mpALIGN_SE)
				{
					tx = maxDrawX - tx - 8;
					ty = minDrawY - ty - 8;
				}
				else
				{  // mpALIGN_SW
					tx = minDrawX + 8;
					ty = minDrawY - ty - 8;
				}
			}

			dc.DrawText(m_name, tx, ty);
		}
	}
}

//-----------------------------------------------------------------------------
// mpProfile implementation
//-----------------------------------------------------------------------------

IMPLEMENT_ABSTRACT_CLASS(mpProfile, mpLayer)

mpProfile::mpProfile(wxString name, int flags)
{
	SetName(name);
	m_flags = flags;
	m_type = mpLAYER_PLOT;
}

void mpProfile::Plot(wxDC& dc, mpWindow& w)
{
	if (m_visible)
	{
		dc.SetPen(m_pen);

		wxCoord startPx = m_drawOutsideMargins ? 0 : w.GetMarginLeft();
		wxCoord endPx = m_drawOutsideMargins ? w.GetScrX()
											 : w.GetScrX() - w.GetMarginRight();
		wxCoord minYpx = m_drawOutsideMargins ? 0 : w.GetMarginTop();
		wxCoord maxYpx = m_drawOutsideMargins
							 ? w.GetScrY()
							 : w.GetScrY() - w.GetMarginBottom();

		// Plot profile linking subsequent point of the profile, instead of
		// mpFY, which plots simple points.
		for (wxCoord i = startPx; i < endPx; ++i)
		{
			wxCoord c0 = w.y2p(GetY(w.p2x(i)));  // (wxCoord) ((w.GetYpos() -
			// GetY( (double)i /
			// w.GetXscl() + w.GetXpos()) )
			// * w.GetYscl());
			wxCoord c1 =
				w.y2p(GetY(w.p2x(i + 1)));  //(wxCoord) ((w.GetYpos() - GetY(
			//(double)(i+1) / w.GetXscl() +
			//(w.GetXpos() ) ) ) * w.GetYscl());
			// c0 = (c0 <= maxYpx) ? ((c0 >= minYpx) ? c0 : minYpx) : maxYpx;
			// c1 = (c1 <= maxYpx) ? ((c1 >= minYpx) ? c1 : minYpx) : maxYpx;
			if (!m_drawOutsideMargins)
			{
				c0 = (c0 <= maxYpx) ? ((c0 >= minYpx) ? c0 : minYpx) : maxYpx;
				c1 = (c1 <= maxYpx) ? ((c1 >= minYpx) ? c1 : minYpx) : maxYpx;
			}
			dc.DrawLine(i, c0, i + 1, c1);
		};
		if (!m_name.IsEmpty())
		{
			dc.SetFont(m_font);

			wxCoord tx, ty;
			dc.GetTextExtent(m_name, &tx, &ty);

			if ((m_flags & mpALIGNMASK) == mpALIGN_RIGHT)
				tx = (w.GetScrX() - tx) - w.GetMarginRight() - 8;
			else if ((m_flags & mpALIGNMASK) == mpALIGN_CENTER)
				tx = ((w.GetScrX() - w.GetMarginRight() - w.GetMarginLeft() -
					   tx) /
					  2) +
					 w.GetMarginLeft();
			else
				tx = w.GetMarginLeft() + 8;

			dc.DrawText(m_name, tx, w.y2p(GetY(w.p2x(tx))));  //(wxCoord)
			//((w.GetPosY() -
			// GetY( (double)tx
			/// w.GetScaleX()
			//+ w.GetPosX()))
			//* w.GetScaleY())
			//);
		}
	}
}

//-----------------------------------------------------------------------------
// mpLayer implementations - furniture (scales, ...)
//-----------------------------------------------------------------------------

#define mpLN10 2.3025850929940456840179914546844

IMPLEMENT_DYNAMIC_CLASS(mpScaleX, mpLayer)

mpScaleX::mpScaleX(wxString name, int flags, bool ticks, unsigned int type)
{
	SetName(name);
	SetFont((wxFont&)*wxSMALL_FONT);
	SetPen((wxPen&)*wxGREY_PEN);
	m_flags = flags;
	m_ticks = ticks;
	m_labelType = type;
	m_type = mpLAYER_AXIS;
	m_labelFormat = wxT("");
}

void mpScaleX::Plot(wxDC& dc, mpWindow& w)
{
	if (m_visible)
	{
		dc.SetPen(m_pen);
		dc.SetFont(m_font);
		int orgy = 0;

		const int extend = w.GetScrX();  //  /2;
		if (m_flags == mpALIGN_CENTER)
			orgy = w.y2p(0);  //(int)(w.GetPosY() * w.GetScaleY());
		if (m_flags == mpALIGN_TOP)
		{
			if (m_drawOutsideMargins)
				orgy = X_BORDER_SEPARATION;
			else
				orgy = w.GetMarginTop();
		}
		if (m_flags == mpALIGN_BOTTOM)
		{
			if (m_drawOutsideMargins)
				orgy = X_BORDER_SEPARATION;
			else
				orgy = w.GetScrY() - w.GetMarginBottom();
		}
		if (m_flags == mpALIGN_BORDER_BOTTOM)
			orgy = w.GetScrY() - 1;  // dc.LogicalToDeviceY(0) - 1;
		if (m_flags == mpALIGN_BORDER_TOP) orgy = 1;  //-dc.LogicalToDeviceY(0);

		dc.DrawLine(0, orgy, w.GetScrX(), orgy);

		// To cut the axis line when draw outside margin is false, use this code
		/*if (m_drawOutsideMargins == true)
			dc.DrawLine( 0, orgy, w.GetScrX(), orgy);
		else
			dc.DrawLine( w.GetMarginLeft(), orgy, w.GetScrX() -
		w.GetMarginRight(), orgy); */

		const double dig = floor(log(128.0 / w.GetScaleX()) / mpLN10);
		const double step = exp(mpLN10 * dig);
		const double end = w.GetPosX() + (double)extend / w.GetScaleX();

		wxCoord tx, ty;
		wxString s;
		wxString fmt;
		int tmp = (int)dig;
		if (m_labelType == mpX_NORMAL)
		{
			if (!m_labelFormat.IsEmpty())
			{
				fmt = m_labelFormat;
			}
			else
			{
				if (tmp >= 1)
				{
					fmt = wxT("%.f");
				}
				else
				{
					tmp = 8 - tmp;
					fmt.Printf(wxT("%%.%df"), tmp >= -1 ? 2 : -tmp);
				}
			}
		}
		else
		{
			// Date and/or time axis representation
			if (m_labelType == mpX_DATETIME)
			{
				fmt = (wxT("%04.0f-%02.0f-%02.0fT%02.0f:%02.0f:%02.0f"));
			}
			else if (m_labelType == mpX_DATE)
			{
				fmt = (wxT("%04.0f-%02.0f-%02.0f"));
			}
			else if ((m_labelType == mpX_TIME) && (end / 60 < 2))
			{
				fmt = (wxT("%02.0f:%02.3f"));
			}
			else
			{
				fmt = (wxT("%02.0f:%02.0f:%02.0f"));
			}
		}

		// double n = floor( (w.GetPosX() - (double)extend / w.GetScaleX()) /
		// step ) * step ;
		double n0 =
			floor(
				(w.GetPosX() /* - (double)(extend - w.GetMarginLeft() - w.GetMarginRight())/ w.GetScaleX() */) /
				step) *
			step;
		double n = 0;
#ifdef MATHPLOT_DO_LOGGING
		wxLogMessage(
			wxT("mpScaleX::Plot: dig: %f , step: %f, end: %f, n: %f"), dig,
			step, end, n0);
#endif
		wxCoord startPx = m_drawOutsideMargins ? 0 : w.GetMarginLeft();
		wxCoord endPx = m_drawOutsideMargins ? w.GetScrX()
											 : w.GetScrX() - w.GetMarginRight();
		wxCoord minYpx = m_drawOutsideMargins ? 0 : w.GetMarginTop();
		wxCoord maxYpx = m_drawOutsideMargins
							 ? w.GetScrY()
							 : w.GetScrY() - w.GetMarginBottom();

		tmp = -65535;
		int labelH = 0;  // Control labels heigth to decide where to put axis
		// name (below labels or on top of axis)
		int maxExtent = 0;
		for (n = n0; n < end; n += step)
		{
			const int p = (int)((n - w.GetPosX()) * w.GetScaleX());
#ifdef MATHPLOT_DO_LOGGING
			wxLogMessage(wxT("mpScaleX::Plot: n: %f -> p = %d"), n, p);
#endif
			if ((p >= startPx) && (p <= endPx))
			{
				if (m_ticks)
				{  // draw axis ticks
					if (m_flags == mpALIGN_BORDER_BOTTOM)
						dc.DrawLine(p, orgy, p, orgy - 4);
					else
						dc.DrawLine(p, orgy, p, orgy + 4);
				}
				else
				{  // draw grid dotted lines
#if wxCHECK_VERSION(3, 0, 0)
					m_pen.SetStyle(wxPENSTYLE_DOT);
#else
					m_pen.SetStyle(wxDOT);
#endif
					dc.SetPen(m_pen);
					if ((m_flags == mpALIGN_BOTTOM) && !m_drawOutsideMargins)
					{
						dc.DrawLine(p, orgy + 4, p, minYpx);
					}
					else
					{
						if ((m_flags == mpALIGN_TOP) && !m_drawOutsideMargins)
						{
							dc.DrawLine(p, orgy - 4, p, maxYpx);
						}
						else
						{
							dc.DrawLine(p, 0 /*-w.GetScrY()*/, p, w.GetScrY());
						}
					}
#if wxCHECK_VERSION(3, 0, 0)
					m_pen.SetStyle(wxPENSTYLE_SOLID);
#else
					m_pen.SetStyle(wxSOLID);
#endif
					dc.SetPen(m_pen);
				}
				// Write ticks labels in s string
				if (m_labelType == mpX_NORMAL)
					s.Printf(fmt, n);
				else if (m_labelType == mpX_DATETIME)
				{
					auto when = (time_t)n;
					struct tm tm = *localtime(&when);
					s.Printf(
						fmt, (double)tm.tm_year + 1900, (double)tm.tm_mon + 1,
						(double)tm.tm_mday, (double)tm.tm_hour,
						(double)tm.tm_min, (double)tm.tm_sec);
				}
				else if (m_labelType == mpX_DATE)
				{
					auto when = (time_t)n;
					struct tm tm = *localtime(&when);
					s.Printf(
						fmt, (double)tm.tm_year + 1900, (double)tm.tm_mon + 1,
						(double)tm.tm_mday);
				}
				else if (
					(m_labelType == mpX_TIME) || (m_labelType == mpX_HOURS))
				{
					double modulus = fabs(n);
					double sign = n / modulus;
					double hh = floor(modulus / 3600);
					double mm = floor((modulus - hh * 3600) / 60);
					double ss = modulus - hh * 3600 - mm * 60;
#ifdef MATHPLOT_DO_LOGGING
					wxLogMessage(
						wxT("%02.0f Hours, %02.0f minutes, %02.0f seconds"),
						sign * hh, mm, ss);
#endif  // MATHPLOT_DO_LOGGING
					if (fmt.Len() == 20)  // Format with hours has 11 chars
						s.Printf(fmt, sign * hh, mm, floor(ss));
					else
						s.Printf(fmt, sign * mm, ss);
				}
				dc.GetTextExtent(s, &tx, &ty);
				labelH = (labelH <= ty) ? ty : labelH;
				/*				if ((p-tx/2-tmp) > 64) { // Problem about
				   non-regular
				   axis labels
									if ((m_flags == mpALIGN_BORDER_BOTTOM) ||
				   (m_flags == mpALIGN_TOP)) {
										dc.DrawText( s, p-tx/2, orgy-4-ty);
									} else {
										dc.DrawText( s, p-tx/2, orgy+4);
									}
									tmp=p+tx/2;
								}
								*/
				maxExtent = (tx > maxExtent)
								? tx
								: maxExtent;  // Keep in mind max label width
			}
		}
		// Actually draw labels, taking care of not overlapping them, and
		// distributing them regularly
		double labelStep = ceil(
							   (maxExtent + mpMIN_X_AXIS_LABEL_SEPARATION) /
							   (w.GetScaleX() * step)) *
						   step;
		for (n = n0; n < end; n += labelStep)
		{
			const int p = (int)((n - w.GetPosX()) * w.GetScaleX());
#ifdef MATHPLOT_DO_LOGGING
			wxLogMessage(
				wxT("mpScaleX::Plot: n_label = %f -> p_label = %d"), n, p);
#endif
			if ((p >= startPx) && (p <= endPx))
			{
				// Write ticks labels in s string
				if (m_labelType == mpX_NORMAL)
					s.Printf(fmt, n);
				else if (m_labelType == mpX_DATETIME)
				{
					auto when = (time_t)n;
					struct tm tm = *localtime(&when);
					s.Printf(
						fmt, (double)tm.tm_year + 1900, (double)tm.tm_mon + 1,
						(double)tm.tm_mday, (double)tm.tm_hour,
						(double)tm.tm_min, (double)tm.tm_sec);
				}
				else if (m_labelType == mpX_DATE)
				{
					auto when = (time_t)n;
					struct tm tm = *localtime(&when);
					s.Printf(
						fmt, (double)tm.tm_year + 1900, (double)tm.tm_mon + 1,
						(double)tm.tm_mday);
				}
				else if (
					(m_labelType == mpX_TIME) || (m_labelType == mpX_HOURS))
				{
					double modulus = fabs(n);
					double sign = n / modulus;
					double hh = floor(modulus / 3600);
					double mm = floor((modulus - hh * 3600) / 60);
					double ss = modulus - hh * 3600 - mm * 60;
#ifdef MATHPLOT_DO_LOGGING
					wxLogMessage(
						wxT("%02.0f Hours, %02.0f minutes, %02.0f seconds"),
						sign * hh, mm, ss);
#endif  // MATHPLOT_DO_LOGGING
					if (fmt.Len() == 20)  // Format with hours has 11 chars
						s.Printf(fmt, sign * hh, mm, floor(ss));
					else
						s.Printf(fmt, sign * mm, ss);
				}
				dc.GetTextExtent(s, &tx, &ty);
				if ((m_flags == mpALIGN_BORDER_BOTTOM) ||
					(m_flags == mpALIGN_TOP))
				{
					dc.DrawText(s, p - tx / 2, orgy - 4 - ty);
				}
				else
				{
					dc.DrawText(s, p - tx / 2, orgy + 4);
				}
			}
		}

		// Draw axis name
		dc.GetTextExtent(m_name, &tx, &ty);
		switch (m_flags)
		{
			case mpALIGN_BORDER_BOTTOM:
				dc.DrawText(m_name, extend - tx - 4, orgy - 8 - ty - labelH);
				break;
			case mpALIGN_BOTTOM:
			{
				if ((!m_drawOutsideMargins) &&
					(w.GetMarginBottom() > (ty + labelH + 8)))
				{
					dc.DrawText(
						m_name, (endPx - startPx - tx) >> 1, orgy + 6 + labelH);
				}
				else
				{
					dc.DrawText(m_name, extend - tx - 4, orgy - 4 - ty);
				}
			}
			break;
			case mpALIGN_CENTER:
				dc.DrawText(m_name, extend - tx - 4, orgy - 4 - ty);
				break;
			case mpALIGN_TOP:
			{
				if ((!m_drawOutsideMargins) &&
					(w.GetMarginTop() > (ty + labelH + 8)))
				{
					dc.DrawText(
						m_name, (endPx - startPx - tx) >> 1,
						orgy - 6 - ty - labelH);
				}
				else
				{
					dc.DrawText(m_name, extend - tx - 4, orgy + 4);
				}
			}
			break;
			case mpALIGN_BORDER_TOP:
				dc.DrawText(m_name, extend - tx - 4, orgy + 6 + labelH);
				break;
			default:
				break;
		}
	}
	/*    if (m_flags != mpALIGN_TOP) {

			if ((m_flags == mpALIGN_BORDER_BOTTOM) || (m_flags == mpALIGN_TOP))
	   {
				dc.DrawText( m_name, extend - tx - 4, orgy - 4 - (ty*2));
			} else {
				dc.DrawText( m_name, extend - tx - 4, orgy - 4 - ty); //orgy + 4
	   + ty);
			}
		}; */
}

IMPLEMENT_DYNAMIC_CLASS(mpScaleY, mpLayer)

mpScaleY::mpScaleY(wxString name, int flags, bool ticks)
{
	SetName(name);
	SetFont((wxFont&)*wxSMALL_FONT);
	SetPen((wxPen&)*wxGREY_PEN);
	m_flags = flags;
	m_ticks = ticks;
	m_type = mpLAYER_AXIS;
	m_labelFormat = wxT("");
}

void mpScaleY::Plot(wxDC& dc, mpWindow& w)
{
	if (m_visible)
	{
		dc.SetPen(m_pen);
		dc.SetFont(m_font);

		int orgx = 0;
		const int extend = w.GetScrY();  // /2;
		if (m_flags == mpALIGN_CENTER)
			orgx = w.x2p(0);  //(int)(w.GetPosX() * w.GetScaleX());
		if (m_flags == mpALIGN_LEFT)
		{
			if (m_drawOutsideMargins)
				orgx = Y_BORDER_SEPARATION;
			else
				orgx = w.GetMarginLeft();
		}
		if (m_flags == mpALIGN_RIGHT)
		{
			if (m_drawOutsideMargins)
				orgx = w.GetScrX() - Y_BORDER_SEPARATION;
			else
				orgx = w.GetScrX() - w.GetMarginRight();
		}
		if (m_flags == mpALIGN_BORDER_RIGHT)
			orgx = w.GetScrX() - 1;  // dc.LogicalToDeviceX(0) - 1;
		if (m_flags == mpALIGN_BORDER_LEFT)
			orgx = 1;  //-dc.LogicalToDeviceX(0);

		// Draw line
		dc.DrawLine(orgx, 0, orgx, extend);

		// To cut the axis line when draw outside margin is false, use this code
		/* if (m_drawOutsideMargins == true)
			dc.DrawLine( orgx, 0, orgx, extend);
		else
			dc.DrawLine( orgx, w.GetMarginTop(), orgx, w.GetScrY() -
		w.GetMarginBottom()); */

		const double dig = floor(log(128.0 / w.GetScaleY()) / mpLN10);
		const double step = exp(mpLN10 * dig);
		const double end = w.GetPosY() + (double)extend / w.GetScaleY();

		wxCoord tx, ty;
		wxString s;
		wxString fmt;
		int tmp = (int)dig;
		double maxScaleAbs = fabs(w.GetDesiredYmax());
		double minScaleAbs = fabs(w.GetDesiredYmin());
		double endscale =
			(maxScaleAbs > minScaleAbs) ? maxScaleAbs : minScaleAbs;
		if (m_labelFormat.IsEmpty())
		{
			if ((endscale < 1e4) && (endscale > 1e-3))
				fmt = wxT("%.2f");
			else
				fmt = wxT("%.1e");
		}
		else
		{
			fmt = m_labelFormat;
		}
		/*    if (tmp>=1)
			{*/
		//    fmt = wxT("%7.5g");
		//     }
		//     else
		//     {
		//         tmp=8-tmp;
		//         fmt.Printf(wxT("%%.%dg"), (tmp >= -1) ? 2 : -tmp);
		//     }

		double n =
			floor(
				(w.GetPosY() -
				 (double)(extend - w.GetMarginTop() - w.GetMarginBottom()) /
					 w.GetScaleY()) /
				step) *
			step;

		/* wxCoord startPx = m_drawOutsideMargins ? 0 : w.GetMarginLeft(); */
		wxCoord endPx = m_drawOutsideMargins ? w.GetScrX()
											 : w.GetScrX() - w.GetMarginRight();
		wxCoord minYpx = m_drawOutsideMargins ? 0 : w.GetMarginTop();
		wxCoord maxYpx = m_drawOutsideMargins
							 ? w.GetScrY()
							 : w.GetScrY() - w.GetMarginBottom();

		tmp = 65536;
		int labelW = 0;
		// Before staring cycle, calculate label height
		int labelHeigth = 0;
		s.Printf(fmt, n);
		dc.GetTextExtent(s, &tx, &labelHeigth);
		for (; n < end; n += step)
		{
			const int p = (int)((w.GetPosY() - n) * w.GetScaleY());
			if ((p >= minYpx) && (p <= maxYpx))
			{
				if (m_ticks)
				{  // Draw axis ticks
					if (m_flags == mpALIGN_BORDER_LEFT)
					{
						dc.DrawLine(orgx, p, orgx + 4, p);
					}
					else
					{
						dc.DrawLine(
							orgx - 4, p, orgx, p);  //( orgx, p, orgx+4, p);
					}
				}
				else
				{
#if wxCHECK_VERSION(3, 0, 0)
					m_pen.SetStyle(wxPENSTYLE_DOT);
#else
					m_pen.SetStyle(wxDOT);
#endif
					dc.SetPen(m_pen);
					if ((m_flags == mpALIGN_LEFT) && !m_drawOutsideMargins)
					{
						dc.DrawLine(orgx - 4, p, endPx, p);
					}
					else
					{
						if ((m_flags == mpALIGN_RIGHT) && !m_drawOutsideMargins)
						{
							dc.DrawLine(minYpx, p, orgx + 4, p);
						}
						else
						{
							dc.DrawLine(0 /*-w.GetScrX()*/, p, w.GetScrX(), p);
						}
					}
#if wxCHECK_VERSION(3, 0, 0)
					m_pen.SetStyle(wxPENSTYLE_SOLID);
#else
					m_pen.SetStyle(wxSOLID);
#endif
					dc.SetPen(m_pen);
				}
				// Print ticks labels
				s.Printf(fmt, n);
				dc.GetTextExtent(s, &tx, &ty);
#ifdef MATHPLOT_DO_LOGGING
				if (ty != labelHeigth)
					wxLogMessage(
						wxT("mpScaleY::Plot: ty(%f) and labelHeigth(%f) "
							"differ!"),
						ty, labelHeigth);
#endif
				labelW = (labelW <= tx) ? tx : labelW;
				if ((tmp - p + labelHeigth / 2) > mpMIN_Y_AXIS_LABEL_SEPARATION)
				{
					if ((m_flags == mpALIGN_BORDER_LEFT) ||
						(m_flags == mpALIGN_RIGHT))
						dc.DrawText(s, orgx + 4, p - ty / 2);
					else
						dc.DrawText(
							s, orgx - 4 - tx,
							p - ty / 2);  //( s, orgx+4, p-ty/2);
					tmp = p - labelHeigth / 2;
				}
			}
		}
		// Draw axis name

		dc.GetTextExtent(m_name, &tx, &ty);
		switch (m_flags)
		{
			case mpALIGN_BORDER_LEFT:
				dc.DrawText(m_name, labelW + 8, 4);
				break;
			case mpALIGN_LEFT:
			{
				if ((!m_drawOutsideMargins) &&
					(w.GetMarginLeft() > (ty + labelW + 8)))
				{
					dc.DrawRotatedText(
						m_name, orgx - 6 - labelW - ty,
						(maxYpx - minYpx + tx) >> 1, 90);
				}
				else
				{
					dc.DrawText(m_name, orgx + 4, 4);
				}
			}
			break;
			case mpALIGN_CENTER:
				dc.DrawText(m_name, orgx + 4, 4);
				break;
			case mpALIGN_RIGHT:
			{
				if ((!m_drawOutsideMargins) &&
					(w.GetMarginRight() > (ty + labelW + 8)))
				{
					dc.DrawRotatedText(
						m_name, orgx + 6 + labelW, (maxYpx - minYpx + tx) >> 1,
						90);
				}
				else
				{
					dc.DrawText(m_name, orgx - tx - 4, 4);
				}
			}
			break;
			case mpALIGN_BORDER_RIGHT:
				dc.DrawText(m_name, orgx - 6 - tx - labelW, 4);
				break;
			default:
				break;
		}
	}

	/*    if (m_flags != mpALIGN_RIGHT) {
		dc.GetTextExtent(m_name, &tx, &ty);
		if (m_flags == mpALIGN_BORDER_LEFT) {
				dc.DrawText( m_name, orgx-tx-4, -extend + ty + 4);
			} else {
				if (m_flags == mpALIGN_BORDER_RIGHT )
					dc.DrawText( m_name, orgx-(tx*2)-4, -extend + ty + 4);
				else
					dc.DrawText( m_name, orgx + 4, -extend + 4);
			}
		}; */
}

//-----------------------------------------------------------------------------
// mpWindow
//-----------------------------------------------------------------------------

IMPLEMENT_DYNAMIC_CLASS(mpWindow, wxWindow)

BEGIN_EVENT_TABLE(mpWindow, wxWindow)
EVT_PAINT(mpWindow::OnPaint)
EVT_SIZE(mpWindow::OnSize)
EVT_SCROLLWIN_THUMBTRACK(mpWindow::OnScrollThumbTrack)
EVT_SCROLLWIN_PAGEUP(mpWindow::OnScrollPageUp)
EVT_SCROLLWIN_PAGEDOWN(mpWindow::OnScrollPageDown)
EVT_SCROLLWIN_LINEUP(mpWindow::OnScrollLineUp)
EVT_SCROLLWIN_LINEDOWN(mpWindow::OnScrollLineDown)
EVT_SCROLLWIN_TOP(mpWindow::OnScrollTop)
EVT_SCROLLWIN_BOTTOM(mpWindow::OnScrollBottom)

EVT_MIDDLE_UP(mpWindow::OnShowPopupMenu)
EVT_RIGHT_DOWN(mpWindow::OnMouseRightDown)  // JLB
EVT_RIGHT_UP(mpWindow::OnShowPopupMenu)
EVT_MOUSEWHEEL(mpWindow::OnMouseWheel)  // JLB
EVT_MOTION(mpWindow::OnMouseMove)  // JLB
EVT_LEFT_DOWN(mpWindow::OnMouseLeftDown)
EVT_LEFT_UP(mpWindow::OnMouseLeftRelease)

EVT_MENU(mpID_CENTER, mpWindow::OnCenter)
EVT_MENU(mpID_FIT, mpWindow::OnFit)
EVT_MENU(mpID_ZOOM_IN, mpWindow::OnZoomIn)
EVT_MENU(mpID_ZOOM_OUT, mpWindow::OnZoomOut)
EVT_MENU(mpID_LOCKASPECT, mpWindow::OnLockAspect)
EVT_MENU(mpID_HELP_MOUSE, mpWindow::OnMouseHelp)
EVT_MENU(mpID_PRINT, mpWindow::OnPrintMenu)
END_EVENT_TABLE()

mpWindow::mpWindow(
	wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size,
	long flag)
	: wxWindow(parent, id, pos, size, flag, wxT("mathplot"))
{
	m_scaleX = m_scaleY = 1.0;
	m_posX = m_posY = 0;
	m_desiredXmin = m_desiredYmin = 0;
	m_desiredXmax = m_desiredYmax = 1;
	m_scrX = m_scrY = 64;  // Fixed from m_scrX = m_scrX = 64;
	m_minX = m_minY = 0;
	m_maxX = m_maxY = 0;
	m_last_lx = m_last_ly = 0;
	m_buff_bmp = nullptr;
	m_enableDoubleBuffer = FALSE;
	m_enableMouseNavigation = TRUE;
	m_mouseMovedAfterRightClick = FALSE;
	m_movingInfoLayer = nullptr;
	// Set margins to 0
	m_marginTop = 0;
	m_marginRight = 0;
	m_marginBottom = 0;
	m_marginLeft = 0;

	m_lockaspect = FALSE;

	m_popmenu.Append(
		mpID_CENTER, _("Center"), _("Center plot view to this position"));
	m_popmenu.Append(mpID_FIT, _("Fit"), _("Set plot view to show all items"));
	m_popmenu.Append(mpID_ZOOM_IN, _("Zoom in"), _("Zoom in plot view."));
	m_popmenu.Append(mpID_ZOOM_OUT, _("Zoom out"), _("Zoom out plot view."));
	m_popmenu.AppendCheckItem(
		mpID_LOCKASPECT, _("Lock aspect"),
		_("Lock horizontal and vertical zoom aspect."));
	m_popmenu.Append(
		mpID_PRINT, _("Print..."), _("Allows printing the graph."));
	m_popmenu.Append(
		mpID_HELP_MOUSE, _("Show mouse commands..."),
		_("Show help about the mouse commands."));

	m_layers.clear();
	SetBackgroundColour(*wxWHITE);
	m_bgColour = *wxWHITE;
	m_fgColour = *wxBLACK;

	m_enableScrollBars = false;
	SetSizeHints(128, 128);

	m_mouseLClick_X = INVALID_CLICK_COORDS;
	m_mouseLClick_Y = INVALID_CLICK_COORDS;

	// J.L.Blanco: Eliminates the "flick" with the double buffer.
	SetBackgroundStyle(wxBG_STYLE_CUSTOM);

	UpdateAll();
}

mpWindow::~mpWindow()
{
	// Free all the layers:
	DelAllLayers(true, false);

	if (m_buff_bmp)
	{
		delete m_buff_bmp;
		m_buff_bmp = nullptr;
	}
}

// Mouse handler, for detecting when the user drag with the right button or just
// "clicks" for the menu
// JLB
void mpWindow::OnMouseRightDown(wxMouseEvent& event)
{
	m_mouseMovedAfterRightClick = FALSE;
	m_mouseRClick_X = event.GetX();
	m_mouseRClick_Y = event.GetY();
	if (m_enableMouseNavigation)
	{
		SetCursor(*wxCROSS_CURSOR);
	}
}

// Process mouse wheel events
// JLB
void mpWindow::OnMouseWheel(wxMouseEvent& event)
{
	if (!m_enableMouseNavigation)
	{
		event.Skip();
		return;
	}

	//     GetClientSize( &m_scrX,&m_scrY);

	if (event.m_controlDown)
	{
		wxPoint clickPt(event.GetX(), event.GetY());
		// CTRL key hold: Zoom in/out:
		if (event.GetWheelRotation() > 0)
			ZoomIn(clickPt);
		else
			ZoomOut(clickPt);
	}
	else
	{
		// Scroll vertically or horizontally (this is SHIFT is hold down).
		int change =
			-event.GetWheelRotation();  // Opposite direction (More intuitive)!
		float changeUnitsX = change / m_scaleX;
		float changeUnitsY = change / m_scaleY;

		if (event.m_shiftDown)
		{
			m_posX += changeUnitsX;
			m_desiredXmax += changeUnitsX;
			m_desiredXmin += changeUnitsX;
		}
		else
		{
			m_posY -= changeUnitsY;
			m_desiredYmax -= changeUnitsY;
			m_desiredYmax -= changeUnitsY;
		}

		UpdateAll();
	}
}

// If the user "drags" with the right buttom pressed, do "pan"
// JLB
void mpWindow::OnMouseMove(wxMouseEvent& event)
{
	if (!m_enableMouseNavigation)
	{
		event.Skip();
		return;
	}

	if (event.m_rightDown)
	{
		m_mouseMovedAfterRightClick =
			TRUE;  // Hides the popup menu after releasing the button!

		// The change:
		int Ax = m_mouseRClick_X - event.GetX();
		int Ay = m_mouseRClick_Y - event.GetY();

		// For the next event, use relative to this coordinates.
		m_mouseRClick_X = event.GetX();
		m_mouseRClick_Y = event.GetY();

		double Ax_units = Ax / m_scaleX;
		double Ay_units = -Ay / m_scaleY;

		m_posX += Ax_units;
		m_posY += Ay_units;
		m_desiredXmax += Ax_units;
		m_desiredXmin += Ax_units;
		m_desiredYmax += Ay_units;
		m_desiredYmin += Ay_units;

		UpdateAll();

#ifdef MATHPLOT_DO_LOGGING
		wxLogMessage(
			_("[mpWindow::OnMouseMove] Ax:%i Ay:%i m_posX:%f m_posY:%f"), Ax,
			Ay, m_posX, m_posY);
#endif
	}
	else
	{
		if (event.m_leftDown)
		{
			if (m_movingInfoLayer == nullptr)
			{
				wxClientDC dc(this);
				wxPen pen(*wxBLACK, 1, wxDOT);
				dc.SetPen(pen);
				dc.SetBrush(*wxTRANSPARENT_BRUSH);
				dc.DrawRectangle(
					m_mouseLClick_X, m_mouseLClick_Y,
					event.GetX() - m_mouseLClick_X,
					event.GetY() - m_mouseLClick_Y);
			}
			else
			{
				wxPoint moveVector(
					event.GetX() - m_mouseLClick_X,
					event.GetY() - m_mouseLClick_Y);
				m_movingInfoLayer->Move(moveVector);
			}
			UpdateAll();
		}
		else
		{
			wxLayerList::iterator li;
			for (li = m_layers.begin(); li != m_layers.end(); ++li)
			{
				if ((*li)->IsInfo() && (*li)->IsVisible())
				{
					auto* tmpLyr = (mpInfoLayer*)(*li);
					tmpLyr->UpdateInfo(*this, event);
					// UpdateAll();
					RefreshRect(tmpLyr->GetRectangle());
				}
			}
			/* if (m_coordTooltip) {
				wxString toolTipContent;
				toolTipContent.Printf(_("X = %f\nY = %f"), p2x(event.GetX()),
			p2y(event.GetY()));
				wxTipWindow** ptr = nullptr;
				wxRect rectBounds(event.GetX(), event.GetY(), 5, 5);
				wxTipWindow* tip = new wxTipWindow(this, toolTipContent, 100,
			ptr, &rectBounds);

			} */
		}
	}
	event.Skip();
}

void mpWindow::OnMouseLeftDown(wxMouseEvent& event)
{
	m_mouseLClick_X = event.GetX();
	m_mouseLClick_Y = event.GetY();
#ifdef MATHPLOT_DO_LOGGING
	wxLogMessage(
		_("mpWindow::OnMouseLeftDown() X = %d , Y = %d"), event.GetX(),
		event.GetY()); /*m_mouseLClick_X, m_mouseLClick_Y);*/
#endif
	wxPoint pointClicked = event.GetPosition();
	m_movingInfoLayer = IsInsideInfoLayer(pointClicked);
	if (m_movingInfoLayer != nullptr)
	{
#ifdef MATHPLOT_DO_LOGGING
		wxLogMessage(
			_("mpWindow::OnMouseLeftDown() started moving layer %lx"),
			(long int)m_movingInfoLayer); /*m_mouseLClick_X, m_mouseLClick_Y);*/
#endif
	}
	event.Skip();
}

void mpWindow::OnMouseLeftRelease(wxMouseEvent& event)
{
	if (m_mouseLClick_X != INVALID_CLICK_COORDS)  // Don't get events that don't
	// started in this window:
	{
		wxPoint release(event.GetX(), event.GetY());
		wxPoint press(m_mouseLClick_X, m_mouseLClick_Y);
		if (m_movingInfoLayer != nullptr)
		{
			m_movingInfoLayer->UpdateReference();
			m_movingInfoLayer = nullptr;
		}
		else
		{
			if (release != press)
			{
				ZoomRect(press, release);
			} /*else {
				if (m_coordTooltip) {
					wxString toolTipContent;
					toolTipContent.Printf(_("X = %f\nY = %f"),
			p2x(event.GetX()), p2y(event.GetY()));
					SetToolTip(toolTipContent);
				}
			} */
		}
		m_mouseLClick_X = INVALID_CLICK_COORDS;
	}
	event.Skip();
}

void mpWindow::Fit()
{
	if (UpdateBBox()) Fit(m_minX, m_maxX, m_minY, m_maxY);
}

// JL
void mpWindow::Fit(
	double xMin, double xMax, double yMin, double yMax, wxCoord* printSizeX,
	wxCoord* printSizeY)
{
	// Save desired borders:
	m_desiredXmin = xMin;
	m_desiredXmax = xMax;
	m_desiredYmin = yMin;
	m_desiredYmax = yMax;

	if (printSizeX != nullptr && printSizeY != nullptr)
	{
		// Printer:
		m_scrX = *printSizeX;
		m_scrY = *printSizeY;
	}
	else
	{
		// Normal case (screen):
		GetClientSize(&m_scrX, &m_scrY);
	}

	double Ax, Ay;

	Ax = xMax - xMin;
	Ay = yMax - yMin;

	m_scaleX = (Ax != 0) ? (m_scrX - m_marginLeft - m_marginRight) / Ax
						 : 1;  // m_scaleX = (Ax!=0) ? m_scrX/Ax : 1;
	m_scaleY = (Ay != 0) ? (m_scrY - m_marginTop - m_marginBottom) / Ay
						 : 1;  // m_scaleY = (Ay!=0) ? m_scrY/Ay : 1;

	if (m_lockaspect)
	{
#ifdef MATHPLOT_DO_LOGGING
		wxLogMessage(
			_("mpWindow::Fit()(lock) m_scaleX=%f,m_scaleY=%f"), m_scaleX,
			m_scaleY);
#endif
		// Keep the lowest "scale" to fit the whole range required by that axis
		// (to actually "fit"!):
		double s = m_scaleX < m_scaleY ? m_scaleX : m_scaleY;
		m_scaleX = s;
		m_scaleY = s;
	}

	// Adjusts corner coordinates: This should be simply:
	//   m_posX = m_minX;
	//   m_posY = m_maxY;
	// But account for centering if we have lock aspect:
	m_posX = (xMin + xMax) / 2 -
			 ((m_scrX - m_marginLeft - m_marginRight) / 2. + m_marginLeft) /
				 m_scaleX;  // m_posX = (xMin+xMax)/2 - (m_scrX/2)/m_scaleX;
	//	m_posY = (yMin+yMax)/2 + ((m_scrY - m_marginTop - m_marginBottom)/2. -
	// m_marginTop)/m_scaleY;  // m_posY = (yMin+yMax)/2 + (m_scrY/2)/m_scaleY;
	m_posY = (yMin + yMax) / 2 +
			 ((m_scrY - m_marginTop - m_marginBottom) / 2. + m_marginTop) /
				 m_scaleY;  // m_posY = (yMin+yMax)/2 + (m_scrY/2)/m_scaleY;

#ifdef MATHPLOT_DO_LOGGING
	wxLogMessage(
		_("mpWindow::Fit() m_desiredXmin=%f m_desiredXmax=%f  m_desiredYmin=%f "
		  "m_desiredYmax=%f"),
		xMin, xMax, yMin, yMax);
	wxLogMessage(
		_("mpWindow::Fit() m_scaleX = %f , m_scrX = %d,m_scrY=%d, Ax=%f, "
		  "Ay=%f, m_posX=%f, m_posY=%f"),
		m_scaleX, m_scrX, m_scrY, Ax, Ay, m_posX, m_posY);
#endif

	// It is VERY IMPORTANT to DO NOT call Refresh if we are drawing to the
	// printer!!
	// Otherwise, the DC dimensions will be those of the window instead of the
	// printer device
	if (printSizeX == nullptr || printSizeY == nullptr) UpdateAll();
}

// Patch ngpaton
void mpWindow::DoZoomInXCalc(const int staticXpixel)
{
	// Preserve the position of the clicked point:
	double staticX = p2x(staticXpixel);
	// Zoom in:
	m_scaleX = m_scaleX * zoomIncrementalFactor;
	// Adjust the new m_posx
	m_posX = staticX - (staticXpixel / m_scaleX);
	// Adjust desired
	m_desiredXmin = m_posX;
	m_desiredXmax =
		m_posX + (m_scrX - (m_marginLeft + m_marginRight)) / m_scaleX;
#ifdef MATHPLOT_DO_LOGGING
	wxLogMessage(
		_("mpWindow::DoZoomInXCalc() prior X coord: (%f), new X coord: (%f) "
		  "SHOULD BE EQUAL!!"),
		staticX, p2x(staticXpixel));
#endif
}

void mpWindow::DoZoomInYCalc(const int staticYpixel)
{
	// Preserve the position of the clicked point:
	double staticY = p2y(staticYpixel);
	// Zoom in:
	m_scaleY = m_scaleY * zoomIncrementalFactor;
	// Adjust the new m_posy:
	m_posY = staticY + (staticYpixel / m_scaleY);
	// Adjust desired
	m_desiredYmax = m_posY;
	m_desiredYmin =
		m_posY - (m_scrY - (m_marginTop + m_marginBottom)) / m_scaleY;
#ifdef MATHPLOT_DO_LOGGING
	wxLogMessage(
		_("mpWindow::DoZoomInYCalc() prior Y coord: (%f), new Y coord: (%f) "
		  "SHOULD BE EQUAL!!"),
		staticY, p2y(staticYpixel));
#endif
}

void mpWindow::DoZoomOutXCalc(const int staticXpixel)
{
	// Preserve the position of the clicked point:
	double staticX = p2x(staticXpixel);
	// Zoom out:
	m_scaleX = m_scaleX / zoomIncrementalFactor;
	// Adjust the new m_posx/y:
	m_posX = staticX - (staticXpixel / m_scaleX);
	// Adjust desired
	m_desiredXmin = m_posX;
	m_desiredXmax =
		m_posX + (m_scrX - (m_marginLeft + m_marginRight)) / m_scaleX;
#ifdef MATHPLOT_DO_LOGGING
	wxLogMessage(
		_("mpWindow::DoZoomOutXCalc() prior X coord: (%f), new X coord: (%f) "
		  "SHOULD BE EQUAL!!"),
		staticX, p2x(staticXpixel));
#endif
}

void mpWindow::DoZoomOutYCalc(const int staticYpixel)
{
	// Preserve the position of the clicked point:
	double staticY = p2y(staticYpixel);
	// Zoom out:
	m_scaleY = m_scaleY / zoomIncrementalFactor;
	// Adjust the new m_posx/y:
	m_posY = staticY + (staticYpixel / m_scaleY);
	// Adjust desired
	m_desiredYmax = m_posY;
	m_desiredYmin =
		m_posY - (m_scrY - (m_marginTop + m_marginBottom)) / m_scaleY;
#ifdef MATHPLOT_DO_LOGGING
	wxLogMessage(
		_("mpWindow::DoZoomOutYCalc() prior Y coord: (%f), new Y coord: (%f) "
		  "SHOULD BE EQUAL!!"),
		staticY, p2y(staticYpixel));
#endif
}

void mpWindow::ZoomIn(const wxPoint& centerPoint)
{
	wxPoint c(centerPoint);
	if (c == wxDefaultPosition)
	{
		GetClientSize(&m_scrX, &m_scrY);
		c.x = (m_scrX - m_marginLeft - m_marginRight) / 2 +
			  m_marginLeft;  // c.x = m_scrX/2;
		c.y = (m_scrY - m_marginTop - m_marginBottom) / 2 -
			  m_marginTop;  // c.y = m_scrY/2;
	}

	// Preserve the position of the clicked point:
	double prior_layer_x = p2x(c.x);
	double prior_layer_y = p2y(c.y);

	// Zoom in:
	m_scaleX = m_scaleX * zoomIncrementalFactor;
	m_scaleY = m_scaleY * zoomIncrementalFactor;

	// Adjust the new m_posx/y:
	m_posX = prior_layer_x - c.x / m_scaleX;
	m_posY = prior_layer_y + c.y / m_scaleY;

	m_desiredXmin = m_posX;
	m_desiredXmax =
		m_posX + (m_scrX - m_marginLeft - m_marginRight) /
					 m_scaleX;  // m_desiredXmax = m_posX + m_scrX / m_scaleX;
	m_desiredYmax = m_posY;
	m_desiredYmin =
		m_posY - (m_scrY - m_marginTop - m_marginBottom) /
					 m_scaleY;  // m_desiredYmin = m_posY - m_scrY / m_scaleY;

#ifdef MATHPLOT_DO_LOGGING
	wxLogMessage(
		_("mpWindow::ZoomIn() prior coords: (%f,%f), new coords: (%f,%f) "
		  "SHOULD BE EQUAL!!"),
		prior_layer_x, prior_layer_y, p2x(c.x), p2y(c.y));
#endif

	UpdateAll();
}

void mpWindow::ZoomOut(const wxPoint& centerPoint)
{
	wxPoint c(centerPoint);
	if (c == wxDefaultPosition)
	{
		GetClientSize(&m_scrX, &m_scrY);
		c.x = (m_scrX - m_marginLeft - m_marginRight) / 2 +
			  m_marginLeft;  // c.x = m_scrX/2;
		c.y = (m_scrY - m_marginTop - m_marginBottom) / 2 -
			  m_marginTop;  // c.y = m_scrY/2;
	}

	// Preserve the position of the clicked point:
	double prior_layer_x = p2x(c.x);
	double prior_layer_y = p2y(c.y);

	// Zoom out:
	m_scaleX = m_scaleX / zoomIncrementalFactor;
	m_scaleY = m_scaleY / zoomIncrementalFactor;

	// Adjust the new m_posx/y:
	m_posX = prior_layer_x - c.x / m_scaleX;
	m_posY = prior_layer_y + c.y / m_scaleY;

	m_desiredXmin = m_posX;
	m_desiredXmax =
		m_posX + (m_scrX - m_marginLeft - m_marginRight) /
					 m_scaleX;  // m_desiredXmax = m_posX + m_scrX / m_scaleX;
	m_desiredYmax = m_posY;
	m_desiredYmin =
		m_posY - (m_scrY - m_marginTop - m_marginBottom) /
					 m_scaleY;  // m_desiredYmin = m_posY - m_scrY / m_scaleY;

#ifdef MATHPLOT_DO_LOGGING
	wxLogMessage(
		_("mpWindow::ZoomOut() prior coords: (%f,%f), new coords: (%f,%f) "
		  "SHOULD BE EQUAL!!"),
		prior_layer_x, prior_layer_y, p2x(c.x), p2y(c.y));
#endif
	UpdateAll();
}

void mpWindow::ZoomInX()
{
	m_scaleX = m_scaleX * zoomIncrementalFactor;
	UpdateAll();
}

void mpWindow::ZoomOutX()
{
	m_scaleX = m_scaleX / zoomIncrementalFactor;
	UpdateAll();
}

void mpWindow::ZoomInY()
{
	m_scaleY = m_scaleY * zoomIncrementalFactor;
	UpdateAll();
}

void mpWindow::ZoomOutY()
{
	m_scaleY = m_scaleY / zoomIncrementalFactor;
	UpdateAll();
}

void mpWindow::ZoomRect(wxPoint p0, wxPoint p1)
{
	// Compute the 2 corners in graph coordinates:
	double p0x = p2x(p0.x);
	double p0y = p2y(p0.y);
	double p1x = p2x(p1.x);
	double p1y = p2y(p1.y);

	// Order them:
	double zoom_x_min = p0x < p1x ? p0x : p1x;
	double zoom_x_max = p0x > p1x ? p0x : p1x;
	double zoom_y_min = p0y < p1y ? p0y : p1y;
	double zoom_y_max = p0y > p1y ? p0y : p1y;

#ifdef MATHPLOT_DO_LOGGING
	wxLogMessage(
		_("Zoom: (%f,%f)-(%f,%f)"), zoom_x_min, zoom_y_min, zoom_x_max,
		zoom_y_max);
#endif

	Fit(zoom_x_min, zoom_x_max, zoom_y_min, zoom_y_max);
}

void mpWindow::LockAspect(bool enable)
{
	m_lockaspect = enable;
	m_popmenu.Check(mpID_LOCKASPECT, enable);

	// Try to fit again with the new config:
	Fit(m_desiredXmin, m_desiredXmax, m_desiredYmin, m_desiredYmax);
}

void mpWindow::OnShowPopupMenu(wxMouseEvent& event)
{
	// Only display menu if the user has not "dragged" the figure
	if (m_enableMouseNavigation)
	{
		SetCursor(*wxSTANDARD_CURSOR);
	}

	if (!m_mouseMovedAfterRightClick)  // JLB
	{
		m_clickedX = event.GetX();
		m_clickedY = event.GetY();
		PopupMenu(&m_popmenu, event.GetX(), event.GetY());
	}
}

void mpWindow::OnLockAspect(wxCommandEvent& WXUNUSED(event))
{
	LockAspect(!m_lockaspect);
}

void mpWindow::OnMouseHelp(wxCommandEvent& WXUNUSED(event))
{
	wxMessageBox(
		_("Supported Mouse commands:\n \
        - Left button down + Mark area: Rectangular zoom\n \
        - Right button down + Move: Pan (Move)\n \
        - Wheel: Vertical scroll\n \
        - Wheel + SHIFT: Horizontal scroll\n \
        - Wheel + CTRL: Zoom in/out"),
		_("wxMathPlot help"), wxOK, this);
}

void mpWindow::OnPrintMenu(wxCommandEvent& WXUNUSED(event))
{
	// Pass two printout objects: for preview, and possible printing.
	auto* plotPrint = new mpPrintout(this);
	auto* plotPrintPreview = new mpPrintout(this);
	auto* preview = new wxPrintPreview(plotPrintPreview, plotPrint);
	wxPreviewFrame* frame = new wxPreviewFrame(
		preview, nullptr, wxT("Print Plot"), wxPoint(100, 100),
		wxSize(600, 650));
	frame->Centre(wxBOTH);
	frame->Initialize();
	frame->Show();
}

void mpWindow::OnFit(wxCommandEvent& WXUNUSED(event)) { Fit(); }
void mpWindow::OnCenter(wxCommandEvent& WXUNUSED(event))
{
	GetClientSize(&m_scrX, &m_scrY);
	int centerX = (m_scrX - m_marginLeft - m_marginRight) /
				  2;  // + m_marginLeft; // c.x = m_scrX/2;
	int centerY = (m_scrY - m_marginTop - m_marginBottom) /
				  2;  // - m_marginTop; // c.y = m_scrY/2;
	SetPos(p2x(m_clickedX - centerX), p2y(m_clickedY - centerY));
	// SetPos( p2x(m_clickedX-m_scrX/2), p2y(m_clickedY-m_scrY/2) );  //SetPos(
	// (double)(m_clickedX-m_scrX/2) / m_scaleX + m_posX,
	// (double)(m_scrY/2-m_clickedY) / m_scaleY + m_posY);
}

void mpWindow::OnZoomIn(wxCommandEvent& WXUNUSED(event))
{
	ZoomIn(wxPoint(m_mouseRClick_X, m_mouseRClick_Y));
}

void mpWindow::OnZoomOut(wxCommandEvent& WXUNUSED(event)) { ZoomOut(); }
void mpWindow::OnSize(wxSizeEvent& WXUNUSED(event))
{
	// Try to fit again with the new window size:
	Fit(m_desiredXmin, m_desiredXmax, m_desiredYmin, m_desiredYmax);
#ifdef MATHPLOT_DO_LOGGING
	wxLogMessage(
		_("mpWindow::OnSize() m_scrX = %d, m_scrY = %d"), m_scrX, m_scrY);
#endif  // MATHPLOT_DO_LOGGING
}

bool mpWindow::AddLayer(mpLayer* layer, bool refreshDisplay)
{
	if (layer != nullptr)
	{
		m_layers.push_back(layer);
		if (refreshDisplay) UpdateAll();
		return true;
	};
	return false;
}

bool mpWindow::DelLayer(
	mpLayer* layer, bool alsoDeleteObject, bool refreshDisplay)
{
	wxLayerList::iterator layIt;
	for (layIt = m_layers.begin(); layIt != m_layers.end(); ++layIt)
	{
		if (*layIt == layer)
		{
			// Also delete the object?
			if (alsoDeleteObject) delete *layIt;
			m_layers.erase(layIt);  // this deleted the reference only
			if (refreshDisplay) UpdateAll();
			return true;
		}
	}
	return false;
}

void mpWindow::DelAllLayers(bool alsoDeleteObject, bool refreshDisplay)
{
	while (m_layers.size() > 0)
	{
		// Also delete the object?
		if (alsoDeleteObject) delete m_layers[0];
		m_layers.erase(m_layers.begin());  // this deleted the reference only
	}
	if (refreshDisplay) UpdateAll();
}

// void mpWindow::DoPrepareDC(wxDC& dc)
// {
//     dc.SetDeviceOrigin(x2p(m_minX), y2p(m_maxY));
// }

void mpWindow::OnPaint(wxPaintEvent& WXUNUSED(event))
{
	wxPaintDC dc(this);
	dc.GetSize(&m_scrX, &m_scrY);  // This is the size of the visible area only!
	//     DoPrepareDC(dc);

#ifdef MATHPLOT_DO_LOGGING
	{
		int px, py;
		GetViewStart(&px, &py);
		wxLogMessage(
			_("[mpWindow::OnPaint] vis.area:%ix%i px=%i py=%i"), m_scrX, m_scrY,
			px, py);
	}
#endif

	// Selects direct or buffered draw:
	wxDC* trgDc;

	// J.L.Blanco @ Aug 2007: Added double buffer support
	if (m_enableDoubleBuffer)
	{
		if (m_last_lx != m_scrX || m_last_ly != m_scrY)
		{
			if (m_buff_bmp) delete m_buff_bmp;
			m_buff_bmp = new wxBitmap(m_scrX, m_scrY);
			m_buff_dc.SelectObject(*m_buff_bmp);
			m_last_lx = m_scrX;
			m_last_ly = m_scrY;
		}
		trgDc = &m_buff_dc;
	}
	else
	{
		trgDc = &dc;
	}

	// Draw background:
	// trgDc->SetDeviceOrigin(0,0);
	trgDc->SetPen(*wxTRANSPARENT_PEN);
	wxBrush brush(GetBackgroundColour());
	trgDc->SetBrush(brush);
	trgDc->SetTextForeground(m_fgColour);
	trgDc->DrawRectangle(0, 0, m_scrX, m_scrY);

	// Draw all the layers:
	// trgDc->SetDeviceOrigin( m_scrX>>1, m_scrY>>1);  // Origin at the center
	wxLayerList::iterator li;
	for (li = m_layers.begin(); li != m_layers.end(); ++li)
	{
		(*li)->Plot(*trgDc, *this);
	};

	// If doublebuffer, draw now to the window:
	if (m_enableDoubleBuffer)
	{
		// trgDc->SetDeviceOrigin(0,0);
		// dc.SetDeviceOrigin(0,0);  // Origin at the center
		dc.Blit(0, 0, m_scrX, m_scrY, trgDc, 0, 0);
	}

	/*    if (m_coordTooltip) {
			wxString toolTipContent;
			wxPoint mousePoint =  wxGetMousePosition();
			toolTipContent.Printf(_("X = %f\nY = %f"), p2x(mousePoint.x),
	   p2y(mousePoint.y));
			SetToolTip(toolTipContent);
		}*/
	// If scrollbars are enabled, refresh them
	if (m_enableScrollBars)
	{
		/*       m_scrollX = (int) floor((m_posX - m_minX)*m_scaleX);
			   m_scrollY = (int) floor((m_maxY - m_posY )*m_scaleY);
			   Scroll(m_scrollX, m_scrollY);*/
		// Scroll(x2p(m_posX), y2p(m_posY));
		//             SetVirtualSize((int) ((m_maxX - m_minX)*m_scaleX), (int)
		//             ((m_maxY - m_minY)*m_scaleY));
		//         int centerX = (m_scrX - m_marginLeft - m_marginRight)/2; // +
		//         m_marginLeft; // c.x = m_scrX/2;
		// 	int centerY = (m_scrY - m_marginTop - m_marginBottom)/2; // -
		// m_marginTop; // c.y = m_scrY/2;
		/*SetScrollbars(1, 1, (int) ((m_maxX - m_minX)*m_scaleX), (int) ((m_maxY
		 * - m_minY)*m_scaleY));*/  //, x2p(m_posX + centerX/m_scaleX), y2p(m_posY - centerY/m_scaleY), true);
	}
}

// void mpWindow::OnScroll2(wxScrollWinEvent &event)
// {
// #ifdef MATHPLOT_DO_LOGGING
//     wxLogMessage(_("[mpWindow::OnScroll2] Init: m_posX=%f m_posY=%f, sc_pos =
//     %d"),m_posX,m_posY, event.GetPosition());
// #endif
//     // If scrollbars are not enabled, Skip operation
//     if (!m_enableScrollBars) {
//         event.Skip();
//         return;
//     }
// //     m_scrollX = (int) floor((m_posX - m_minX)*m_scaleX);
// //     m_scrollY = (int) floor((m_maxY - m_posY /*- m_minY*/)*m_scaleY);
// //     Scroll(m_scrollX, m_scrollY);
//
// //     GetClientSize( &m_scrX, &m_scrY);
//     //Scroll(x2p(m_desiredXmin), y2p(m_desiredYmin));
//     int pixelStep = 1;
//     if (event.GetOrientation() == wxHORIZONTAL) {
//         //m_desiredXmin -= (m_scrollX - event.GetPosition())/m_scaleX;
//         //m_desiredXmax -= (m_scrollX - event.GetPosition())/m_scaleX;
//         m_posX -= (m_scrollX - event.GetPosition())/m_scaleX;
//         m_scrollX = event.GetPosition();
//     }
//     Fit(m_desiredXmin, m_desiredXmax, m_desiredYmin, m_desiredYmax);
// // /*    int pixelStep = 1;
// //     if (event.GetOrientation() == wxHORIZONTAL) {
// //         m_posX 		-= (px -
// event.GetPosition())/m_scaleX;//(pixelStep/m_scaleX);
// // 	m_desiredXmax 	-= (px -
// event.GetPosition())/m_scaleX;//(pixelStep/m_scaleX);
// // 	m_desiredXmin 	-= (px -
// event.GetPosition())/m_scaleX;//(pixelStep/m_scaleX);
// //         //SetPosX( (double)px / GetScaleX() + m_minX +
// (double)(width>>1)/GetScaleX());
// // //         m_posX = p2x(px); //m_minX + (double)(px /*+
// (m_scrX)*/)/GetScaleX();
// //     } else {
// //         m_posY 		+= (py -
// event.GetPosition())/m_scaleY;//(pixelStep/m_scaleY);
// // 	m_desiredYmax	+= (py -
// event.GetPosition())/m_scaleY;//(pixelStep/m_scaleY);
// // 	m_desiredYmax	+= (py -
// event.GetPosition())/m_scaleY;//(pixelStep/m_scaleY);
// //         //SetPosY( m_maxY - (double)py / GetScaleY() -
// (double)(height>>1)/GetScaleY());
// //         //m_posY = m_maxY - (double)py / GetScaleY() -
// (double)(height>>1)/GetScaleY();
// // //         m_posY = p2y(py);//m_maxY - (double)(py /*+
// (m_scrY)*/)/GetScaleY();
// //     }*/
// #ifdef MATHPLOT_DO_LOGGING
//     int px, py;
//     GetViewStart( &px, &py);
//     wxLogMessage(_("[mpWindow::OnScroll2] End:  m_posX = %f, m_posY = %f, px
//     = %f, py = %f"),m_posX, m_posY, px, py);
// #endif
//
//     UpdateAll();
// //     event.Skip();
// }

void mpWindow::SetMPScrollbars(bool status)
{
	// Temporary behaviour: always disable scrollbars
	m_enableScrollBars = status;  // false;
	if (status == false)
	{
		SetScrollbar(wxHORIZONTAL, 0, 0, 0);
		SetScrollbar(wxVERTICAL, 0, 0, 0);
	}
	// else the scroll bars will be updated in UpdateAll();
	UpdateAll();

	//     EnableScrolling(false, false);
	//     m_enableScrollBars = status;
	//     EnableScrolling(status, status);
	/*    m_scrollX = (int) floor((m_posX - m_minX)*m_scaleX);
		m_scrollY = (int) floor((m_posY - m_minY)*m_scaleY);*/
	//     int scrollWidth = (int) floor((m_maxX - m_minX)*m_scaleX) - m_scrX;
	//     int scrollHeight = (int) floor((m_minY - m_maxY)*m_scaleY) - m_scrY;

	// /*    m_scrollX = (int) floor((m_posX - m_minX)*m_scaleX);
	//     m_scrollY = (int) floor((m_maxY - m_posY /*- m_minY*/)*m_scaleY);
	//     int scrollWidth = (int) floor(((m_maxX - m_minX) - (m_desiredXmax -
	//     m_desiredXmin))*m_scaleX);
	//     int scrollHeight = (int) floor(((m_maxY - m_minY) - (m_desiredYmax -
	//     m_desiredYmin))*m_scaleY);
	// #ifdef MATHPLOT_DO_LOGGING
	//     wxLogMessage(_("mpWindow::SetMPScrollbars() scrollWidth = %d,
	//     scrollHeight = %d"), scrollWidth, scrollHeight);
	// #endif
	//     if(status) {
	//         SetScrollbars(1,
	//                       1,
	//                       scrollWidth,
	//                       scrollHeight,
	//                       m_scrollX,
	//                       m_scrollY);
	// //         SetVirtualSize((int) (m_maxX - m_minX), (int) (m_maxY -
	// m_minY));
	//     }
	//     Refresh(false);*/
}

bool mpWindow::UpdateBBox()
{
	bool first = TRUE;

	for (auto f : m_layers)
	{
		if (f->HasBBox())
		{
			if (first)
			{
				first = FALSE;
				m_minX = f->GetMinX();
				m_maxX = f->GetMaxX();
				m_minY = f->GetMinY();
				m_maxY = f->GetMaxY();
			}
			else
			{
				if (f->GetMinX() < m_minX) m_minX = f->GetMinX();
				if (f->GetMaxX() > m_maxX) m_maxX = f->GetMaxX();
				if (f->GetMinY() < m_minY) m_minY = f->GetMinY();
				if (f->GetMaxY() > m_maxY) m_maxY = f->GetMaxY();
			}
		}
		// node = node->GetNext();
	}
#ifdef MATHPLOT_DO_LOGGING
	wxLogDebug(
		wxT("[mpWindow::UpdateBBox] Bounding box: Xmin = %f, Xmax = %f, Ymin = "
			"%f, YMax = %f"),
		m_minX, m_maxX, m_minY, m_maxY);
#endif  // MATHPLOT_DO_LOGGING
	return first == FALSE;
}

// void mpWindow::UpdateAll()
// {
// GetClientSize( &m_scrX,&m_scrY);
/*    if (m_enableScrollBars) {
		// The "virtual size" of the scrolled window:
		const int sx = (int)((m_maxX - m_minX) * GetScaleX());
		const int sy = (int)((m_maxY - m_minY) * GetScaleY());
	SetVirtualSize(sx, sy);
	SetScrollRate(1, 1);*/
//         const int px = (int)((GetPosX() - m_minX) * GetScaleX());// -
//         m_scrX); //(cx>>1));

// J.L.Blanco, Aug 2007: Formula fixed:
//         const int py = (int)((m_maxY - GetPosY()) * GetScaleY());// -
//         m_scrY); //(cy>>1));
//         int px, py;
//         GetViewStart(&px0, &py0);
// 	px = (int)((m_posX - m_minX)*m_scaleX);
// 	py = (int)((m_maxY - m_posY)*m_scaleY);

//         SetScrollbars( 1, 1, sx - m_scrX, sy - m_scrY, px, py, TRUE);
//     }

// Working code
// 	UpdateBBox();
//    Refresh( FALSE );
// end working code

// Old version
/*   bool box = UpdateBBox();
	if (box)
{
		int cx, cy;
		GetClientSize( &cx, &cy);

		// The "virtual size" of the scrolled window:
		const int sx = (int)((m_maxX - m_minX) * GetScaleX());
		const int sy = (int)((m_maxY - m_minY) * GetScaleY());

		const int px = (int)((GetPosX() - m_minX) * GetScaleX() - (cx>>1));

		// J.L.Blanco, Aug 2007: Formula fixed:
		const int py = (int)((m_maxY - GetPosY()) * GetScaleY() - (cy>>1));

		SetScrollbars( 1, 1, sx, sy, px, py, TRUE);

#ifdef MATHPLOT_DO_LOGGING
		wxLogMessage(_("[mpWindow::UpdateAll] Size:%ix%i
ScrollBars:%i,%i"),sx,sy,px,py);
#endif
}

	FitInside();
	Refresh( FALSE );
*/
// }

void mpWindow::UpdateAll()
{
	if (UpdateBBox())
	{
		if (m_enableScrollBars)
		{
			int cx, cy;
			GetClientSize(&cx, &cy);
			// Do x scroll bar
			{
				// Convert margin sizes from pixels to coordinates
				double leftMargin = m_marginLeft / m_scaleX;
				// Calculate the range in coords that we want to scroll over
				double maxX = (m_desiredXmax > m_maxX) ? m_desiredXmax : m_maxX;
				double minX = (m_desiredXmin < m_minX) ? m_desiredXmin : m_minX;
				if ((m_posX + leftMargin) < minX) minX = m_posX + leftMargin;
				// Calculate scroll bar size and thumb position
				int sizeX = (int)((maxX - minX) * m_scaleX);
				int thumbX = (int)(((m_posX + leftMargin) - minX) * m_scaleX);
				SetScrollbar(
					wxHORIZONTAL, thumbX, cx - (m_marginRight + m_marginLeft),
					sizeX);
			}
			// Do y scroll bar
			{
				// Convert margin sizes from pixels to coordinates
				double topMargin = m_marginTop / m_scaleY;
				// Calculate the range in coords that we want to scroll over
				double maxY = (m_desiredYmax > m_maxY) ? m_desiredYmax : m_maxY;
				if ((m_posY - topMargin) > maxY) maxY = m_posY - topMargin;
				double minY = (m_desiredYmin < m_minY) ? m_desiredYmin : m_minY;
				// Calculate scroll bar size and thumb position
				int sizeY = (int)((maxY - minY) * m_scaleY);
				int thumbY = (int)((maxY - (m_posY - topMargin)) * m_scaleY);
				SetScrollbar(
					wxVERTICAL, thumbY, cy - (m_marginTop + m_marginBottom),
					sizeY);
			}
		}
	}

	Refresh(FALSE);
}

void mpWindow::DoScrollCalc(const int position, const int orientation)
{
	if (orientation == wxVERTICAL)
	{
		// Y axis
		// Get top margin in coord units
		double topMargin = m_marginTop / m_scaleY;
		// Calculate maximum Y coord to be shown in the graph
		double maxY = m_desiredYmax > m_maxY ? m_desiredYmax : m_maxY;
		// Set new position
		SetPosY((maxY - (position / m_scaleY)) + topMargin);
	}
	else
	{
		// X Axis
		// Get left margin in coord units
		double leftMargin = m_marginLeft / m_scaleX;
		// Calculate minimum X coord to be shown in the graph
		double minX = (m_desiredXmin < m_minX) ? m_desiredXmin : m_minX;
		// Set new position
		SetPosX((minX + (position / m_scaleX)) - leftMargin);
	}
}

void mpWindow::OnScrollThumbTrack(wxScrollWinEvent& event)
{
	DoScrollCalc(event.GetPosition(), event.GetOrientation());
}

void mpWindow::OnScrollPageUp(wxScrollWinEvent& event)
{
	int scrollOrientation = event.GetOrientation();
	// Get position before page up
	int position = GetScrollPos(scrollOrientation);
	// Get thumb size
	int thumbSize = GetScrollThumb(scrollOrientation);
	// Need to adjust position by a page
	position -= thumbSize;
	if (position < 0) position = 0;

	DoScrollCalc(position, scrollOrientation);
}
void mpWindow::OnScrollPageDown(wxScrollWinEvent& event)
{
	int scrollOrientation = event.GetOrientation();
	// Get position before page up
	int position = GetScrollPos(scrollOrientation);
	// Get thumb size
	int thumbSize = GetScrollThumb(scrollOrientation);
	// Get scroll range
	int scrollRange = GetScrollRange(scrollOrientation);
	// Need to adjust position by a page
	position += thumbSize;
	if (position > (scrollRange - thumbSize))
		position = scrollRange - thumbSize;

	DoScrollCalc(position, scrollOrientation);
}

void mpWindow::OnScrollLineUp(wxScrollWinEvent& event)
{
	int scrollOrientation = event.GetOrientation();
	// Get position before page up
	int position = GetScrollPos(scrollOrientation);
	// Need to adjust position by a line
	position -= mpSCROLL_NUM_PIXELS_PER_LINE;
	if (position < 0) position = 0;

	DoScrollCalc(position, scrollOrientation);
}

void mpWindow::OnScrollLineDown(wxScrollWinEvent& event)
{
	int scrollOrientation = event.GetOrientation();
	// Get position before page up
	int position = GetScrollPos(scrollOrientation);
	// Get thumb size
	int thumbSize = GetScrollThumb(scrollOrientation);
	// Get scroll range
	int scrollRange = GetScrollRange(scrollOrientation);
	// Need to adjust position by a page
	position += mpSCROLL_NUM_PIXELS_PER_LINE;
	if (position > (scrollRange - thumbSize))
		position = scrollRange - thumbSize;

	DoScrollCalc(position, scrollOrientation);
}

void mpWindow::OnScrollTop(wxScrollWinEvent& event)
{
	DoScrollCalc(0, event.GetOrientation());
}

void mpWindow::OnScrollBottom(wxScrollWinEvent& event)
{
	int scrollOrientation = event.GetOrientation();
	// Get thumb size
	int thumbSize = GetScrollThumb(scrollOrientation);
	// Get scroll range
	int scrollRange = GetScrollRange(scrollOrientation);

	DoScrollCalc(scrollRange - thumbSize, scrollOrientation);
}
// End patch ngpaton

void mpWindow::SetScaleX(double scaleX)
{
	if (scaleX != 0) m_scaleX = scaleX;
	UpdateAll();
}

// New methods implemented by Davide Rondini

unsigned int mpWindow::CountLayers()
{
	// wxNode *node = m_layers.GetFirst();
	unsigned int layerNo = 0;
	for (auto& m_layer : m_layers)  // while(node)
	{
		if (m_layer->HasBBox()) layerNo++;
		// node = node->GetNext();
	};
	return layerNo;
}

mpLayer* mpWindow::GetLayer(int position)
{
	if ((position >= (int)m_layers.size()) || position < 0) return nullptr;
	return m_layers[position];
}

mpLayer* mpWindow::GetLayerByName(const wxString& name)
{
	for (auto& m_layer : m_layers)
		if (!m_layer->GetName().Cmp(name)) return m_layer;
	return nullptr;  // Not found
}

void mpWindow::GetBoundingBox(double* bbox)
{
	bbox[0] = m_minX;
	bbox[1] = m_maxX;
	bbox[2] = m_minY;
	bbox[3] = m_maxY;
}

bool mpWindow::SaveScreenshot(
	const wxString& filename, int type, wxSize imageSize, bool fit)
{
	int sizeX, sizeY;
	int bk_scrX, bk_scrY;
	if (imageSize == wxDefaultSize)
	{
		sizeX = m_scrX;
		sizeY = m_scrY;
	}
	else
	{
		sizeX = imageSize.x;
		sizeY = imageSize.y;
		bk_scrX = m_scrX;
		bk_scrY = m_scrY;
		SetScr(sizeX, sizeY);
	}

	wxBitmap screenBuffer(sizeX, sizeY);
	wxMemoryDC screenDC;
	screenDC.SelectObject(screenBuffer);
	screenDC.SetPen(*wxTRANSPARENT_PEN);
	wxBrush brush(GetBackgroundColour());
	screenDC.SetBrush(brush);
	screenDC.DrawRectangle(0, 0, sizeX, sizeY);

	if (fit)
	{
		Fit(m_minX, m_maxX, m_minY, m_maxY, &sizeX, &sizeY);
	}
	else
	{
		Fit(m_desiredXmin, m_desiredXmax, m_desiredYmin, m_desiredYmax, &sizeX,
			&sizeY);
	}
	// Draw all the layers:
	wxLayerList::iterator li;
	for (li = m_layers.begin(); li != m_layers.end(); ++li)
		(*li)->Plot(screenDC, *this);

	if (imageSize != wxDefaultSize)
	{
		// Restore dimensions
		SetScr(bk_scrX, bk_scrY);
		Fit(m_desiredXmin, m_desiredXmax, m_desiredYmin, m_desiredYmax,
			&bk_scrX, &bk_scrY);
		UpdateAll();
	}
	// Once drawing is complete, actually save screen shot
	wxImage screenImage = screenBuffer.ConvertToImage();
	return screenImage.SaveFile(filename, (wxBitmapType)type);
}

void mpWindow::SetMargins(int top, int right, int bottom, int left)
{
	m_marginTop = top;
	m_marginRight = right;
	m_marginBottom = bottom;
	m_marginLeft = left;
}

mpInfoLayer* mpWindow::IsInsideInfoLayer(wxPoint& point)
{
	wxLayerList::iterator li;
	for (li = m_layers.begin(); li != m_layers.end(); ++li)
	{
#ifdef MATHPLOT_DO_LOGGING
		wxLogMessage(
			_("mpWindow::IsInsideInfoLayer() examinining layer = %p"), (*li));
#endif  // MATHPLOT_DO_LOGGING
		if ((*li)->IsInfo())
		{
			auto* tmpLyr = (mpInfoLayer*)(*li);
#ifdef MATHPLOT_DO_LOGGING
			wxLogMessage(_("mpWindow::IsInsideInfoLayer() layer = %p"), (*li));
#endif  // MATHPLOT_DO_LOGGING
			if (tmpLyr->Inside(point))
			{
				return tmpLyr;
			}
		}
	}
	return nullptr;
}

void mpWindow::SetLayerVisible(const wxString& name, bool viewable)
{
	mpLayer* lx = GetLayerByName(name);
	if (lx)
	{
		lx->SetVisible(viewable);
		UpdateAll();
	}
}

bool mpWindow::IsLayerVisible(const wxString& name)
{
	mpLayer* lx = GetLayerByName(name);
	return (lx) ? lx->IsVisible() : false;
}

void mpWindow::SetLayerVisible(const unsigned int position, bool viewable)
{
	mpLayer* lx = GetLayer(position);
	if (lx)
	{
		lx->SetVisible(viewable);
		UpdateAll();
	}
}

bool mpWindow::IsLayerVisible(const unsigned int position)
{
	mpLayer* lx = GetLayer(position);
	return (lx) ? lx->IsVisible() : false;
}

void mpWindow::SetColourTheme(
	const wxColour& bgColour, const wxColour& drawColour,
	const wxColour& axesColour)
{
	SetBackgroundColour(bgColour);
	SetForegroundColour(drawColour);
	m_bgColour = bgColour;
	m_fgColour = drawColour;
	m_axColour = axesColour;
	// cycle between layers to set colours and properties to them
	wxLayerList::iterator li;
	for (li = m_layers.begin(); li != m_layers.end(); ++li)
	{
		if ((*li)->GetLayerType() == mpLAYER_AXIS)
		{
			wxPen axisPen = (*li)->GetPen();  // Get the old pen to modify only
			// colour, not style or width
			axisPen.SetColour(axesColour);
			(*li)->SetPen(axisPen);
		}
		if ((*li)->GetLayerType() == mpLAYER_INFO)
		{
			wxPen infoPen = (*li)->GetPen();  // Get the old pen to modify only
			// colour, not style or width
			infoPen.SetColour(drawColour);
			(*li)->SetPen(infoPen);
		}
	}
}

// void mpWindow::EnableCoordTooltip(bool value)
// {
//      m_coordTooltip = value;
// //      if (value) GetToolTip()->SetDelay(100);
// }

/*
double mpWindow::p2x(wxCoord pixelCoordX, bool drawOutside )
{
	if (drawOutside) {
		return m_posX + pixelCoordX/m_scaleX;
	}
	// Draw inside margins
	double marginScaleX = ((double)(m_scrX - m_marginLeft -
m_marginRight))/m_scrX;
	return m_marginLeft + (m_posX + pixelCoordX/m_scaleX)/marginScaleX;
}

double mpWindow::p2y(wxCoord pixelCoordY, bool drawOutside )
{
	if (drawOutside) {
		return m_posY - pixelCoordY/m_scaleY;
	}
	// Draw inside margins
	double marginScaleY = ((double)(m_scrY - m_marginTop -
m_marginBottom))/m_scrY;
	return m_marginTop + (m_posY - pixelCoordY/m_scaleY)/marginScaleY;
}

wxCoord mpWindow::x2p(double x, bool drawOutside)
{
	if (drawOutside) {
		return (wxCoord) ((x-m_posX) * m_scaleX);
	}
	// Draw inside margins
	double marginScaleX = ((double)(m_scrX - m_marginLeft -
m_marginRight))/m_scrX;
#ifdef MATHPLOT_DO_LOGGING
	wxLogMessage(wxT("x2p ScrX = %d, marginRight = %d, marginLeft = %d,
marginScaleX = %f"), m_scrX, m_marginRight, m_marginLeft,  marginScaleX);
#endif // MATHPLOT_DO_LOGGING
	return (wxCoord) (int)(((x-m_posX) * m_scaleX)*marginScaleX) - m_marginLeft;
}

wxCoord mpWindow::y2p(double y, bool drawOutside)
{
	if (drawOutside) {
		return (wxCoord) ( (m_posY-y) * m_scaleY);
	}
	// Draw inside margins
	double marginScaleY = ((double)(m_scrY - m_marginTop -
m_marginBottom))/m_scrY;
#ifdef MATHPLOT_DO_LOGGING
	wxLogMessage(wxT("y2p ScrY = %d, marginTop = %d, marginBottom = %d,
marginScaleY = %f"), m_scrY, m_marginTop, m_marginBottom, marginScaleY);
#endif // MATHPLOT_DO_LOGGING
	return (wxCoord) ((int)((m_posY-y) * m_scaleY)*marginScaleY) - m_marginTop;
}
*/

//-----------------------------------------------------------------------------
// mpFXYVector implementation - by Jose Luis Blanco (AGO-2007)
//-----------------------------------------------------------------------------

IMPLEMENT_DYNAMIC_CLASS(mpFXYVector, mpFXY)

// Constructor
mpFXYVector::mpFXYVector(wxString name, int flags) : mpFXY(name, flags)
{
	m_index = 0;
	m_minX = -1;
	m_maxX = 1;
	m_minY = -1;
	m_maxY = 1;
	m_type = mpLAYER_PLOT;
}

void mpFXYVector::Rewind() { m_index = 0; }
bool mpFXYVector::GetNextXY(double& x, double& y)
{
	if (m_index >= m_xs.size())
		return FALSE;
	else
	{
		x = m_xs[m_index];
		y = m_ys[m_index++];
		return m_index <= m_xs.size();
	}
}

void mpFXYVector::Clear()
{
	m_xs.clear();
	m_ys.clear();
}

void mpFXYVector::SetData(
	const std::vector<float>& xs, const std::vector<float>& ys)
{
	// Check if the data vectora are of the same size
	if (xs.size() != ys.size())
	{
		wxLogError(
			_("wxMathPlot error: X and Y vector are not of the same length!"));
		return;
	}
	const size_t N = xs.size();
	std::vector<double> Xd(N), Yd(N);
	for (size_t i = 0; i < xs.size(); i++)
	{
		Xd[i] = xs[i];
		Yd[i] = ys[i];
	}
	SetData(Xd, Yd);
}

void mpFXYVector::SetData(
	const std::vector<double>& xs, const std::vector<double>& ys)
{
	// Check if the data vectora are of the same size
	if (xs.size() != ys.size())
	{
		wxLogError(
			_("wxMathPlot error: X and Y vector are not of the same length!"));
		return;
	}
	// Copy the data:
	m_xs = xs;
	m_ys = ys;

	// Update internal variables for the bounding box.
	if (xs.size() > 0)
	{
		m_minX = xs[0];
		m_maxX = xs[0];
		m_minY = ys[0];
		m_maxY = ys[0];

		std::vector<double>::const_iterator it;

		for (it = xs.begin(); it != xs.end(); ++it)
		{
			if (*it < m_minX) m_minX = *it;
			if (*it > m_maxX) m_maxX = *it;
		}
		for (it = ys.begin(); it != ys.end(); ++it)
		{
			if (*it < m_minY) m_minY = *it;
			if (*it > m_maxY) m_maxY = *it;
		}
		m_minX -= 0.5f;
		m_minY -= 0.5f;
		m_maxX += 0.5f;
		m_maxY += 0.5f;
	}
	else
	{
		m_minX = -1;
		m_maxX = 1;
		m_minY = -1;
		m_maxY = 1;
	}
}

void mpFXYVector::AppendDataPoint(float x, float y)
{
	// Append the data:
	m_xs.push_back(x);
	m_ys.push_back(y);

	if (m_xs.size() == 1)
	{
		m_minX = x;
		m_maxX = x;
		m_minY = y;
		m_maxY = y;

		const double margX = std::max(fabs(m_minX), fabs(m_maxX)) * 0.05;
		const double margY = std::max(fabs(m_minY), fabs(m_maxY)) * 0.05;

		m_minX -= margX;
		m_maxX += margX;
		m_minY -= margY;
		m_maxY += margY;
	}
	else
	{
		m_minX = std::min(x - fabs(x) * 0.05, m_minX);
		m_maxX = std::max(x + fabs(x) * 0.05, m_maxX);
		m_minY = std::min(y - fabs(y) * 0.05, m_minY);
		m_maxY = std::max(y + fabs(y) * 0.05, m_maxY);
	}
}

//-----------------------------------------------------------------------------
// mpText - provided by Val Greene
//-----------------------------------------------------------------------------

IMPLEMENT_DYNAMIC_CLASS(mpText, mpLayer)

/** @param name text to be displayed
@param offsetx x position in percentage (0-100)
@param offsetx y position in percentage (0-100)
*/
mpText::mpText(wxString name, int offsetx, int offsety)
{
	SetName(name);

	if (offsetx >= 0 && offsetx <= 100)
		m_offsetx = offsetx;
	else
		m_offsetx = 5;

	if (offsety >= 0 && offsety <= 100)
		m_offsety = offsety;
	else
		m_offsetx = 50;
	m_type = mpLAYER_INFO;
}

/** mpText Layer plot handler.
This implementation will plot the text adjusted to the visible area.
*/

void mpText::Plot(wxDC& dc, mpWindow& w)
{
	if (m_visible)
	{
		dc.SetPen(m_pen);
		dc.SetFont(m_font);

		wxCoord tw = 0, th = 0;
		dc.GetTextExtent(GetName(), &tw, &th);

		//     int left = -dc.LogicalToDeviceX(0);
		//     int width = dc.LogicalToDeviceX(0) - left;
		//     int bottom = dc.LogicalToDeviceY(0);
		//     int height = bottom - -dc.LogicalToDeviceY(0);

		/*    dc.DrawText( GetName(),
			(int)((((float)width/100.0) * m_offsety) + left - (tw/2)),
			(int)((((float)height/100.0) * m_offsetx) - bottom) );*/
		int px = m_offsetx *
				 (w.GetScrX() - w.GetMarginLeft() - w.GetMarginRight()) / 100;
		int py = m_offsety *
				 (w.GetScrY() - w.GetMarginTop() - w.GetMarginBottom()) / 100;
		dc.DrawText(GetName(), px, py);
	}
}

//-----------------------------------------------------------------------------
// mpPrintout - provided by Davide Rondini
//-----------------------------------------------------------------------------

mpPrintout::mpPrintout(mpWindow* drawWindow, const wxChar* title)
	: wxPrintout(title)
{
	drawn = false;
	plotWindow = drawWindow;
}

bool mpPrintout::OnPrintPage(int page)
{
	wxDC* trgDc = GetDC();
	if ((trgDc) && (page == 1))
	{
		wxCoord m_prnX, m_prnY;
		int marginX = 50;
		int marginY = 50;
		trgDc->GetSize(&m_prnX, &m_prnY);

		m_prnX -= (2 * marginX);
		m_prnY -= (2 * marginY);
		trgDc->SetDeviceOrigin(marginX, marginY);

#ifdef MATHPLOT_DO_LOGGING
		wxLogMessage(wxT("Print Size: %d x %d\n"), m_prnX, m_prnY);
		wxLogMessage(
			wxT("Screen Size: %d x %d\n"), plotWindow->GetScrX(),
			plotWindow->GetScrY());
#endif

		// Set the scale according to the page:
		plotWindow->Fit(
			plotWindow->GetDesiredXmin(), plotWindow->GetDesiredXmax(),
			plotWindow->GetDesiredYmin(), plotWindow->GetDesiredYmax(), &m_prnX,
			&m_prnY);

		// Get the colours of the plotWindow to restore them ath the end
		wxColour oldBgColour = plotWindow->GetBackgroundColour();
		wxColour oldFgColour = plotWindow->GetForegroundColour();
		wxColour oldAxColour = plotWindow->GetAxesColour();

		// Draw background, ensuring to use white background for printing.
		trgDc->SetPen(*wxTRANSPARENT_PEN);
		// wxBrush brush( plotWindow->GetBackgroundColour() );
		wxBrush brush = *wxWHITE_BRUSH;
		trgDc->SetBrush(brush);
		trgDc->DrawRectangle(0, 0, m_prnX, m_prnY);

		// Draw all the layers:
		// trgDc->SetDeviceOrigin( m_prnX>>1, m_prnY>>1);  // Origin at the
		// center
		mpLayer* layer;
		for (unsigned int li = 0; li < plotWindow->CountAllLayers(); ++li)
		{
			layer = plotWindow->GetLayer(li);
			layer->Plot(*trgDc, *plotWindow);
		};
		// Restore device origin
		// trgDc->SetDeviceOrigin(0, 0);
		// Restore colours
		plotWindow->SetColourTheme(oldBgColour, oldFgColour, oldAxColour);
		// Restore drawing
		plotWindow->Fit(
			plotWindow->GetDesiredXmin(), plotWindow->GetDesiredXmax(),
			plotWindow->GetDesiredYmin(), plotWindow->GetDesiredYmax(), nullptr,
			nullptr);
		plotWindow->UpdateAll();
	}
	return true;
}

bool mpPrintout::HasPage(int page) { return (page == 1); }
//-----------------------------------------------------------------------------
// mpMovableObject - provided by Jose Luis Blanco
//-----------------------------------------------------------------------------
void mpMovableObject::TranslatePoint(
	double x, double y, double& out_x, double& out_y)
{
	double ccos = cos(m_reference_phi);  // Avoid computing cos/sin twice.
	double csin = sin(m_reference_phi);

	out_x = m_reference_x + ccos * x - csin * y;
	out_y = m_reference_y + csin * x + ccos * y;
}

// This method updates the buffers m_trans_shape_xs/ys, and the precomputed
// bounding box.
void mpMovableObject::ShapeUpdated()
{
	// Just in case...
	if (m_shape_xs.size() != m_shape_ys.size())
	{
		wxLogError(
			wxT("[mpMovableObject::ShapeUpdated] Error, m_shape_xs and "
				"m_shape_ys have different lengths!"));
	}
	else
	{
		double ccos = cos(m_reference_phi);  // Avoid computing cos/sin twice.
		double csin = sin(m_reference_phi);

		m_trans_shape_xs.resize(m_shape_xs.size());
		m_trans_shape_ys.resize(m_shape_xs.size());

		std::vector<double>::iterator itXi, itXo;
		std::vector<double>::iterator itYi, itYo;

		m_bbox_min_x = 1e300;
		m_bbox_max_x = -1e300;
		m_bbox_min_y = 1e300;
		m_bbox_max_y = -1e300;

		for (itXo = m_trans_shape_xs.begin(), itYo = m_trans_shape_ys.begin(),
			itXi = m_shape_xs.begin(), itYi = m_shape_ys.begin();
			 itXo != m_trans_shape_xs.end(); ++itXo, ++itYo, ++itXi, ++itYi)
		{
			*itXo = m_reference_x + ccos * (*itXi) - csin * (*itYi);
			*itYo = m_reference_y + csin * (*itXi) + ccos * (*itYi);

			// Keep BBox:
			if (*itXo < m_bbox_min_x) m_bbox_min_x = *itXo;
			if (*itXo > m_bbox_max_x) m_bbox_max_x = *itXo;
			if (*itYo < m_bbox_min_y) m_bbox_min_y = *itYo;
			if (*itYo > m_bbox_max_y) m_bbox_max_y = *itYo;
		}
	}
}

void mpMovableObject::Plot(wxDC& dc, mpWindow& w)
{
	if (m_visible)
	{
		dc.SetPen(m_pen);

		auto itX = m_trans_shape_xs.begin();
		auto itY = m_trans_shape_ys.begin();

		if (!m_continuous)
		{
			// for some reason DrawPoint does not use the current pen,
			// so we use DrawLine for fat pens
			if (m_pen.GetWidth() <= 1)
			{
				while (itX != m_trans_shape_xs.end())
				{
					dc.DrawPoint(w.x2p(*(itX++)), w.y2p(*(itY++)));
				}
			}
			else
			{
				while (itX != m_trans_shape_xs.end())
				{
					wxCoord cx = w.x2p(*(itX++));
					wxCoord cy = w.y2p(*(itY++));
					dc.DrawLine(cx, cy, cx, cy);
				}
			}
		}
		else
		{
			wxCoord cx0 = 0, cy0 = 0;
			bool first = TRUE;
			while (itX != m_trans_shape_xs.end())
			{
				wxCoord cx = w.x2p(*(itX++));
				wxCoord cy = w.y2p(*(itY++));
				if (first)
				{
					first = FALSE;
					cx0 = cx;
					cy0 = cy;
				}
				dc.DrawLine(cx0, cy0, cx, cy);
				cx0 = cx;
				cy0 = cy;
			}
		}

		if (!m_name.IsEmpty() && m_showName)
		{
			dc.SetFont(m_font);

			wxCoord tx, ty;
			dc.GetTextExtent(m_name, &tx, &ty);

			if (HasBBox())
			{
				auto sx =
					(wxCoord)((m_bbox_max_x - w.GetPosX()) * w.GetScaleX());
				auto sy =
					(wxCoord)((w.GetPosY() - m_bbox_max_y) * w.GetScaleY());

				tx = sx - tx - 8;
				ty = sy - 8 - ty;
			}
			else
			{
				const int sx = w.GetScrX() >> 1;
				const int sy = w.GetScrY() >> 1;

				if ((m_flags & mpALIGNMASK) == mpALIGN_NE)
				{
					tx = sx - tx - 8;
					ty = -sy + 8;
				}
				else if ((m_flags & mpALIGNMASK) == mpALIGN_NW)
				{
					tx = -sx + 8;
					ty = -sy + 8;
				}
				else if ((m_flags & mpALIGNMASK) == mpALIGN_SW)
				{
					tx = -sx + 8;
					ty = sy - 8 - ty;
				}
				else
				{
					tx = sx - tx - 8;
					ty = sy - 8 - ty;
				}
			}

			dc.DrawText(m_name, tx, ty);
		}
	}
}

//-----------------------------------------------------------------------------
// mpCovarianceEllipse - provided by Jose Luis Blanco
//-----------------------------------------------------------------------------

// Called to update the m_shape_xs, m_shape_ys vectors, whenever a parameter
// changes.
void mpCovarianceEllipse::RecalculateShape()
{
	m_shape_xs.clear();
	m_shape_ys.clear();

	// Preliminary checks:
	if (m_quantiles < 0)
	{
		wxLogError(
			wxT("[mpCovarianceEllipse] Error: quantiles must be non-negative"));
		return;
	}
	if (m_cov_00 < 0)
	{
		wxLogError(
			wxT("[mpCovarianceEllipse] Error: cov(0,0) must be non-negative"));
		return;
	}
	if (m_cov_11 < 0)
	{
		wxLogError(
			wxT("[mpCovarianceEllipse] Error: cov(1,1) must be non-negative"));
		return;
	}

	m_shape_xs.resize(m_segments, 0);
	m_shape_ys.resize(m_segments, 0);

	// Compute the two eigenvalues of the covariance:
	// -------------------------------------------------
	double b = -m_cov_00 - m_cov_11;
	double c = m_cov_00 * m_cov_11 - m_cov_01 * m_cov_01;

	double D = b * b - 4 * c;

	if (D < 0)
	{
		wxLogError(
			wxT("[mpCovarianceEllipse] Error: cov is not positive definite"));
		return;
	}

	double eigenVal0 = 0.5 * (-b + sqrt(D));
	double eigenVal1 = 0.5 * (-b - sqrt(D));

	// Compute the two corresponding eigenvectors:
	// -------------------------------------------------
	double eigenVec0_x, eigenVec0_y;
	double eigenVec1_x, eigenVec1_y;

	if (D == 0)  // Square ellipse
	{
		eigenVec0_y = 0;
		eigenVec0_x = 1;
	}
	else if (fabs(eigenVal0 - m_cov_00) > 1e-6)
	{
		double k1x = m_cov_01 / (eigenVal0 - m_cov_00);
		eigenVec0_y = 1;
		eigenVec0_x = eigenVec0_y * k1x;
	}
	else
	{
		double k1y = m_cov_01 / (eigenVal0 - m_cov_11);
		eigenVec0_x = 1;
		eigenVec0_y = eigenVec0_x * k1y;
	}

	if (D == 0)  // Square ellipse
	{
		eigenVec1_y = 1;
		eigenVec1_x = 0;
	}
	else if (fabs(eigenVal1 - m_cov_00) > 1e-6)
	{
		double k2x = m_cov_01 / (eigenVal1 - m_cov_00);
		eigenVec1_y = 1;
		eigenVec1_x = eigenVec1_y * k2x;
	}
	else
	{
		double k2y = m_cov_01 / (eigenVal1 - m_cov_11);
		eigenVec1_x = 1;
		eigenVec1_y = eigenVec1_x * k2y;
	}

	// Normalize the eigenvectors:
	double len = sqrt(eigenVec0_x * eigenVec0_x + eigenVec0_y * eigenVec0_y);
	eigenVec0_x /= len;  // It *CANNOT* be zero
	eigenVec0_y /= len;

	len = sqrt(eigenVec1_x * eigenVec1_x + eigenVec1_y * eigenVec1_y);
	eigenVec1_x /= len;  // It *CANNOT* be zero
	eigenVec1_y /= len;

	// Take the sqrt of the eigenvalues (required for the ellipse scale):
	eigenVal0 = sqrt(eigenVal0);
	eigenVal1 = sqrt(eigenVal1);

	// Compute the 2x2 matrix M = diag(eigVal) * (~eigVec)  (each eigen vector
	// is a row):
	double M_00 = eigenVec0_x * eigenVal0;
	double M_01 = eigenVec0_y * eigenVal0;

	double M_10 = eigenVec1_x * eigenVal1;
	double M_11 = eigenVec1_y * eigenVal1;

	// The points of the 2D ellipse:
	double ang;
	double Aang = 6.283185308 / (m_segments - 1);
	int i;
	for (i = 0, ang = 0; i < m_segments; i++, ang += Aang)
	{
		double ccos = cos(ang);
		double csin = sin(ang);

		m_shape_xs[i] = m_quantiles * (ccos * M_00 + csin * M_10);
		m_shape_ys[i] = m_quantiles * (ccos * M_01 + csin * M_11);
	}  // end for points on ellipse

	ShapeUpdated();
}

//-----------------------------------------------------------------------------
// mpPolygon - provided by Jose Luis Blanco
//-----------------------------------------------------------------------------
void mpPolygon::setPoints(
	const std::vector<double>& points_xs, const std::vector<double>& points_ys,
	bool closedShape)
{
	if (points_xs.size() != points_ys.size())
	{
		wxLogError(
			wxT("[mpPolygon] Error: points_xs and points_ys must have the same "
				"number of elements"));
	}
	else
	{
		m_shape_xs = points_xs;
		m_shape_ys = points_ys;

		if (closedShape && !points_xs.empty())
		{
			m_shape_xs.push_back(points_xs[0]);
			m_shape_ys.push_back(points_ys[0]);
		}

		ShapeUpdated();
	}
}
void mpPolygon::setPoints(
	const std::vector<float>& points_xs, const std::vector<float>& points_ys,
	bool closedShape)
{
	if (points_xs.size() != points_ys.size())
	{
		wxLogError(
			wxT("[mpPolygon] Error: points_xs and points_ys must have the same "
				"number of elements"));
	}
	else
	{
		m_shape_xs.resize(points_xs.size());
		m_shape_ys.resize(points_xs.size());

		if (!points_xs.empty())
		{
			std::vector<float>::const_iterator itX, itY;
			std::vector<double>::iterator itXo, itYo;
			for (itX = points_xs.begin(), itY = points_ys.begin(),
				itXo = m_shape_xs.begin(), itYo = m_shape_ys.begin();
				 itX != points_xs.end(); ++itX, ++itY, ++itXo, ++itYo)
			{
				*itXo = (double)*itX;
				*itYo = (double)*itY;
			}

			if (closedShape)
			{
				m_shape_xs.push_back((double)points_xs[0]);
				m_shape_ys.push_back((double)points_ys[0]);
			}
		}

		ShapeUpdated();
	}
}

//-----------------------------------------------------------------------------
// mpBitmapLayer - provided by Jose Luis Blanco
//-----------------------------------------------------------------------------
void mpBitmapLayer::GetBitmapCopy(wxImage& outBmp) const
{
	if (m_validImg) outBmp = m_bitmap;
}

void mpBitmapLayer::SetBitmap(
	const wxImage& inBmp, double x, double y, double lx, double ly)
{
	if (!inBmp.Ok())
	{
		wxLogError(wxT("[mpBitmapLayer] Assigned bitmap is not Ok()!"));
	}
	else
	{
		if (lx < 0)
		{
			wxLogError(
				wxT("[mpBitmapLayer::SetBitmap] Assigned lx is negative!!"));
		}
		if (ly < 0)
		{
			wxLogError(
				wxT("[mpBitmapLayer::SetBitmap] Assigned ly is negative!!"));
		}

		m_bitmap = inBmp;  //.GetSubBitmap( wxRect(0, 0, inBmp.GetWidth(),
		// inBmp.GetHeight()));
		m_min_x = x;
		m_min_y = y;
		m_max_x = x + lx;
		m_max_y = y + ly;
		m_validImg = true;
	}
}

void mpBitmapLayer::Plot(wxDC& dc, mpWindow& w)
{
	if (m_visible && m_validImg)
	{
		/*	1st: We compute (x0,y0)-(x1,y1), the pixel coordinates of the real
		outer limits
				 of the image rectangle within the (screen) mpWindow. Note that
		these coordinates
				 might fall well far away from the real view limits when the
		user zoom in.

			2nd: We compute (dx0,dy0)-(dx1,dy1), the pixel coordinates the
		rectangle that will
				 be actually drawn into the mpWindow, i.e. the clipped real
		rectangle that
				 avoids the non-visible parts. (offset_x,offset_y) are the pixel
		coordinates
				 that correspond to the window point (dx0,dy0) within the image
		"m_bitmap", and
				 (b_width,b_height) is the size of the bitmap patch that will be
		drawn.

		(x0,y0) .................  (x1,y0)
			.                          .
			.                          .
		(x0,y1) ................   (x1,y1)
					  (In pixels!!)
		*/

		// 1st step -------------------------------
		wxCoord x0 = w.x2p(m_min_x);
		wxCoord y0 = w.y2p(m_max_y);
		wxCoord x1 = w.x2p(m_max_x);
		wxCoord y1 = w.y2p(m_min_y);

		// 2nd step -------------------------------
		// Precompute the size of the actual bitmap pixel on the screen (e.g.
		// will be >1 if zoomed in)
		double screenPixelX = (x1 - x0) / (double)m_bitmap.GetWidth();
		double screenPixelY = (y1 - y0) / (double)m_bitmap.GetHeight();

		// The minimum number of pixels that the streched image will overpass
		// the actual mpWindow borders:
		auto borderMarginX = (wxCoord)(screenPixelX + 1);  // ceil
		auto borderMarginY = (wxCoord)(screenPixelY + 1);  // ceil

		// The actual drawn rectangle (dx0,dy0)-(dx1,dy1) is (x0,y0)-(x1,y1)
		// clipped:
		wxCoord dx0 = x0, dx1 = x1, dy0 = y0, dy1 = y1;
		if (dx0 < 0) dx0 = -borderMarginX;
		if (dy0 < 0) dy0 = -borderMarginY;
		if (dx1 > w.GetScrX()) dx1 = w.GetScrX() + borderMarginX;
		if (dy1 > w.GetScrY()) dy1 = w.GetScrY() + borderMarginY;

		// For convenience, compute the width/height of the rectangle to be
		// actually drawn:
		wxCoord d_width = dx1 - dx0 + 1;
		wxCoord d_height = dy1 - dy0 + 1;

		// Compute the pixel offsets in the internally stored bitmap:
		auto offset_x = (wxCoord)((dx0 - x0) / screenPixelX);
		auto offset_y = (wxCoord)((dy0 - y0) / screenPixelY);

		// and the size in pixel of the area to be actually drawn from the
		// internally stored bitmap:
		auto b_width = (wxCoord)((dx1 - dx0 + 1) / screenPixelX);
		auto b_height = (wxCoord)((dy1 - dy0 + 1) / screenPixelY);

#ifdef MATHPLOT_DO_LOGGING
		wxLogMessage(
			_("[mpBitmapLayer::Plot] screenPixel: x=%f y=%f  d_width=%ix%i"),
			screenPixelX, screenPixelY, d_width, d_height);
		wxLogMessage(
			_("[mpBitmapLayer::Plot] offset: x=%i y=%i  bmpWidth=%ix%i"),
			offset_x, offset_y, b_width, b_height);
#endif

		// Is there any visible region?
		if (d_width > 0 && d_height > 0)
		{
			// Build the scaled bitmap from the image, only if it has changed:
			if (m_scaledBitmap.GetWidth() != d_width ||
				m_scaledBitmap.GetHeight() != d_height ||
				m_scaledBitmap_offset_x != offset_x ||
				m_scaledBitmap_offset_y != offset_y)
			{
				wxRect r(wxRect(offset_x, offset_y, b_width, b_height));
				// Just for the case....
				if (r.x < 0) r.x = 0;
				if (r.y < 0) r.y = 0;
				if (r.width > m_bitmap.GetWidth())
					r.width = m_bitmap.GetWidth();
				if (r.height > m_bitmap.GetHeight())
					r.height = m_bitmap.GetHeight();

				m_scaledBitmap = wxBitmap(
					wxBitmap(m_bitmap).GetSubBitmap(r).ConvertToImage().Scale(
						d_width, d_height));
				m_scaledBitmap_offset_x = offset_x;
				m_scaledBitmap_offset_y = offset_y;
			}

			// Draw it:
			dc.DrawBitmap(m_scaledBitmap, dx0, dy0, true);
		}
	}

	// Draw the name label
	if (!m_name.IsEmpty() && m_showName)
	{
		dc.SetFont(m_font);

		wxCoord tx, ty;
		dc.GetTextExtent(m_name, &tx, &ty);

		if (HasBBox())
		{
			auto sx = (wxCoord)((m_max_x - w.GetPosX()) * w.GetScaleX());
			auto sy = (wxCoord)((w.GetPosY() - m_max_y) * w.GetScaleY());

			tx = sx - tx - 8;
			ty = sy - 8 - ty;
		}
		else
		{
			const int sx = w.GetScrX() >> 1;
			const int sy = w.GetScrY() >> 1;

			if ((m_flags & mpALIGNMASK) == mpALIGN_NE)
			{
				tx = sx - tx - 8;
				ty = -sy + 8;
			}
			else if ((m_flags & mpALIGNMASK) == mpALIGN_NW)
			{
				tx = -sx + 8;
				ty = -sy + 8;
			}
			else if ((m_flags & mpALIGNMASK) == mpALIGN_SW)
			{
				tx = -sx + 8;
				ty = sy - 8 - ty;
			}
			else
			{
				tx = sx - tx - 8;
				ty = sy - 8 - ty;
			}
		}

		dc.DrawText(m_name, tx, ty);
	}
}

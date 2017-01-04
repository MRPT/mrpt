/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "CRawlogTreeView.h"

#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
#include <wx/window.h>
#include <wx/dcclient.h>
#include <wx/imaglist.h>
#include <wx/dcbuffer.h>
#include <mrpt/gui/wx28-fixes.h>

IMPLEMENT_DYNAMIC_CLASS(CRawlogTreeView, wxScrolledWindow)

BEGIN_EVENT_TABLE(CRawlogTreeView, wxScrolledWindow)
	EVT_LEFT_DOWN ( CRawlogTreeView::OnLeftDown )
    EVT_MOUSEWHEEL( CRawlogTreeView::OnMouseWheel )
	EVT_CHAR      ( CRawlogTreeView::OnKey )
END_EVENT_TABLE()

#include <mrpt/system/datetime.h>

#define MRPT_NO_WARN_BIG_HDR // It's ok here
#include <mrpt/obs.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::obs;
using namespace std;

const int CRawlogTreeView::ROW_HEIGHT = 17;
const int CRawlogTreeView::TREE_HORZ_STEPS = 25;

#ifdef wxUSE_UNICODE
#define _U(x) wxString((x),wxConvUTF8)
#define _UU(x,y) wxString((x),y)
#else
#define _U(x) (x)
#define _UU(x,y) (x)
#endif



/* ------------------------------------------------------------
						CRawlogTreeView
   ------------------------------------------------------------ */
CRawlogTreeView::CRawlogTreeView(
	wxWindow* parent,
	wxWindowID id,
	const wxPoint& pos,
	const wxSize& size,
	long style,
	const wxString& name) :
		wxScrolledWindow(parent,id,pos,size,style | wxVSCROLL | wxFULL_REPAINT_ON_RESIZE ,name),
		m_rawlog(NULL),
		m_imageList(NULL),
		m_selectedItem(-1),
		m_event_select_change(NULL),
		m_win_parent(NULL),
		m_rawlog_start(INVALID_TIMESTAMP),
		m_rawlog_last(INVALID_TIMESTAMP),
		m_tree_nodes()
{
}

/* ------------------------------------------------------------
						Destructor
   ------------------------------------------------------------ */
CRawlogTreeView::~CRawlogTreeView()
{
	if (m_imageList)
		delete m_imageList;
}


/* ------------------------------------------------------------
						setRawlogSource
   ------------------------------------------------------------ */
void CRawlogTreeView::setRawlogSource( CRawlog *rawlog )
{
	m_rawlog = rawlog;
	reloadFromRawlog();
}

/* ------------------------------------------------------------
						reloadFromRawlog
   ------------------------------------------------------------ */
void CRawlogTreeView::reloadFromRawlog( int hint_rawlog_items )
{
	// Recompute the total height of the scroll area:
	//  We also compute a list for each index with:
	//    - Pointer to data
	//	  - level in the hierarchy (0,1,2)
	// --------------------------------------------------------

	if (m_rawlog)
	{
		if (hint_rawlog_items<0)
				m_tree_nodes.reserve( m_rawlog->size()+100 );
		else	m_tree_nodes.reserve( hint_rawlog_items+100 );
	}

	// Create a "tree node" for each element in the rawlog:
	// ---------------------------------------------------------
	m_tree_nodes.clear();

	m_rawlog_start = INVALID_TIMESTAMP;
	m_rawlog_last  = INVALID_TIMESTAMP;

	// Root:
	m_tree_nodes.push_back( TNodeData() );
	TNodeData  &d = m_tree_nodes.back();
	d.level = 0;

//	CVectorDouble	tims;

	if (m_rawlog)
	{
		CRawlog::iterator end_it = m_rawlog->end();
		size_t		rawlog_index = 0;
		for (CRawlog::iterator it=m_rawlog->begin();it!=end_it;it++,rawlog_index++)
		{
			m_tree_nodes.push_back( TNodeData() );
			TNodeData  &d = m_tree_nodes.back();
			d.level = 1;
			d.data = (*it);
			d.index = rawlog_index;

			// For containers, go recursively:
			if ( (*it)->GetRuntimeClass()==CLASS_ID(CSensoryFrame))
			{
				CSensoryFramePtr	sf = CSensoryFramePtr( *it );
				for (CSensoryFrame::iterator o=sf->begin();o!=sf->end();++o)
				{
					m_tree_nodes.push_back( TNodeData() );
					TNodeData  &d = m_tree_nodes.back();
					d.level = 2;
					d.data = (*o);

                    if ((*o)->timestamp!=INVALID_TIMESTAMP)
                    {
                        m_rawlog_last = (*o)->timestamp;
                        if (m_rawlog_start == INVALID_TIMESTAMP)
                            m_rawlog_start = (*o)->timestamp;
                    }
				}
			}
			else
			if ( (*it)->GetRuntimeClass()==CLASS_ID(CActionCollection))
			{
				CActionCollectionPtr	acts = CActionCollectionPtr( *it );
				for (CActionCollection::iterator a=acts->begin();a!=acts->end();++a)
				{
					m_tree_nodes.push_back( TNodeData() );
					TNodeData  &d = m_tree_nodes.back();
					d.level = 2;
					d.data = a->get_ptr();

					if ((*a)->timestamp!=INVALID_TIMESTAMP)
					{
						m_rawlog_last = (*a)->timestamp;
						if (m_rawlog_start == INVALID_TIMESTAMP)
							m_rawlog_start = (*a)->timestamp;
					}
				}
			}
			else
			if ( (*it)->GetRuntimeClass()->derivedFrom( CLASS_ID(CObservation) ))
			{
			    CObservationPtr o = CObservationPtr(*it);
                if (o->timestamp!=INVALID_TIMESTAMP)
                {
                    m_rawlog_last = o->timestamp;
                    if (m_rawlog_start == INVALID_TIMESTAMP)
                        m_rawlog_start = o->timestamp;

					//tims.push_back( mrpt::system::timeDifference(m_rawlog_start, o->timestamp));
                }
			}
		}
	}

//	mrpt::system::vectorToTextFile(tims,"tims.txt");

	// Set new size:
	int ly = m_tree_nodes.size();
	SetScrollbars(ROW_HEIGHT, ROW_HEIGHT, 50, ly);
}

/* ------------------------------------------------------------
						reloadFromRawlog
   ------------------------------------------------------------ */
void CRawlogTreeView::OnDraw(wxDC& dc)
{
	// The origin of the window:
	int xc0,y0;
	GetViewStart(&xc0,&y0);		// Not pixels, but **lines**

	int w,h;
	GetSize(&w,&h);

	wxRect  visibleArea(xc0*ROW_HEIGHT,y0*ROW_HEIGHT,w*ROW_HEIGHT,h*ROW_HEIGHT);

	//wxBufferedDC	dc(&real_dc, visibleArea.GetSize() ); //wxSize(w,h) );

	dc.SetPen( *wxTRANSPARENT_PEN );
	dc.SetBrush( *wxWHITE_BRUSH );
	dc.DrawRectangle(xc0*ROW_HEIGHT,y0*ROW_HEIGHT,w,h);

	// The first time only, create fonts:
	static wxFont	font_normal(11,wxFONTFAMILY_ROMAN, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL );
	static wxFont	font_bold  (11,wxFONTFAMILY_ROMAN, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD  );
	static wxColour wxColorGray(40,40,40);
	static wxColour brush_SELECTED_COLOR( 0,200, 200);
	static wxPen gray_thick( brush_SELECTED_COLOR, 3 );
	static wxPen gray_thin( brush_SELECTED_COLOR, 1 );

	size_t first_item = y0;

	size_t n_visible_items = (h/ROW_HEIGHT)+1;
	if (n_visible_items<1) n_visible_items=1;

	size_t last_item = first_item+n_visible_items-1;

	last_item = min(last_item, m_tree_nodes.size() );

	// Draw icons?
	// ----------------------------
	int   x0 = 30;
	int   Ax0_img  = 0; // Increment in x for the icons, if any.

	if (m_imageList && m_imageList->GetImageCount()>0)
	{
		int		imgw,imgh;
		m_imageList->GetSize(0,imgw,imgh);
		Ax0_img = imgw + 4;
	}

	// Draw only those elements that are visible:
	// -----------------------------------------------
	const int		first_tim_y = ROW_HEIGHT + 1;
	const int		last_tim_y  = (m_tree_nodes.size()-1)*ROW_HEIGHT - 1;


	for (size_t i=first_item;i<last_item;i++)
	{
		int y = i*ROW_HEIGHT;	// y= bottom of that row
		TNodeData  &d = m_tree_nodes[i];

		switch (d.level)
		{
		case 0:
			dc.SetFont( font_bold );
			dc.SetTextForeground( *wxBLACK );
			break;

		case 1:
			dc.SetFont( font_normal );
			dc.SetTextForeground( *wxBLACK );
			break;

		default:
		case 2:
			dc.SetFont( font_normal );
			dc.SetTextForeground( wxColorGray );
			break;
		};

		// Select icon & text according to type of object
		// ------------------------------------------------
		int			icon = -1;
		wxString	s;

		if (i==0)
		{
			// The root node:
			s = _U( format("Rawlog: %s",m_rawlog_name.c_str()).c_str() );
			icon = 3;
		}
		else
		{
			// According to class ID:
			m_rawlog->getAsGeneric( d.index );  // Just to assure it's on memory

			if (d.data.present())
			{
				// Icon:
				icon = iconIndexFromClass( d.data->GetRuntimeClass() );

				// Text:
				if (d.level==1)
				{
					s << _U( format("[%i] ",(int)d.index ).c_str() );
				}

				s << _U( d.data->GetRuntimeClass()->className );

				// Sensor label:
				if ( d.data->GetRuntimeClass()->derivedFrom( CLASS_ID(CObservation) ) )
				{
					CObservationPtr obs = CObservationPtr(d.data);

					/*if (first_tim==INVALID_TIMESTAMP)
					{
						first_tim = obs->timestamp;
						first_tim_y = y+1;
					}

					last_tim = obs->timestamp;
					last_tim_y = y+1;*/

					if ( !obs->sensorLabel.empty())
						s << wxT(" : ") << _U( obs->sensorLabel.c_str() );
				}
			}
		}

		// Draw text and icon:
		// ------------------------------------
		if (m_selectedItem==int(i))
		{
			dc.SetBrush( brush_SELECTED_COLOR );
			dc.DrawRectangle(x0+TREE_HORZ_STEPS * d.level,y,w-x0,ROW_HEIGHT);
			dc.SetTextBackground( brush_SELECTED_COLOR );

			dc.SetPen( gray_thick );
			dc.DrawLine(x0,y+1,x0+TREE_HORZ_STEPS * d.level,y+1);
		}
		else
		{
			dc.SetTextBackground( GetBackgroundColour() );

			dc.SetPen( gray_thin );
			dc.DrawLine(x0,y+1,x0+TREE_HORZ_STEPS * d.level,y+1);
		}

		dc.DrawText( s, Ax0_img + x0 + TREE_HORZ_STEPS * d.level, y );
		if (m_imageList && icon>=0)
			m_imageList->Draw(icon, dc,x0 + TREE_HORZ_STEPS * d.level +1,y );
	}

	// Draw time-line:
	// -----------------------------------------------
	dc.SetPen( *wxLIGHT_GREY_PEN );
	dc.DrawLine(15, visibleArea.GetTop() ,15, visibleArea.GetBottom() );

	dc.SetTextForeground( wxColorGray );
	dc.SetFont( font_normal );
	dc.DrawRotatedText(_("Time"),17,10,-90);

	// timestamps in time-line:
	// -----------------------------------------------
	if (m_rawlog_start!=INVALID_TIMESTAMP && m_rawlog_last!=INVALID_TIMESTAMP && last_tim_y>first_tim_y)
	{
		dc.SetFont( font_normal );

		const double len_tim = mrpt::system::timeDifference(m_rawlog_start,m_rawlog_last);

		for (size_t i=first_item;i<last_item;i++)
		{
			int y = i*ROW_HEIGHT;	// y= bottom of that row
			TNodeData  &d = m_tree_nodes[i];

			if (d.data.present())
			{
				TTimeStamp	t_this = INVALID_TIMESTAMP;
				if ( d.data->GetRuntimeClass()->derivedFrom( CLASS_ID(CObservation) ) )
				{
					CObservationPtr obs = CObservationPtr(d.data);
					t_this = obs->timestamp;
				}

				if (t_this!=INVALID_TIMESTAMP)
				{
					// Draw line:
					if (m_selectedItem==int(i))
							dc.SetPen( gray_thick );
					else	dc.SetPen( gray_thin );

					// Calc. the "y" coordinate for this time:
					double At = mrpt::system::timeDifference(m_rawlog_start,t_this);
					double rat = At / len_tim;
					int  ty =  first_tim_y + (last_tim_y-first_tim_y)*rat;

					dc.DrawLine(15,ty,x0,y+1);

					// Draw some text labels with times:
					if ((i%5)==0)
					{
						dc.DrawLine(10,ty,15,ty);

						// Seconds from start of rawlog:
						if (m_rawlog_start!=INVALID_TIMESTAMP)
						{
							double sec = mrpt::system::timeDifference(m_rawlog_start,t_this);
							wxString s = _U( format("%.03fs", sec ).c_str() );
							dc.DrawRotatedText(s,17,ty+3,-90);
						}
					}
				}
			}
		} // end for i
	}
}

// Return an icon index depending on the class of the object in the tree view:
int CRawlogTreeView::iconIndexFromClass( const TRuntimeClassId* class_ID )
{
	int iconIndex=-1;

	if ( class_ID==CLASS_ID(CObservation2DRangeScan) )
		iconIndex = 6;
	else if ( class_ID==CLASS_ID(CObservationImage) )
		iconIndex = 4;
	else if ( class_ID==CLASS_ID(CObservationStereoImages) )
		iconIndex = 5;
	else if ( class_ID==CLASS_ID(CObservationGPS) )
		iconIndex = 7;
	else if ( class_ID==CLASS_ID(CObservationGasSensors) )
		iconIndex = 8;
	else if ( class_ID==CLASS_ID(CObservationWirelessPower) )
		iconIndex = 8;
	else if ( class_ID==CLASS_ID(CObservationRFID) )
		iconIndex = 8;
	else if (class_ID->derivedFrom( CLASS_ID(CObservation) ))
		iconIndex = 2;	// Default observation
	else if ( class_ID==CLASS_ID(CActionCollection) )
		iconIndex = 0;
	else if ( class_ID==CLASS_ID(CSensoryFrame) )
		iconIndex = 1;
	else if (class_ID->derivedFrom( CLASS_ID(CAction) ))
		iconIndex = 2;

	return iconIndex;
}

/* ------------------------------------------------------------
						OnLeftDown
   ------------------------------------------------------------ */
void CRawlogTreeView::OnLeftDown(wxMouseEvent& e)
{
	// The origin of the window:
	int xc0,y0;
	GetViewStart(&xc0,&y0);		// Not pixels, but **lines**

	// Determine the clicked row:
	int nLineThisView = e.GetY() / ROW_HEIGHT;

	size_t sel_item = y0 + nLineThisView;

	SetSelectedItem(sel_item);
}

/* ------------------------------------------------------------
						ConnectSelectedItemChange
   ------------------------------------------------------------ */
void CRawlogTreeView::ConnectSelectedItemChange( wxRawlogTreeEventFunction func)
{
	m_event_select_change = func;
}


/* ------------------------------------------------------------
						SetSelectedItem
   ------------------------------------------------------------ */
void CRawlogTreeView::SetSelectedItem( int sel_item, bool force_refresh )
{
	if (sel_item<(int)m_tree_nodes.size())
	{
		if (sel_item!=m_selectedItem || force_refresh)
		{
			m_selectedItem = sel_item;
			Refresh();

			if (m_event_select_change && m_win_parent)
			{
				(*m_event_select_change)(
					m_win_parent,
					this,
					evSelected,
					sel_item,
					sel_item>=0 ? m_tree_nodes[sel_item].data : CSerializablePtr()
					);
			}
		}
	}
}

/* ------------------------------------------------------------
						OnMouseWheel
   ------------------------------------------------------------ */
void CRawlogTreeView::OnMouseWheel( wxMouseEvent &event )
{
	int x,y;
	GetViewStart(&x,&y);

    if (event.GetWheelRotation()>0)
            y--;
    else    y++;

	int ly = m_tree_nodes.size();

	if (y>=0 && y<ly)
		SetScrollbars(ROW_HEIGHT, ROW_HEIGHT, 50, ly, x,y);
}

/* ------------------------------------------------------------
						OnKey
   ------------------------------------------------------------ */
void CRawlogTreeView::OnKey(wxKeyEvent &event)
{
	int x,y,y0;
	GetViewStart(&x,&y);

	int w,h;
	GetSize(&w,&h);

	int nLinesPerPage = h/ROW_HEIGHT;

	y0=y;
	int ly = m_tree_nodes.size();

	if (event.GetKeyCode()==WXK_UP)
	{
		int sel = m_selectedItem-1;
		if (sel>=0)
			SetSelectedItem(sel);
	}
	else
	if (event.GetKeyCode()==WXK_DOWN)
	{
		int sel = m_selectedItem+1;
		SetSelectedItem(sel);
	}
	else
	if (event.GetKeyCode()==WXK_PAGEDOWN)
	{
		SetSelectedItem(m_selectedItem+nLinesPerPage);
		y+=nLinesPerPage;
		if (y>ly) y=ly;
	}
	else
	if (event.GetKeyCode()==WXK_PAGEUP)
	{
		int sel = m_selectedItem-nLinesPerPage;
		if (sel<=0) sel=0;

		SetSelectedItem(sel);
		y-=nLinesPerPage;
		if (y<0) y=0;
	}
	else
	if (event.GetKeyCode()==WXK_END)
	{
		SetSelectedItem(ly-1);
		y=ly-1;
	}
	else
	if (event.GetKeyCode()==WXK_HOME)
	{
		SetSelectedItem(0);
		y=0;
	}
	else event.Skip();

	if (y>=0 && y<ly && y!=y0)
		SetScrollbars(ROW_HEIGHT, ROW_HEIGHT, 50, ly, x,y);

}

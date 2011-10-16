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

#ifndef CRawlogTreeView_H
#define CRawlogTreeView_H

#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/statline.h>
#include <wx/statbmp.h>
#include <wx/button.h>
#include <wx/scrolwin.h>

#include <mrpt/slam/CRawlog.h>


enum TRawlogTreeViewEvent
{
	evSelected
};

class CRawlogTreeView;

/** The type for event handler
  */
typedef void (*wxRawlogTreeEventFunction)(
	wxWindow*				me,
	CRawlogTreeView*		the_tree,
	TRawlogTreeViewEvent	ev,
	int						item_index,
	const mrpt::utils::CSerializablePtr &item_data);


/** A tree view that represents efficiently all rawlog's items.
 */
class CRawlogTreeView: public wxScrolledWindow
{
public:
	/** Constructor
	  */
    CRawlogTreeView(
		wxWindow* parent=NULL,
		wxWindowID id = -1,
		const wxPoint& pos = wxDefaultPosition,
		const wxSize& size = wxDefaultSize,
		long style = wxVSCROLL,
		const wxString& name = wxT("rawlogTreeView") );

	virtual ~CRawlogTreeView();

	void OnDraw(wxDC& dc);	//!< Draws the rawlog items

    void AssignImageList(wxImageList *imageList)
    {
        m_imageList = imageList;
    }

	/** Sets the rawlog to be rendered in the control (It's kept as a pointer, so the original object cannot be destroyed).
	  *  It automatically calls "reloadFromRawlog".
	  */
	void setRawlogSource( mrpt::slam::CRawlog *rawlog );

	/** Sets the name of the rawlog file, used for the root item */
	void setRawlogName(const std::string &s)
	{
		m_rawlog_name = s;
	}

	/** Reloads from the rawlog: it adapts the size of the scroll window and refresh the view.
	  */
	void reloadFromRawlog( int hint_rawlog_items = -1);

	/** Sets a handler for the event of selected item changes.
	  */
	void ConnectSelectedItemChange( wxRawlogTreeEventFunction func);

	/** This method MUST be called to obtain feedback from events.
	  */
	void setWinParent( wxWindow * win )
	{
		m_win_parent = win;
	}

	/** Returns the time of the first element in the rawlog. */
	mrpt::system::TTimeStamp getFirstTimestamp() const
	{
		return m_rawlog_start;
	}

	void SetSelectedItem( int index , bool force_refresh = false );	//!< Changes the selected item, if different, and raises user callback, if any.
	int GetSelectedItem() const { return m_selectedItem; }

protected:
	mrpt::slam::CRawlog		*m_rawlog;	//!< A reference to the rawlog to be rendered.
	wxImageList				*m_imageList; //!< We own this pointer
	int						m_selectedItem; //!< Selected row, or -1 if none
	std::string				m_rawlog_name;	//!< File name
	wxRawlogTreeEventFunction	m_event_select_change;
	wxWindow				*m_win_parent;

	mrpt::system::TTimeStamp	m_rawlog_start;
	mrpt::system::TTimeStamp	m_rawlog_last;

	struct TNodeData
	{
		TNodeData() : level(0), data(), index(0)
		{ }

		~TNodeData()
		{
		}

		uint8_t				level;		//!< Hierarchy level: 0,1,2.
		mrpt::utils::CSerializablePtr	data;	//!< The object, or NULL
		size_t		index;
	};

	std::vector<TNodeData>	m_tree_nodes;	//!< The nuimber of rows to display for the rawlog, used to compute the height

	/** Returns an icon index depending on the class of the object in the tree view
	  */
	static int iconIndexFromClass( const mrpt::utils::TRuntimeClassId* class_ID );

	static const int ROW_HEIGHT;
	static const int TREE_HORZ_STEPS;


	// Events:
    void OnLeftDown(wxMouseEvent& event);
	void OnMouseWheel( wxMouseEvent &event );
	void OnKey(wxKeyEvent &event);


    DECLARE_DYNAMIC_CLASS(CRawlogTreeView )
    DECLARE_EVENT_TABLE()
};

#endif


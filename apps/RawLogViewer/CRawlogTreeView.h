/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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


/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef CRawlogTreeView_H
#define CRawlogTreeView_H

#include <mrpt/obs/CRawlog.h>
#include <wx/button.h>
#include <wx/menu.h>
#include <wx/notebook.h>
#include <wx/scrolwin.h>
#include <wx/sizer.h>
#include <wx/statbmp.h>
#include <wx/statline.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include <atomic>
#include <functional>

enum TRawlogTreeViewEvent
{
  evSelected
};

class CRawlogTreeView;

/** The type for event handler
 */
using wxRawlogTreeEventFunction = std::function<void(
    wxWindow* me,
    CRawlogTreeView* the_tree,
    TRawlogTreeViewEvent ev,
    int item_index,
    const mrpt::serialization::CSerializable::Ptr& item_data)>;

/** A tree view that represents efficiently all rawlog's items.
 */
class CRawlogTreeView : public wxScrolledWindow
{
 public:
  /** Set to true to avoid the GUI to be refreshed while changing the rawlog
   * in memory */
  static std::atomic_bool RAWLOG_UNDERGOING_CHANGES;

  /** Constructor
   */
  CRawlogTreeView(
      wxWindow* parent = nullptr,
      wxWindowID id = -1,
      const wxPoint& pos = wxDefaultPosition,
      const wxSize& size = wxDefaultSize,
      long style = wxVSCROLL,
      const wxString& name = wxT("rawlogTreeView"));

  /** Draws the rawlog items */
  void OnDraw(wxDC& dc) override;

  void AssignImageList(std::shared_ptr<wxImageList> imageList) { m_imageList = imageList; }
  /** Sets the rawlog to be rendered in the control (It's kept as a pointer,
   * so the original object cannot be destroyed).
   *  It automatically calls "reloadFromRawlog".
   */
  void setRawlogSource(mrpt::obs::CRawlog* rawlog);

  /** Sets the name of the rawlog file, used for the root item */
  void setRawlogName(const std::string& s) { m_rawlog_name = s; }

  /** Reloads from the rawlog: it adapts the size of the scroll window and
   * refresh the view */
  void reloadFromRawlog();

  /** Sets a handler for the event of selected item changes.
   */
  void ConnectSelectedItemChange(const wxRawlogTreeEventFunction& func);

  /** This method MUST be called to obtain feedback from events.
   */
  void setWinParent(wxWindow* win) { m_win_parent = win; }
  /** Returns the time of the first element in the rawlog. */
  mrpt::system::TTimeStamp getFirstTimestamp() const { return m_rawlog_start; }

  /** Changes the selected item, if different, and raises user callback, if
   * any. */
  void SetSelectedItem(int index, bool force_refresh = false);
  int GetSelectedItem() const { return m_selectedItem; }

  bool m_is_thumb_tracking = false;

  void ScrollToPercent(double pc);

  struct TNodeData
  {
    TNodeData() = default;

    /** Hierarchy level: 0,1,2. */
    uint8_t level = 0;

    /** The object, or nullptr */
    mrpt::serialization::CSerializable::Ptr data;

    size_t index = 0;

    std::optional<mrpt::Clock::time_point> timestamp;
    std::optional<std::string> sensorLabel;
  };

  size_t m_firstVisibleItem = 0, m_lastVisibleItem = 0;
  size_t getTotalTreeNodes() const { return m_tree_nodes.size(); }

  const std::deque<TNodeData>& treeNodes() const { return m_tree_nodes; }

  bool isItemIndexVisible(size_t idx) const;

 protected:
  void OnDrawImpl(wxDC& dc);

  /** A reference to the rawlog to be rendered. */
  mrpt::obs::CRawlog* m_rawlog{nullptr};
  /** We own this pointer */
  std::shared_ptr<wxImageList> m_imageList;
  /** Selected row, or -1 if none */
  int m_selectedItem{-1};
  /** File name */
  std::string m_rawlog_name;
  wxRawlogTreeEventFunction m_event_select_change;
  wxWindow* m_win_parent{nullptr};

  std::string m_last_exported_rawlog_file;

  mrpt::system::TTimeStamp m_rawlog_start;
  mrpt::system::TTimeStamp m_rawlog_last;

  /** The nuimber of rows to display for the rawlog, used to compute the
   * height */
  std::deque<TNodeData> m_tree_nodes;

  /** Returns an icon index depending on the class of the object in the tree
   * view
   */
  static int iconIndexFromClass(const mrpt::rtti::TRuntimeClassId* class_ID);

  static const int ROW_HEIGHT;
  static const int TREE_HORZ_STEPS;

  wxMenu m_contextMenu;

  // Events:
  void OnLeftDown(wxMouseEvent& event);
  void OnRightDown(wxMouseEvent& event);
  void OnMouseWheel(wxMouseEvent& event);
  void OnKey(wxKeyEvent& event);

  void onScrollThumbTrack(wxScrollWinEvent& ev);
  void onScrollThumbRelease(wxScrollWinEvent& ev);

  void onMnuExportToOtherFile(wxCommandEvent&);
  void onMnuAppendSaveFile(wxCommandEvent&);

  DECLARE_DYNAMIC_CLASS(CRawlogTreeView)
  DECLARE_EVENT_TABLE()
};

#endif

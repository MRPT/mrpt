/////////////////////////////////////////////////////////////////////////////
// Name:        filebrws.h
// Purpose:     A file browser widget with tree and/or list control views
// Author:      John Labenski
// Created:     07/01/02
// Copyright:   John Labenski, 2002
// License:     wxWidgets
/////////////////////////////////////////////////////////////////////////////

#ifndef __WX_FILEBROWSER_H__
#define __WX_FILEBROWSER_H__

#include "wx/listctrl.h"
#include "wx/dirctrl.h"
#include "wx/filedlg.h"
#include "wx/textdlg.h"
#include "wx/generic/filedlgg.h"
#include "wx/things/thingdef.h"

class WXDLLEXPORT wxCheckBox;
class WXDLLEXPORT wxComboBox;
class WXDLLEXPORT wxTreeEvent;
class WXDLLEXPORT wxSplitterWindow;
class WXDLLEXPORT wxGenericDirCtrl;
class WXDLLEXPORT wxListCtrl;
class WXDLLEXPORT wxListEvent;
class WXDLLEXPORT wxToolBar;
class WXDLLEXPORT wxBitmapButton;
class WXDLLEXPORT wxConfigBase;
class WXDLLEXPORT wxFileCtrl;
class WXDLLEXPORT wxFileName;

class WXDLLIMPEXP_THINGS wxFileBrowser;

#include "wx/dynarray.h"
WX_DECLARE_OBJARRAY_WITH_DECL(wxFileData, wxArrayFileData, class WXDLLIMPEXP_THINGS);

//----------------------------------------------------------------------------
// MultilineTextDialog : wxTextEntryDialog for multiple lines
//----------------------------------------------------------------------------

class WXDLLIMPEXP_THINGS MultilineTextDialog : public wxTextEntryDialog
{
public:
    MultilineTextDialog(wxWindow *parent,
                        const wxString& message,
                        const wxString& caption = wxGetTextFromUserPromptStr,
                        const wxString& value = wxEmptyString,
                        long  style = 0,
                        const wxPoint& pos = wxDefaultPosition);
};

//----------------------------------------------------------------------------
// wxFileBrowserEvent : events for the wxFileBrowser
//----------------------------------------------------------------------------
// wxEVT_FILEBROWSER_FILE_SELECTED  - a file has been selected (single click)
// wxEVT_FILEBROWSER_FILE_ACTIVATED - a file has been double clicked or enter pressed
// wxEVT_FILEBROWSER_DIR_SELECTED   - a dir has been selected (single click)
// wxEVT_FILEBROWSER_DIR_ACTIVATED  - a dir has been double clicked or enter pressed

BEGIN_DECLARE_EVENT_TYPES()
    DECLARE_EXPORTED_EVENT_TYPE(WXDLLIMPEXP_THINGS, wxEVT_FILEBROWSER_FILE_SELECTED,  wxEVT_USER_FIRST + 1000)
    DECLARE_EXPORTED_EVENT_TYPE(WXDLLIMPEXP_THINGS, wxEVT_FILEBROWSER_FILE_ACTIVATED, wxEVT_USER_FIRST + 1001)
    DECLARE_EXPORTED_EVENT_TYPE(WXDLLIMPEXP_THINGS, wxEVT_FILEBROWSER_DIR_SELECTED,   wxEVT_USER_FIRST + 1002)
    DECLARE_EXPORTED_EVENT_TYPE(WXDLLIMPEXP_THINGS, wxEVT_FILEBROWSER_DIR_ACTIVATED,  wxEVT_USER_FIRST + 1003)
END_DECLARE_EVENT_TYPES()

class WXDLLIMPEXP_THINGS wxFileBrowserEvent : public wxCommandEvent
{
public:
    wxFileBrowserEvent( wxEventType commandType = wxEVT_nullptr,
                        wxFileBrowser *fileBrowser = nullptr,
                        wxWindowID id = wxID_ANY );

    wxFileBrowserEvent( const wxFileBrowserEvent &event ) : wxCommandEvent(event) {}

    // Get the full path + filename
    wxString GetFilePath() const { return GetString(); }
    void SetFilePath(const wxString &filepath) { SetString(filepath); }

    virtual wxEvent *Clone() const { return new wxFileBrowserEvent(*this); }

private:
    DECLARE_ABSTRACT_CLASS(wxFileBrowserEvent)
};

typedef void (wxEvtHandler::*wxFileBrowserEventFunction)(wxFileBrowserEvent&);

#define wxFileBrowserEventHandler(func) \
    (wxObjectEventFunction)(wxEventFunction)wxStaticCastEvent(wxFileBrowserEventFunction, &func)

#define wx__DECLARE_FILEBROWSEREVT(evt, id, fn) wx__DECLARE_EVT1( evt, id, wxFileBrowserEventHandler(fn))

#define EVT_FILEBROWSER_FILE_SELECTED(id, fn)  wx__DECLARE_FILEBROWSEREVT( wxEVT_FILEBROWSER_FILE_SELECTED,  id, fn )
#define EVT_FILEBROWSER_FILE_ACTIVATED(id, fn) wx__DECLARE_FILEBROWSEREVT( wxEVT_FILEBROWSER_FILE_ACTIVATED, id, fn )
#define EVT_FILEBROWSER_DIR_SELECTED(id, fn)   wx__DECLARE_FILEBROWSEREVT( wxEVT_FILEBROWSER_DIR_SELECTED,   id, fn )
#define EVT_FILEBROWSER_DIR_ACTIVATED(id, fn)  wx__DECLARE_FILEBROWSEREVT( wxEVT_FILEBROWSER_DIR_ACTIVATED,  id, fn )

//----------------------------------------------------------------------------
// wxFileBrowser
//----------------------------------------------------------------------------

enum wxFileBrowserStyles_Type
{
    // note: these are wxListCtrl styles to allow normal wxWindow styles to work

    wxFILEBROWSER_TREE       = wxLC_SORT_DESCENDING,  // treectrl view
    wxFILEBROWSER_LIST       = wxLC_LIST,             // listctrl view
    wxFILEBROWSER_DETAILS    = wxLC_REPORT,           // listctrl details view
    wxFILEBROWSER_SMALL_ICON = wxLC_SMALL_ICON,       // listctrl icon view
    wxFILEBROWSER_LARGE_ICON = wxLC_ICON,             // NOT IMPL listctrl large icon
    wxFILEBROWSER_PREVIEW    = wxLC_SORT_ASCENDING,   // NOT implemented

    wxFILEBROWSER_SPLIT_VERTICAL = wxLC_NO_HEADER,    // tree and listctrl are
                                                      // split vertically else horizontal

    wxFILEBROWSER_SHOW_FOLDERS = wxLC_NO_SORT_HEADER, // when showing listview also show
                                                      // the folders in the treectrl

    wxFILEBROWSER_VIEW_MASK = wxFILEBROWSER_TREE|wxFILEBROWSER_LIST|wxFILEBROWSER_DETAILS|wxFILEBROWSER_SMALL_ICON|wxFILEBROWSER_LARGE_ICON|wxFILEBROWSER_PREVIEW,
    wxFILEBROWSER_STYLE_MASK = wxFILEBROWSER_VIEW_MASK|wxFILEBROWSER_SPLIT_VERTICAL|wxFILEBROWSER_SHOW_FOLDERS
};

class WXDLLIMPEXP_THINGS wxFileBrowser : public wxControl
{
public :
    wxFileBrowser() : wxControl() { Init(); }

    wxFileBrowser( wxWindow* parent, const wxWindowID id,
                   const wxString& dir = wxDirDialogDefaultFolderStr,
                   const wxPoint& pos = wxDefaultPosition,
                   const wxSize& size = wxDefaultSize,
                   long style = wxFILEBROWSER_DETAILS,
                   const wxString& filter = wxFileSelectorDefaultWildcardStr,
                   int defaultFilter = 0,
                   const wxString& name = wxT("wxFileBrowser")) : wxControl()
    {
        Init();
        Create(parent, id, dir, pos, size, style, filter, defaultFilter, name);
    }

    virtual ~wxFileBrowser();

    bool Create( wxWindow* parent, const wxWindowID id,
                 const wxString& dir = wxDirDialogDefaultFolderStr,
                 const wxPoint& pos = wxDefaultPosition,
                 const wxSize& size = wxDefaultSize,
                 long style = wxFILEBROWSER_DETAILS,
                 const wxString& filter = wxFileSelectorDefaultWildcardStr,
                 int defaultFilter = 0,
                 const wxString& name = wxT("wxFileBrowser") );

    // Get the current dir (not file), optionally add a trailing platform dependent '/' or '\'
    wxString GetPath(bool add_wxFILE_SEP_PATH = false) const;
    // Go to a directory, returns sucess
    bool SetPath(const wxString &dirName);

    // go to a dir or send an EVT_FILEBROWSER_FILE_ACTIVATED if a filename
    bool OpenFilePath(const wxString &filePath);

    // Go to a higher directory, returns sucess
    bool CanGoUpDir() const;
    bool GoUpDir();

    // Go to your "Home" folder "~/" in unix, "My Documents" in MSW
    bool GoToHomeDir();

    // Go forwards and backwards through the recent dir history
    bool CanGoPathHistoryForward();
    bool CanGoPathHistoryBackward();
    bool GoPathHistoryForward();
    bool GoPathHistoryBackward();
    // Add a new path to the history paths at the current index
    void AddPathHistory(const wxString& path);

    // Set the file filter to one of the filter combobox items
    bool SetFilter(int comboItem);
    // Set all the file filters, deleting previous and select one
    bool SetFilters(const wxString &filters, int select = 0);
    // Add or set the file filter, "All Files (*)|*", it must have a "|" in it
    bool AddFilter(const wxString &filter);
    // Get the current file filter
    wxString GetFilter() const { return m_filter; }
    // Get the wild card used for the filter
    wxString GetWild() const { return m_filter.AfterLast(wxT('|')); }

    // Set how the files are displayed - see enum wxFileBrowserStyles_Type
    void SetBrowserStyle(long style);
    long GetBrowserStyle() const { return m_browser_style; }
    bool HasBrowserStyle(int style_mask) const { return (m_browser_style & style_mask) != 0; }

    // Show or hide hidden files
    void ShowHidden(bool show_hidden);
    bool GetShowHidden() const { return m_show_hidden; }

    // When showing the files in a listctrl also show the folders in the treectrl
    //   also don't let them unsplit it
    void ShowFolders(bool show_folders);
    bool GetShowFolders() const { return HasBrowserStyle(wxFILEBROWSER_SHOW_FOLDERS); }

    // When splitting, split vertically or horizontally
    void SplitVertical(bool split_vertically);
    bool GetSplitVertical() const { return HasBrowserStyle(wxFILEBROWSER_SPLIT_VERTICAL); }

    // -----------------------------------------------------------------------
    // implementation

    // utility function, returns the dir part of the filepath w/ trailing wxFILE_SEP_PATH
    bool GetPathFromFilePath(const wxString &filepath, wxString &path) const;

    // Delete all selected items in the wxFileCtrl
    bool DeleteSelectedListItems(bool ask_ok = true);
    // Store a list of selected items that you'll copy/cut when you paste them
    bool CopyCutSelectedListItems(bool copy_them);
    // Paste the stored CopyCutSelectedListItems - based on CopyCutSelectedListItems list
    bool PasteCopyCutSelectedListItems();

    // Get a list of all the selected items in the list control
    wxArrayInt GetSelectedListItems() const;
    // Get the wxFileData items that are selected in the list control
    wxArrayFileData GetSelectedListFileData() const;
    // Get the currently focused list item or nullptr if none selected
    wxFileData *GetFocusedListItem() const;

    // Create a wxFileData from a wxFileName
    wxFileData CreateFileData(const wxFileName& fileName) const;

    // Get the last or currently focused path + filename
    wxString GetLastFocusedFilePath();

    // Show a simple dialog that contains the properties of the file/dir
    void ShowPropertiesDialog(const wxFileData &fileData) const;

    // returns a string with the name of a program to run the file
    wxString GetOpenWithFileCmd(wxFileData* fd) const;

    // Get a pointer to the path history combo, can change its contents
    wxComboBox *GetPathCombo() const { return m_pathCombo; }
    // Get a pointer to the filter combo
    //   don't delete selections less than the # of filters passed in
    //   ie. check for items with GetClientData() !NULL
    wxComboBox *GetFilterCombo() const { return m_filterCombo; }

    // Can this file be read/opened?
    bool CanRead(const wxString& filePath) const;
    // Can this file be written to, deleted, moved, cut...
    bool CanWrite(const wxString& filePath) const;

    // Update the menu/toolbar items
    void UpdateMenu(wxMenu *menu);
    void UpdateToolBar(wxToolBar *toolBar);
    // Update the state of the toolbar and menu items
    void UpdateItems();

#if wxUSE_CONFIG
    // Load the recent paths/filters, max = 20
    void LoadConfig(wxConfigBase& config,
                    bool paths=true, bool filters=true,
                    const wxString &configPath = wxT("/wxFileBrowser"));
    // Save the recent paths, filters, if n_xxx < 0 then don't save it
    void SaveConfig(wxConfigBase& config,
                    int n_paths=10, int n_filters=10,
                    const wxString &configPath = wxT("/wxFileBrowser"));
#endif // wxUSE_CONFIG

protected :
    void OnSize( wxSizeEvent& event );
    void DoSize();

    virtual wxSize DoGetBestSize() const;

    // toolbar tools events
    void OnViewButtons(wxCommandEvent &event);
    void OnPathCombo(wxCommandEvent &event);
    void OnPathComboEnter(wxCommandEvent &event);
    void OnFilterCombo(wxCommandEvent &event);
    void OnFilterComboEnter(wxCommandEvent &event);

    // wxDirCtrl events - a wxTreeCtrl
    void OnTreeItemSelection(wxTreeEvent &event);
    void OnTreeItemActivation(wxTreeEvent &event);
    void OnTreeRightClick(wxTreeEvent& event);

    // wxFileCtrl events - a wxListCtrl
    void OnListColClick(wxListEvent &event);
    void OnListItemActivated(wxListEvent &event);
    void OnListItemSelected(wxListEvent &event);
    void OnListRightClick(wxListEvent &event);

    void OnTreeMenu(wxCommandEvent &event);
    void OnListMenu(wxCommandEvent &event);

    void OnIdle( wxIdleEvent &event );

    //for delayed set path from combo
    void OnSetPath( wxCommandEvent &event );
    void OnSetFilter( wxCommandEvent &event );

    // (re)inserts the item at pos, deleting it if it existed after pos
    //    keeps recent items at top
    bool InsertComboItem(wxComboBox *combo, const wxString &item, int pos = 0) const;

    // Send an event, returns false if event.Veto() called
    bool DoSendEvent(wxFileBrowserEvent &event) const;

    int FBStyleToLCStyle(int fb_style) const; // wxFileBrowserStyles_Type to wxLC_XXX
    int FBStyleToMenuID(int fb_style) const;  // wxFileBrowserStyles_Type menu id
    int MenuIDToFBStyle(int menuID) const;    // menu id to wxFileBrowserStyles_Type

    // Windows
    wxToolBar        *m_viewToolBar;
    wxToolBar        *m_pathToolBar;

    wxBitmapButton   *m_viewButton;
    wxComboBox       *m_filterCombo;

    wxComboBox       *m_pathCombo;

    wxSplitterWindow *m_splitterWin;
    wxGenericDirCtrl *m_dirCtrl;
    wxFileCtrl       *m_fileCtrl;

    wxMenu           *m_listMenu;   // popup menu in listctrl
    wxMenu           *m_treeMenu;   // popup menu in treectrl
    wxMenu           *m_viewMenu;   // popup menu in for changing view

    // data
    wxString m_filter;              // current filter
    wxString m_path;                // current path
    wxString m_lastFocusedFilePath; // path + filename of last focused item

    wxArrayString m_pathHistory;    // recently used paths
    int m_path_history_index;       // current index in recently used paths

    wxArrayFileData m_copycutFiles; // list of names when copying or cutting
    bool m_last_copy;               // last CopyCutSelectedListItems was a copy, else cut

    int m_init_filters;             // # of filters initially passed in

    bool m_ignore_tree_event;       // temporarily ignore m_dirCtrl events
    long m_browser_style;

    int m_filterComboSelection;     // last selection of the filter/path combo
    int m_pathComboSelection;

    bool m_show_hidden;             // show hidden files

private :
    void Init();
    DECLARE_EVENT_TABLE()
    DECLARE_DYNAMIC_CLASS(wxFileBrowser)
};

#endif // __WX_FILEBROWSER_H__

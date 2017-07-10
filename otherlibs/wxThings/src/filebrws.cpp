/////////////////////////////////////////////////////////////////////////////
// Name:        filebrws.cpp
// Author:      John Labenski
// Created:     07/01/02
// Copyright:   John Labenski, 2002
// License:     wxWidgets
/////////////////////////////////////////////////////////////////////////////

// For compilers that support precompilation, includes "wx/wx.h".
#include "wx/wxprec.h"

#ifdef __BORLANDC__
    #pragma hdrstop
#endif

#ifndef WX_PRECOMP
    #include "wx/wx.h"
#endif // WX_PRECOMP

#include "wx/mimetype.h"
#include "wx/tglbtn.h"     // for EVT_TOGGLE_BUTTON
#include "wx/splitter.h"
#include "wx/imaglist.h"
#include "wx/confbase.h"
#include "wx/filename.h"
#include "wx/dir.h"
#include "wx/artprov.h"
#include "wx/image.h"      // wxInitAllImageHandlers
#include "wx/renderer.h"
#include "wx/file.h"
#include "wx/txtstrm.h"
#include "wx/wfstream.h"

#include "wx/filefn.h"     // wxStat

#include "wx/things/filebrws.h"

#include "wx/arrimpl.cpp"
WX_DEFINE_OBJARRAY(wxArrayFileData);

// #include "wx/stedit/stedit.h"

// defined in src/generic/dirctrlg.cpp
extern size_t wxGetAvailableDrives(wxArrayString &paths, wxArrayString &names, wxArrayInt &icon_ids);

#define BORDER    5
#define MIN_SPLIT 8

//------------------------------------------------------------------------------
// constants
//------------------------------------------------------------------------------

enum
{
    ID_wxFILEBROWSER_PATH_COMBO = 100,
    ID_wxFILEBROWSER_PATH_BUTTON,
    ID_wxFILEBROWSER_DIRCTRL,
    ID_wxFILEBROWSER_LISTCTRL,
    ID_wxFLEBROWSER_VIEW_TOOLBAR,
    ID_wxFLEBROWSER_PATH_TOOLBAR,

    ID_wxFILEBROWSER_FILTER_COMBO,

    ID_wxFILEBROWSER_VIEW_BUTTON,

    ID_wxFILEBROWSER_VIEW_TREE,
    ID_wxFILEBROWSER_VIEW_LIST,
    ID_wxFILEBROWSER_VIEW_DETAILS,
    ID_wxFILEBROWSER_VIEW_SMALL_ICON,
    ID_wxFILEBROWSER_VIEW_LARGE_ICON,
    ID_wxFILEBROWSER_VIEW_PREVIEW,

    ID_wxFILEBROWSER_GO_BACK,
    ID_wxFILEBROWSER_GO_FORWARD,
    ID_wxFILEBROWSER_GO_UP,
    ID_wxFILEBROWSER_GO_HOME,
    ID_wxFILEBROWSER_REFRESH,

    ID_wxFILEBROWSER_TREE_MENU_PROPERITES,

    ID_wxFILEBROWSER_LIST_MENU_VIEW_FILE,
    ID_wxFILEBROWSER_LIST_MENU_OPEN_WITH,
    ID_wxFILEBROWSER_LIST_MENU_NEW_FOLDER,
    ID_wxFILEBROWSER_LIST_MENU_DELETE,
    ID_wxFILEBROWSER_LIST_MENU_RENAME,
    ID_wxFILEBROWSER_LIST_MENU_PROPERITES,

    ID_wxFILEBROWSER_LIST_MENU_OPTIONS,
    ID_wxFILEBROWSER_SHOW_HIDDEN,
    ID_wxFILEBROWSER_SHOW_FOLDERS,
    ID_wxFILEBROWSER_SPLIT_VERTICAL,

    ID_wxFILEBROWSER_COMBOSETPATH,  // for delayed setting path to get out of combo event for Gtk
    ID_wxFILEBROWSER_COMBOSETFILTER
};

int wxFileBrowser::FBStyleToLCStyle(int s) const // wxFileBrowserStyles_Type to wxLC_XXX
{
    return
        (s)&wxFILEBROWSER_TREE       ? wxLC_REPORT :
        (s)&wxFILEBROWSER_LIST       ? wxLC_LIST :
        (s)&wxFILEBROWSER_DETAILS    ? wxLC_REPORT :
        (s)&wxFILEBROWSER_SMALL_ICON ? wxLC_SMALL_ICON :
        (s)&wxFILEBROWSER_LARGE_ICON ? wxLC_ICON :
        (s)&wxFILEBROWSER_PREVIEW    ? wxLC_ICON : wxLC_REPORT;
}
int wxFileBrowser::FBStyleToMenuID(int s) const // wxFileBrowserStyles_Type menu id
{
    return
        (s)&wxFILEBROWSER_TREE       ? ID_wxFILEBROWSER_VIEW_TREE :
        (s)&wxFILEBROWSER_LIST       ? ID_wxFILEBROWSER_VIEW_LIST :
        (s)&wxFILEBROWSER_DETAILS    ? ID_wxFILEBROWSER_VIEW_DETAILS :
        (s)&wxFILEBROWSER_SMALL_ICON ? ID_wxFILEBROWSER_VIEW_SMALL_ICON :
        (s)&wxFILEBROWSER_LARGE_ICON ? ID_wxFILEBROWSER_VIEW_LARGE_ICON :
        (s)&wxFILEBROWSER_PREVIEW    ? ID_wxFILEBROWSER_VIEW_PREVIEW : ID_wxFILEBROWSER_VIEW_DETAILS;
}
int wxFileBrowser::MenuIDToFBStyle(int id) const    // menu id to wxFileBrowserStyles_Type
{
    return
        (id)==ID_wxFILEBROWSER_VIEW_TREE       ? wxFILEBROWSER_TREE :
        (id)==ID_wxFILEBROWSER_VIEW_LIST       ? wxFILEBROWSER_LIST :
        (id)==ID_wxFILEBROWSER_VIEW_DETAILS    ? wxFILEBROWSER_DETAILS :
        (id)==ID_wxFILEBROWSER_VIEW_SMALL_ICON ? wxFILEBROWSER_SMALL_ICON :
        (id)==ID_wxFILEBROWSER_VIEW_LARGE_ICON ? wxFILEBROWSER_LARGE_ICON :
        (id)==ID_wxFILEBROWSER_VIEW_PREVIEW    ? wxFILEBROWSER_PREVIEW : wxFILEBROWSER_DETAILS;
}

//============================================================================
// Button Icons
//============================================================================
/* XPM */
/*
static const char *view_xpm_data[] = {
// columns rows colors chars-per-pixel
"16 16 4 1",
"a c Black",
"b c #0000FF",
"c c #FFFFFF",
"d c #808080",
// pixels
"aaaaaaaaaaaaaaaa",
"abbbbbbbbbbbbbba",
"abbbbbbbbbbbbbba",
"aaaaaaaaaaaaaaaa",
"adddddddddddddda",
"acccccccccccccca",
"acbbccccbbccccca",
"acbbaaacbbaaacca",
"acccccccccccccca",
"acbbccccbbccccca",
"acbbaaacbbaaacca",
"acccccccccccccca",
"acbbccccbbccccca",
"acbbaaacbbaaacca",
"acccccccccccccca",
"aaaaaaaaaaaaaaaa"};

static const char *details_xpm_data[]={
"16 15 4 1",
"# c None",
"a c #000000",
"b c #000080",
". c #c0c0c0",
"################",
"###aaaa#aaa#aaa#",
"################",
"bbbbbbbbbbbbbbbb",
"################",
"b##aaaa#aaa#aaa#",
"################",
"b##aaaa#aaa#aaa#",
"################",
"b##aaaa#aaa#aaa#",
"################",
"b##aaaa#aaa#aaa#",
"################",
"b##aaaa#aaa#aaa#",
"################"};

static const char *list_xpm_data[]={
"16 15 4 1",
". c None",
"b c #000000",
"# c #000080",
"a c #ffffff",
"................",
"##......##......",
"#a#.bbb.#a#.bbb.",
"###.....###.....",
"................",
"................",
"##......##......",
"#a#.bbb.#a#.bbb.",
"###.....###.....",
"................",
"................",
"##......##......",
"#a#.bbb.#a#.bbb.",
"###.....###.....",
"................"};

static const char *large_icons_xpm_data[]={
"16 15 4 1",
". c None",
"b c #000000",
"# c #000080",
"a c #ffffff",
"................",
"..###...........",
"..#a##..........",
"..#aa#..........",
"..#aa#..........",
"..####..........",
"................",
".bbbbbb...###...",
"..........#a##..",
"..........#aa#..",
"..........#aa#..",
"..........####..",
"................",
".........bbbbbb.",
"................"};

static const char *small_icons_xpm_data[]={
"16 15 4 1",
". c None",
"b c #000000",
"# c #000080",
"a c #ffffff",
"................",
".##.............",
".#a#.bbb........",
".###............",
"................",
"................",
"........##......",
"........#a#.bbb.",
"........###.....",
"................",
"................",
"...##...........",
"...#a#.bbb......",
"...###..........",
"................"};

static const char *tree_xpm_data[]={
"16 15 4 1",
". c None",
"# c #000000",
"a c #ffff00",
"b c #ffffff",
".#aa#...........",
".#a##...........",
"..#.............",
"..####aa#.......",
".....#a##.......",
"......#.........",
"......####b#b#..",
"......#..#b#b#..",
"......#.........",
"......####b#b#..",
"......#..#b#b#..",
"......#.........",
".......###b#b#..",
".........#b#b#..",
"................"};

static const char *preview_xpm_data[]={
"16 15 14 1",
". c None",
"k c #000080",
"l c #0058c0",
"g c #0080ff",
"e c #585858",
"c c #808080",
"b c #89562f",
"f c #a0a0a0",
"h c #a86a3a",
"j c #bd7741",
"i c #d48549",
"# c #ff0000",
"a c #ffff00",
"d c #ffffff",
".......###......",
".a..a..bc.#.....",
"..aa...bdc......",
"..aa..cbddc.....",
".a..acdbdddc....",
"....cddbddddc...",
"....eddbddddf...",
"....eddbdddddf..",
".....edbddddff..",
"..g.hiijiiih.g..",
".gggghiiiihgggg.",
".kllllhhhhggllk.",
".llllllllllllll.",
".kklkllklklklkk.",
"................"};

static const char *hidden_xpm_data[] = {
"16 15 7 1",
"  c None",
"a c Black",
"b c #EFBE1B",
"d c #F6F0B4",
"e c #FFEF7D",
"f c #FBD915",
"g c #8E8220",
"                ",
"        aaa     ",
"       addfa    ",
"      adffffa   ",
"      adfdadba  ",
"      affaaabga ",
"       aedadbga ",
"      adfbbbgga ",
"     adfbaggga  ",
"    adfba aaa   ",
"   adfba        ",
"  adfbgga       ",
"  abbaga        ",
"   aa a         ",
"                "};
*/

wxBitmap GetBitmapFromIconId(int imageId)
{
    return wxTheFileIconsTable->GetSmallImageList()->GetBitmap(imageId);
}

// ----------------------------------------------------------------------------
// CopyDir - copy a full directory recursively, returns # files copied
// code taken and modified from wxWidgets forum, by "Tyler" - public domain
// ----------------------------------------------------------------------------

int CopyDir(const wxString& fromDir, const wxString& toDir)
{
    wxString from(fromDir), to(toDir);

    if (!from.length() || !to.length()) return false;

    // append a slash if there is not one (for easier parsing)
    // because who knows what people will pass to the function.
    if (to[to.length()-1] != wxFILE_SEP_PATH)
        to += wxFILE_SEP_PATH;
    if (from[from.length()-1] != wxFILE_SEP_PATH)
        from += wxFILE_SEP_PATH;

    // first make sure that the source dir exists
    if(!wxDir::Exists(from))
    {
        wxLogError(from + wxT(" does not exist. Can not copy directory."));
        return 0;
    }

    if (!wxDir::Exists(to))
        wxMkdir(to);

    wxDir dir(from);
    wxString filename;
    int count = 0;

    if (dir.GetFirst(&filename))
    {
        do {
            if (wxDirExists(from + filename))
            {
                wxMkdir(to + filename);
                count += CopyDir(from + filename, to + filename);
            }
            else
            {
                wxCopyFile(from + filename, to + filename);
                count++;
            }
        }
        while (dir.GetNext(&filename));
    }
    return count;
}

//------------------------------------------------------------------------------
// MultilineTextDialog - makes the wxTextEntryDialog multiline
//------------------------------------------------------------------------------

MultilineTextDialog::MultilineTextDialog(wxWindow *parent,
                                         const wxString& message,
                                         const wxString& caption,
                                         const wxString& value,
                                         long style,
                                         const wxPoint& pos)
                    :wxTextEntryDialog(parent, message, caption, value,
                          style|wxTextEntryDialogStyle|wxTE_MULTILINE, pos)
{
    int height = m_textctrl->GetSize().y;
    m_textctrl->SetSize(300, 100);
    height = (100-height > 0) ? 100 - height : 0;
    wxSize size(GetSize());
    SetSize(size.x, size.y+height);
    Layout();
    //m_textctrl->SetEditable(false);
}

//------------------------------------------------------------------------------
// MultilineTextDialog
//------------------------------------------------------------------------------
#define ID_OPENWITH_BROWSE   5
#define ID_OPENWITH_TEXTCTRL 6

class OpenWithDialog : public wxDialog
{
public:
    OpenWithDialog(wxWindow* parent, wxWindowID winId,
                   const wxFileData& fileData,
                   const wxString& caption = wxT("Open With"),
                   const wxString& openCommand = wxEmptyString,
                   long style = wxDEFAULT_DIALOG_STYLE,
                   const wxPoint& pos = wxDefaultPosition) : wxDialog(), m_fileData(fileData)
    {
        m_textCtrl = nullptr;
        Create(parent, winId, fileData, caption, openCommand, style, pos);
    }
    bool Create(wxWindow* parent, wxWindowID winId,
                const wxFileData& fileData,
                const wxString& caption = wxT("Open With"),
                const wxString& openCommand = wxEmptyString,
                long style = wxDEFAULT_DIALOG_STYLE,
                const wxPoint& pos = wxDefaultPosition);

    wxString GetOpenCommand() const { return m_command; }

protected:
    void OnButton(wxCommandEvent& event);

    wxString m_command;
    wxFileData m_fileData;
    wxTextCtrl *m_textCtrl;
    DECLARE_EVENT_TABLE();
};

BEGIN_EVENT_TABLE(OpenWithDialog, wxDialog)
    EVT_BUTTON(-1, OpenWithDialog::OnButton)
END_EVENT_TABLE()

bool OpenWithDialog::Create(wxWindow *parent, wxWindowID winId,
                            const wxFileData& fileData,
                            const wxString& caption,
                            const wxString& openCommand,
                            long style, const wxPoint& pos)
{
    if (!wxDialog::Create(parent, winId, caption, pos, wxDefaultSize, style))
        return false;

    m_fileData = fileData;
    wxFileName fileName(fileData.GetFilePath());
    wxString openCmd = openCommand;
    wxString description(wxT("Unknown file type"));
    if (openCmd.IsEmpty())
    {
        wxFileType *fileType = wxTheMimeTypesManager->GetFileTypeFromExtension(fileName.GetExt());
        if (fileType)
        {
            fileType->GetDescription(&description);
            openCmd = fileType->GetOpenCommand(fileName.GetFullPath());
        }
    }

    wxBoxSizer *rootSizer = new wxBoxSizer( wxVERTICAL );

    wxStaticText *labelText = new wxStaticText( this, wxID_ANY, wxT("Choose the program to open the file"), wxDefaultPosition, wxDefaultSize, 0 );
    rootSizer->Add( labelText, 0, wxGROW|wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    wxFlexGridSizer *iconSizer = new wxFlexGridSizer( 2, 0, 0 );
    iconSizer->AddGrowableCol( 1 );

    wxBitmap iconBitmap = GetBitmapFromIconId(m_fileData.GetImageId());
    wxStaticBitmap *staticBitmap = new wxStaticBitmap( this, wxID_ANY, iconBitmap, wxDefaultPosition, wxDefaultSize );
    iconSizer->Add( staticBitmap, 0, wxALIGN_CENTER|wxALL, 5 );

    wxStaticText *nameText = new wxStaticText( this, wxID_ANY, fileName.GetFullPath(), wxDefaultPosition, wxDefaultSize, 0 );
    iconSizer->Add( nameText, 0, wxGROW|wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    iconSizer->AddSpacer(5);

    wxStaticText *descripText = new wxStaticText( this, wxID_ANY, description, wxDefaultPosition, wxDefaultSize, 0 );
    iconSizer->Add( descripText, 0, wxGROW|wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    rootSizer->Add( iconSizer, 0, wxGROW|wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    wxStaticBox *staticBox = new wxStaticBox( this, wxID_ANY, wxT("Program") );
    wxStaticBoxSizer *staticBoxSizer = new wxStaticBoxSizer( staticBox, wxVERTICAL );

    m_textCtrl = new wxTextCtrl( this, ID_OPENWITH_TEXTCTRL, openCmd, wxDefaultPosition, wxSize(400,-1), 0 );
    staticBoxSizer->Add( m_textCtrl, 0, wxGROW|wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    wxFlexGridSizer *browseSizer = new wxFlexGridSizer( 2, 0, 0 );
    browseSizer->AddGrowableCol( 1 );

    wxButton *browseButton = new wxButton( this, ID_OPENWITH_BROWSE, wxT("Browse..."), wxDefaultPosition, wxDefaultSize, 0 );
    browseSizer->Add( browseButton, 0, wxALIGN_CENTER|wxALL, 5 );

    //wxCheckBox *item10 = new wxCheckBox( this, wxID_ANY, wxT("Always use selected program to open this kind of file"), wxDefaultPosition, wxDefaultSize, 0 );
    //browseSizer->Add( item10, 0, wxGROW|wxALIGN_CENTER_VERTICAL|wxALL, 5 );
    browseSizer->AddSpacer(5);
    staticBoxSizer->Add( browseSizer, 0, wxGROW|wxALIGN_CENTER|wxALL, 0 );

    rootSizer->Add( staticBoxSizer, 0, wxGROW|wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    wxBoxSizer *buttonSizer = new wxBoxSizer( wxHORIZONTAL );

    wxButton *okButton = new wxButton( this, wxID_OK, wxT("OK"), wxDefaultPosition, wxDefaultSize, 0 );
    okButton->SetDefault();
    buttonSizer->Add( okButton, 0, wxALIGN_CENTER|wxALL, 5 );

    wxButton *cancelButton = new wxButton( this, wxID_CANCEL, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
    buttonSizer->Add( cancelButton, 0, wxALIGN_CENTER|wxALL, 5 );

    rootSizer->Add( buttonSizer, 0, wxALIGN_CENTER|wxALL, 5 );

    SetMinSize(wxSize(400, 300));
    SetSizer( rootSizer );
    rootSizer->SetSizeHints( this );
    return true;
}

void OpenWithDialog::OnButton(wxCommandEvent& event)
{
    switch (event.GetId())
    {
        case wxID_CANCEL : m_command.Clear(); break;
        case wxID_OK     : m_command = m_textCtrl->GetValue(); break;
        case ID_OPENWITH_BROWSE :
        {
            wxString filters(wxT("All files (*)|*"));
#ifdef __WXMSW__
            filters = wxT("Executable files (*.exe)|*.exe|Batch files (*.bat)|*.bat|All files (*)|*");
#endif //__WXMSW__
            wxString startPath = m_fileData.GetFilePath();
            wxString fileName = wxFileSelector( wxT("Open With"),
                                                wxPathOnly(startPath),
                                                wxFileNameFromPath(startPath),
                                                wxEmptyString,
                                                filters,
                                                wxFD_OPEN|wxFD_FILE_MUST_EXIST );
            if (!fileName.IsEmpty())
            {
                fileName += wxT(" \"") + m_fileData.GetFilePath() + wxT("\"");
                m_textCtrl->SetValue(fileName);
            }

            break;
        }
    };

    event.Skip(); // let wxDialog exit if OK or CANCEL
}

//------------------------------------------------------------------------------
// wxFilePropertiesDialog - a properties dialog for a file
//------------------------------------------------------------------------------

class wxFilePropertiesDialog : public wxDialog
{
public:
    wxFilePropertiesDialog(wxWindow* parent, wxWindowID winId,
                           const wxFileData& fileData,
                           const wxString& caption = wxT("Properties"),
                           long style = wxDEFAULT_DIALOG_STYLE,
                           const wxPoint& pos = wxDefaultPosition) : wxDialog(), m_fileData(fileData)
    {
        Create(parent, winId, fileData, caption, style, pos);
    }
    bool Create(wxWindow* parent, wxWindowID winId,
                const wxFileData& fileData,
                const wxString& caption = wxT("Properties"),
                long style = wxDEFAULT_DIALOG_STYLE,
                const wxPoint& pos = wxDefaultPosition);

protected:
    wxFileData m_fileData;
    DECLARE_EVENT_TABLE();
};

BEGIN_EVENT_TABLE(wxFilePropertiesDialog, wxDialog)
END_EVENT_TABLE()

bool wxFilePropertiesDialog::Create(wxWindow *parent, wxWindowID winId,
                                    const wxFileData& fileData,
                                    const wxString& caption,
                                    long style, const wxPoint& pos)
{
    m_fileData = fileData;

    if (!wxDialog::Create(parent, winId, caption, pos, wxDefaultSize, style))
        return false;

    wxString typeStr;
    wxString locationStr = fileData.GetFilePath().BeforeLast(wxFILE_SEP_PATH);
    wxString sizeStr;
    wxString modifiedStr = fileData.GetModificationTime();
    wxString attrStr = fileData.GetEntry(wxFileData::FileList_Perm);

    if (fileData.IsDir())
    {
        typeStr = sizeStr = _("Directory");
    }
    else if (fileData.IsLink())
    {
        typeStr = sizeStr = _("Link");
    }
    else if (fileData.IsDrive())
    {
        typeStr = sizeStr = _("Drive");
    }
    else if (fileData.IsFile())
    {
        wxString ext = wxFileName(fileData.GetFileName()).GetExt();
        wxString description;
        wxFileType *ft = wxTheMimeTypesManager->GetFileTypeFromExtension(ext);
        if (ft && ft->GetDescription(&description))
            typeStr = description;

        if (description.IsEmpty()) // gtk does this...
        {
            if (!ext.IsEmpty())
                typeStr = ext + wxT(" file");
            else
                typeStr = wxT("Unknown file type");
        }

        sizeStr = fileData.GetEntry(wxFileData::FileList_Size) + wxT(" bytes");
        attrStr = fileData.GetEntry(wxFileData::FileList_Perm);
    }

    wxPanel *panel = new wxPanel(this, wxID_ANY);
    wxBoxSizer *rootSizer = new wxBoxSizer( wxVERTICAL );
    wxFlexGridSizer *fileNameSizer = new wxFlexGridSizer( 2, 0, 0 );

    wxBitmap iconBitmap = GetBitmapFromIconId(fileData.GetImageId());
    wxStaticBitmap *iconStaticBitmap = new wxStaticBitmap( panel, wxID_ANY, iconBitmap,
                                               wxDefaultPosition, wxDefaultSize );
    fileNameSizer->Add( iconStaticBitmap, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    wxStaticText *fileNameText = new wxStaticText( panel, wxID_ANY, fileData.GetFileName(), wxDefaultPosition, wxDefaultSize, 0 );
    fileNameSizer->Add( fileNameText, 0, wxALIGN_CENTRE|wxALL, 5 );
    rootSizer->Add( fileNameSizer, 0, wxGROW|wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    wxStaticBox *generalBox = new wxStaticBox( panel, wxID_ANY, wxT("General") );
    wxStaticBoxSizer *generalBoxSizer = new wxStaticBoxSizer( generalBox, wxVERTICAL );
    wxFlexGridSizer *generalSizer = new wxFlexGridSizer( 2, 0, 0 );

    wxStaticText *typeNameText = new wxStaticText( panel, wxID_ANY, wxT("Type:"), wxDefaultPosition, wxDefaultSize, 0 );
    generalSizer->Add( typeNameText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    wxStaticText *typeText = new wxStaticText( panel, wxID_ANY, typeStr, wxDefaultPosition, wxDefaultSize, 0 );
    generalSizer->Add( typeText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    wxStaticText *locationNameText = new wxStaticText( panel, wxID_ANY, wxT("Location:"), wxDefaultPosition, wxDefaultSize, 0 );
    generalSizer->Add( locationNameText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    wxStaticText *locationText = new wxStaticText( panel, wxID_ANY, locationStr, wxDefaultPosition, wxDefaultSize, 0 );
    generalSizer->Add( locationText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    wxStaticText *sizeNameText = new wxStaticText( panel, wxID_ANY, wxT("Size:"), wxDefaultPosition, wxDefaultSize, 0 );
    generalSizer->Add( sizeNameText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    wxStaticText *sizeText = new wxStaticText( panel, wxID_ANY, sizeStr, wxDefaultPosition, wxDefaultSize, 0 );
    generalSizer->Add( sizeText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    wxStaticText *modifiedNameText = new wxStaticText( panel, wxID_ANY, wxT("Modified:"), wxDefaultPosition, wxDefaultSize, 0 );
    generalSizer->Add( modifiedNameText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    wxStaticText *modifiedText = new wxStaticText( panel, wxID_ANY, modifiedStr, wxDefaultPosition, wxDefaultSize, 0 );
    generalSizer->Add( modifiedText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    wxStaticText *attributesNameText = new wxStaticText( panel, wxID_ANY, wxT("Attributes:"), wxDefaultPosition, wxDefaultSize, 0 );
    generalSizer->Add( attributesNameText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    wxStaticText *attributesText = new wxStaticText( panel, wxID_ANY, attrStr, wxDefaultPosition, wxDefaultSize, 0 );
    generalSizer->Add( attributesText, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    generalBoxSizer->Add( generalSizer, 0, wxGROW|wxALIGN_CENTER_VERTICAL, 5 );
    rootSizer->Add( generalBoxSizer, 0, wxGROW|wxALIGN_CENTER_VERTICAL|wxALL, 5 );

    wxFlexGridSizer *buttonSizer = new wxFlexGridSizer( 2, 0, 0 );
    wxButton *okButton = new wxButton( panel, wxID_OK, wxT("OK"), wxDefaultPosition, wxDefaultSize, 0 );
    okButton->SetDefault();
    buttonSizer->Add( okButton, 0, wxALIGN_CENTRE|wxALL, 5 );
    rootSizer->Add( buttonSizer, 0, wxALIGN_CENTRE|wxALL, 5 );

    panel->SetAutoLayout( true );
    panel->SetSizer( rootSizer );
    rootSizer->Fit( panel );
    rootSizer->SetSizeHints( this );

    // have a min size so it really only varies for really long paths
    wxSize size = GetSize();
    if (size.x < 400)
        SetSize(wxSize(400, size.y));

    return true;
}

//------------------------------------------------------------------------------
// wxFileBrowserEvent, events for the wxFileBrowser
//------------------------------------------------------------------------------
DEFINE_EVENT_TYPE(wxEVT_FILEBROWSER_FILE_SELECTED)
DEFINE_EVENT_TYPE(wxEVT_FILEBROWSER_FILE_ACTIVATED)
DEFINE_EVENT_TYPE(wxEVT_FILEBROWSER_DIR_SELECTED)
DEFINE_EVENT_TYPE(wxEVT_FILEBROWSER_DIR_ACTIVATED)

IMPLEMENT_ABSTRACT_CLASS(wxFileBrowserEvent, wxCommandEvent)

wxFileBrowserEvent::wxFileBrowserEvent(wxEventType commandType,
                                       wxFileBrowser *fileBrowser, wxWindowID win_id)
                   :wxCommandEvent(commandType, win_id)
{
    SetEventObject( fileBrowser );
}

//----------------------------------------------------------------------------
// wxFileBrowser
//----------------------------------------------------------------------------
IMPLEMENT_DYNAMIC_CLASS(wxFileBrowser, wxControl);

BEGIN_EVENT_TABLE(wxFileBrowser, wxControl)
    // View toolbar items
    EVT_MENU(ID_wxFILEBROWSER_VIEW_TREE,       wxFileBrowser::OnViewButtons)
    EVT_MENU(ID_wxFILEBROWSER_VIEW_LIST,       wxFileBrowser::OnViewButtons)
    EVT_MENU(ID_wxFILEBROWSER_VIEW_DETAILS,    wxFileBrowser::OnViewButtons)
    EVT_MENU(ID_wxFILEBROWSER_VIEW_SMALL_ICON, wxFileBrowser::OnViewButtons)
    EVT_MENU(ID_wxFILEBROWSER_VIEW_LARGE_ICON, wxFileBrowser::OnViewButtons)
    EVT_MENU(ID_wxFILEBROWSER_VIEW_PREVIEW,    wxFileBrowser::OnViewButtons)

    EVT_BUTTON(ID_wxFILEBROWSER_VIEW_BUTTON,   wxFileBrowser::OnViewButtons)
    //EVT_MENU(ID_wxFILEBROWSER_VIEW_BUTTON,     wxFileBrowser::OnViewButtons)

    // menu items for the right click treectrl menu
    EVT_MENU(ID_wxFILEBROWSER_TREE_MENU_PROPERITES, wxFileBrowser::OnTreeMenu)

    // menu items for the right click listctrl menu
    EVT_MENU(ID_wxFILEBROWSER_GO_BACK,              wxFileBrowser::OnListMenu)
    EVT_MENU(ID_wxFILEBROWSER_GO_FORWARD,           wxFileBrowser::OnListMenu)
    EVT_MENU(ID_wxFILEBROWSER_GO_UP,                wxFileBrowser::OnListMenu)
    EVT_MENU(ID_wxFILEBROWSER_GO_HOME,              wxFileBrowser::OnListMenu)
    EVT_MENU(ID_wxFILEBROWSER_REFRESH,              wxFileBrowser::OnListMenu)

    EVT_MENU(wxID_OPEN,                             wxFileBrowser::OnListMenu)
    EVT_MENU(ID_wxFILEBROWSER_LIST_MENU_VIEW_FILE,  wxFileBrowser::OnListMenu)
    EVT_MENU(ID_wxFILEBROWSER_LIST_MENU_OPEN_WITH,  wxFileBrowser::OnListMenu)
    EVT_MENU(ID_wxFILEBROWSER_LIST_MENU_NEW_FOLDER, wxFileBrowser::OnListMenu)
    EVT_MENU(wxID_CUT,                              wxFileBrowser::OnListMenu)
    EVT_MENU(wxID_COPY,                             wxFileBrowser::OnListMenu)
    EVT_MENU(wxID_PASTE,                            wxFileBrowser::OnListMenu)
    EVT_MENU(ID_wxFILEBROWSER_LIST_MENU_DELETE,     wxFileBrowser::OnListMenu)
    EVT_MENU(ID_wxFILEBROWSER_LIST_MENU_RENAME,     wxFileBrowser::OnListMenu)
    EVT_MENU(ID_wxFILEBROWSER_LIST_MENU_PROPERITES, wxFileBrowser::OnListMenu)
    EVT_MENU(ID_wxFILEBROWSER_SHOW_HIDDEN,          wxFileBrowser::OnListMenu)
    EVT_MENU(ID_wxFILEBROWSER_SHOW_FOLDERS,         wxFileBrowser::OnListMenu)
    EVT_MENU(ID_wxFILEBROWSER_SPLIT_VERTICAL,       wxFileBrowser::OnListMenu)

    // combobox events
    EVT_COMBOBOX  (ID_wxFILEBROWSER_FILTER_COMBO, wxFileBrowser::OnFilterCombo)
    EVT_TEXT_ENTER(ID_wxFILEBROWSER_FILTER_COMBO, wxFileBrowser::OnFilterComboEnter)
    EVT_COMBOBOX  (ID_wxFILEBROWSER_PATH_COMBO,   wxFileBrowser::OnPathCombo)
    EVT_TEXT_ENTER(ID_wxFILEBROWSER_PATH_COMBO,   wxFileBrowser::OnPathComboEnter)
    EVT_BUTTON    (ID_wxFILEBROWSER_PATH_BUTTON,  wxFileBrowser::OnPathComboEnter)
    // fake internal events to get around GTK combo event loops
    EVT_MENU      (ID_wxFILEBROWSER_COMBOSETPATH,   wxFileBrowser::OnSetPath)
    EVT_MENU      (ID_wxFILEBROWSER_COMBOSETFILTER, wxFileBrowser::OnSetFilter)

    // TreeCtrl
    EVT_TREE_SEL_CHANGED     (wxID_ANY, wxFileBrowser::OnTreeItemSelection)
    EVT_TREE_ITEM_ACTIVATED  (wxID_ANY, wxFileBrowser::OnTreeItemActivation)
    EVT_TREE_ITEM_RIGHT_CLICK(wxID_ANY, wxFileBrowser::OnTreeRightClick)

    // ListCtrl
    EVT_LIST_ITEM_ACTIVATED   (wxID_ANY, wxFileBrowser::OnListItemActivated)
    EVT_LIST_ITEM_SELECTED    (wxID_ANY, wxFileBrowser::OnListItemSelected)
    EVT_LIST_ITEM_RIGHT_CLICK (wxID_ANY, wxFileBrowser::OnListRightClick)

    EVT_SIZE (wxFileBrowser::OnSize)
    //EVT_IDLE (wxFileBrowser::OnIdle)
END_EVENT_TABLE()

void wxFileBrowser::Init()
{
    m_ignore_tree_event = true;  // turned off after Create
    m_init_filters = 0;
    m_browser_style = wxFILEBROWSER_LIST;

    m_path_history_index = 0;

    m_viewToolBar   = nullptr;
    m_pathToolBar   = nullptr;

    m_viewButton    = nullptr;
    m_filterCombo   = nullptr;
    m_pathCombo     = nullptr;
    m_splitterWin   = nullptr;
    m_dirCtrl       = nullptr;
    m_fileCtrl      = nullptr;
    m_listMenu      = nullptr;
    m_treeMenu      = nullptr;
    m_viewMenu      = nullptr;

    m_filterComboSelection = 0;
    m_pathComboSelection = 0;

    m_show_hidden = false;
}

bool wxFileBrowser::Create( wxWindow *parent, const wxWindowID id,
                            const wxString& dir,
                            const wxPoint& pos, const wxSize& size, long style,
                            const wxString& filter, int defaultFilter,
                            const wxString& name)
{
    m_ignore_tree_event = true;

    if (!wxControl::Create(parent, id, pos, size, style, wxDefaultValidator, name))
        return false;

    // disable the log in case image handlers have already been initialized
    {
        wxLogNull logNull;
        wxInitAllImageHandlers(); // need this for the icons
    }

    // Find what directory to start with
    if (!GetPathFromFilePath(dir, m_path))
        m_path = wxGetCwd();

    // ------------------------------------------------------------------------
    // Create the popup menu for the wxFileCtrl
    m_listMenu = new wxMenu;
    m_listMenu->Append(wxID_OPEN, wxT("&Open"));
    m_listMenu->Append(ID_wxFILEBROWSER_LIST_MENU_VIEW_FILE,  wxT("&View file..."));
    m_listMenu->Append(ID_wxFILEBROWSER_LIST_MENU_OPEN_WITH,  wxT("Open &with..."));
    m_listMenu->AppendSeparator();
    m_listMenu->Append(ID_wxFILEBROWSER_LIST_MENU_NEW_FOLDER, wxT("&New folder"));
    m_listMenu->AppendSeparator();
    m_listMenu->Append(wxID_CUT,   wxT("Cu&t"));
    m_listMenu->Append(wxID_COPY,  wxT("&Copy"));
    m_listMenu->Append(wxID_PASTE, wxT("&Paste"));
    m_listMenu->AppendSeparator();
    m_listMenu->Append(ID_wxFILEBROWSER_LIST_MENU_DELETE, wxT("&Delete..."));
    m_listMenu->Append(ID_wxFILEBROWSER_LIST_MENU_RENAME, wxT("Rena&me"));
    m_listMenu->AppendSeparator();
    m_listMenu->Append(ID_wxFILEBROWSER_LIST_MENU_PROPERITES, wxT("P&roperties"));
    m_listMenu->AppendSeparator();

    m_treeMenu = new wxMenu;
    m_treeMenu->Append(ID_wxFILEBROWSER_TREE_MENU_PROPERITES, wxT("P&roperties"));

    wxMenu *optionsMenu = new wxMenu(wxEmptyString);
    optionsMenu->AppendCheckItem(ID_wxFILEBROWSER_SHOW_HIDDEN, wxT("Show Hidden Files"));
    optionsMenu->AppendCheckItem(ID_wxFILEBROWSER_SHOW_FOLDERS, wxT("Show Folders"));
    optionsMenu->AppendCheckItem(ID_wxFILEBROWSER_SPLIT_VERTICAL, wxT("Split Vertically"));
    m_listMenu->Append(ID_wxFILEBROWSER_LIST_MENU_OPTIONS, wxT("Optio&ns"), optionsMenu);

    // ------------------------------------------------------------------------
    // Create the menu for the view button menu
    m_viewMenu = new wxMenu;
    m_viewMenu->AppendRadioItem(ID_wxFILEBROWSER_VIEW_TREE,       wxT("Tree view"));
    m_viewMenu->AppendRadioItem(ID_wxFILEBROWSER_VIEW_LIST,       wxT("List view"));
    m_viewMenu->AppendRadioItem(ID_wxFILEBROWSER_VIEW_DETAILS,    wxT("Details view"));
    m_viewMenu->AppendRadioItem(ID_wxFILEBROWSER_VIEW_SMALL_ICON, wxT("Small icon view"));
    //m_viewMenu->AppendRadioItem(ID_wxFILEBROWSER_VIEW_LARGE_ICON, wxT("Large icon view"));
    //m_viewMenu->AppendRadioItem(ID_wxFILEBROWSER_VIEW_PREVIEW, wxT("Image preview view"));

    // ------------------------------------------------------------------------
    // Create the first Toolbar
    m_viewToolBar = new wxToolBar(this, ID_wxFLEBROWSER_VIEW_TOOLBAR);

    m_viewToolBar->AddTool(ID_wxFILEBROWSER_GO_BACK, wxT("Back"),
        wxArtProvider::GetBitmap(wxART_GO_BACK), wxArtProvider::GetBitmap(wxART_GO_BACK, wxART_TOOLBAR),
        wxITEM_NORMAL, wxT("Go back one directory"), wxT("Go back one directory"));
    m_viewToolBar->AddTool(ID_wxFILEBROWSER_GO_FORWARD, wxT("Forward"),
        wxArtProvider::GetBitmap(wxART_GO_FORWARD), wxArtProvider::GetBitmap(wxART_GO_FORWARD, wxART_TOOLBAR),
        wxITEM_NORMAL, wxT("Go forward one directory"), wxT("Go forward one directory"));
    m_viewToolBar->AddTool(ID_wxFILEBROWSER_GO_UP, wxT("Up"),
        wxArtProvider::GetBitmap(wxART_GO_DIR_UP), wxArtProvider::GetBitmap(wxART_GO_DIR_UP, wxART_TOOLBAR),
        wxITEM_NORMAL, wxT("Go up one directory"), wxT("Go up one directory"));
    m_viewToolBar->AddTool(ID_wxFILEBROWSER_GO_HOME, wxT("Home"),
        wxArtProvider::GetBitmap(wxART_GO_HOME), wxArtProvider::GetBitmap(wxART_GO_HOME, wxART_TOOLBAR),
        wxITEM_NORMAL, wxT("Go to your home directory"), wxT("Go to your home directory"));
    m_viewToolBar->AddSeparator();

    m_viewToolBar->AddCheckTool(ID_wxFILEBROWSER_SHOW_FOLDERS, wxT("Folders"),
        wxArtProvider::GetBitmap(wxART_HELP_SIDE_PANEL), wxArtProvider::GetBitmap(wxART_HELP_SIDE_PANEL, wxART_TOOLBAR),
        wxT("Show folder view"), wxT("Show folder view"));
    m_viewToolBar->AddSeparator();

    // MenuButton METHOD ----------------
//    m_viewButton = new wxMenuButton(m_viewToolBar, wxID_ANY,
//                                    //wxBitmap(view_xpm_data),
//                                    wxArtProvider::GetBitmap(wxART_LIST_VIEW),
//                                    wxDefaultPosition, wxSize(32,22));
//    m_viewButton->AssignMenu(viewMenu, false);
//    m_viewButton->SetToolTip(wxT("Change view mode"));
//    m_viewToolBar->AddControl(m_viewButton);

    // CREATE BITMAP METHOD ----------------
    wxBitmap viewBmp(wxArtProvider::GetBitmap(wxART_LIST_VIEW)); //, wxART_TOOLBAR));
    wxSize toolBmpSize = m_viewToolBar->GetToolSize();
    wxSize toolSize = m_viewToolBar->GetToolSize();
    {
    wxImage viewImg(viewBmp.ConvertToImage());
    int w = viewImg.GetWidth(), h = viewImg.GetHeight();
    int s = 10;
    unsigned char r = 103, g = 200, b = 101;
    viewImg.GetOrFindMaskColour(&r, &g, &b);
    viewImg.Resize(wxSize(w + s, h), wxPoint(0,0), r, g, b);
    viewImg.SetMask(false);
    viewBmp = wxBitmap(viewImg);
    wxMemoryDC memDC;
    memDC.SelectObject(viewBmp);
    wxRendererNative& renderer = wxRendererNative::Get();
    renderer.DrawComboBoxDropButton(this, memDC, wxRect(w, 0, s, toolBmpSize.y), wxCONTROL_PRESSED);
    //renderer.DrawDropArrow(this, memDC, wxRect(w, 0, s, toolBmpSize.y));

    memDC.SelectObject(wxNullBitmap);
    viewImg = viewBmp.ConvertToImage();
    viewImg.SetMaskColour(r, g, b);
    viewBmp = wxBitmap(viewImg);
    }

    m_viewButton = new wxBitmapButton(m_viewToolBar, ID_wxFILEBROWSER_VIEW_BUTTON,
                                    //wxBitmap(view_xpm_data),
                                    viewBmp,
                                    wxDefaultPosition,
                                    wxSize(toolSize.GetWidth()+10, toolSize.GetHeight()));
    m_viewButton->SetToolTip(wxT("Change view mode"));
    m_viewToolBar->AddControl(m_viewButton);

    //m_viewToolBar->AddTool(ID_wxFILEBROWSER_VIEW_BUTTON, wxT("Views"),
    //    viewBmp, viewBmp,
    //    wxITEM_NORMAL, wxT("Change view mode"), wxT("Change view mode"));

    m_filterCombo = new wxComboBox(m_viewToolBar, ID_wxFILEBROWSER_FILTER_COMBO,
                                   wxT("Any file (*.*)"), // sets init size
                                   wxDefaultPosition, wxSize(40, wxDefaultCoord),
                                   0, nullptr,
                                   wxCB_DROPDOWN|wxTE_PROCESS_ENTER);
    m_filterCombo->SetToolTip(wxT("Filter files using wildcards (file1?2.a*)"));
    SetFilters(filter, defaultFilter);
    m_viewToolBar->AddSeparator();
    m_viewToolBar->AddControl(m_filterCombo);

    m_viewToolBar->Realize();

    // ------------------------------------------------------------------------
    // Create second Toolbar
    m_pathToolBar = new wxToolBar(this, ID_wxFLEBROWSER_PATH_TOOLBAR);

    m_pathCombo = new wxComboBox(m_pathToolBar, ID_wxFILEBROWSER_PATH_COMBO,
                                 wxT("C: I'm a pretty long dir name so I'm sized big"),
                                 wxDefaultPosition, wxSize(40, wxDefaultCoord),
                                 0, nullptr,
                                 wxCB_DROPDOWN|wxTE_PROCESS_ENTER);
    m_pathCombo->SetToolTip(wxT("Enter path"));
    m_pathCombo->Append(GetPath(true), (void*)nullptr);
    m_pathCombo->SetSelection(0);
    m_pathComboSelection = 0;
    m_pathToolBar->AddControl(m_pathCombo);

    m_pathToolBar->AddTool(ID_wxFILEBROWSER_PATH_BUTTON, wxT("Open"),
        wxArtProvider::GetBitmap(wxART_FILE_OPEN), wxArtProvider::GetBitmap(wxART_FILE_OPEN, wxART_TOOLBAR),
        wxITEM_NORMAL, wxT("Go to path or open file"), wxT("Go to path or open file"));

    m_pathToolBar->Realize();

    // ------------------------------------------------------------------------
    // Create the splitter window and children
    int dirCtrlStyle = style & wxFILEBROWSER_TREE ? wxDIRCTRL_DIR_ONLY : 0;
    m_splitterWin = new wxSplitterWindow(this, wxID_ANY,
                                         wxDefaultPosition, wxDefaultSize,
                                         wxSP_BORDER|wxSP_3D|wxCLIP_CHILDREN);
    m_splitterWin->SetMinimumPaneSize(MIN_SPLIT); // don't let it unsplit
    m_splitterWin->SetSashGravity(0.3);           // bottom grows more than top

    m_dirCtrl = new wxGenericDirCtrl(m_splitterWin, ID_wxFILEBROWSER_DIRCTRL,
                                     m_path, wxDefaultPosition, wxSize(50,50),
                                     dirCtrlStyle|wxNO_BORDER, wxEmptyString, 0);

    m_dirCtrl->Show(true);

    m_fileCtrl = new wxFileCtrl(m_splitterWin, wxID_ANY, GetWild(), false,
                                wxDefaultPosition, wxSize(50,50),
                                wxNO_BORDER|wxLC_SINGLE_SEL|FBStyleToLCStyle(style));
    m_fileCtrl->Show(true);
    m_fileCtrl->GoToDir(m_path);

    // ------------------------------------------------------------------------

    SetBrowserStyle( style );
    m_splitterWin->Show(true);

    AddPathHistory(GetPath(true));

    UpdateItems();
    m_ignore_tree_event = false;
    return true;
}

wxFileBrowser::~wxFileBrowser()
{
    // delete all the attached data
    int n, count = m_filterCombo->GetCount();
    for ( n = 0; n < count; n++ )
    {
        wxString *data = (wxString*)m_filterCombo->GetClientData(n);
        delete data;
    }

    delete m_listMenu;
    delete m_treeMenu;
    delete m_viewMenu;
}

void wxFileBrowser::OnSize( wxSizeEvent &event )
{
    //wxPrintf(wxT("OnSize GetSize(%d %d) Event(%d %d)\n"), GetSize().x, GetSize().y, event.GetSize().x, event.GetSize().y); fflush(stdout);
    event.Skip();
    DoSize();

    // The m_pathCombo disappears for horiz resizing, just send another event
#if !wxCHECK_VERSION(2, 7, 0) && __WXMSW__
    if (event.GetId() == GetId())
    {
        wxSizeEvent newEvent(event);
        newEvent.SetId(GetId()+1);
        AddPendingEvent(newEvent);
    }
    else
        event.Skip(false);
#endif // !wxCHECK_VERSION(2, 7, 0) && __WXMSW__
}

// The code in src/gtk/window.cpp wxWindow::DoSetSize fails since
//  m_parent->m_wxwindow == nullptr so nothing is done
#ifdef __WXGTK__
    #include <gtk/gtk.h>
    void GtkToolbarResizeWindow(wxWindow* win, const wxSize& size)
    {
        // don't take the x,y values, they're wrong because toolbar sets them
        GtkWidget  *widget = GTK_WIDGET(win->m_widget);
        //gtk_widget_set_usize(widget, size.x, size.y); this is deprecated use below
        gtk_widget_set_size_request(widget, size.x, size.y);
        if (GTK_WIDGET_VISIBLE(widget))
            gtk_widget_queue_resize(widget);
    }
#endif //__WXGTK__

void wxFileBrowser::DoSize()
{
    if (!m_splitterWin)
        return;

    wxSize clientSize(GetClientSize());
    int h = 0;

    if (m_viewToolBar && m_filterCombo)
    {
        wxSize tbSize(m_viewToolBar->GetSize());
        m_viewToolBar->SetSize(0, h, clientSize.x, tbSize.y);
        wxSize toolSize(m_viewToolBar->GetToolSize());
        wxSize marginSize(m_viewToolBar->GetMargins());
        wxRect comboRect(m_filterCombo->GetRect());
        wxSize comboSize(clientSize.x - comboRect.x - marginSize.x, comboRect.height);

#ifdef __WXGTK__
        int sep = m_viewToolBar->GetToolSeparation();
        comboSize.x = clientSize.x - 6*toolSize.x - 6*sep;
        GtkToolbarResizeWindow(m_filterCombo, comboSize);
#else
        m_filterCombo->SetSize(comboSize);
        m_viewToolBar->Realize();
#endif

        //wxPrintf(wxT("FilterCombo %d %d - %d\n"), comboSize.x, comboSize.y, m_filterCombo->GetSize().y);
        h += tbSize.y;
    }

    if (m_pathToolBar && m_pathCombo)
    {
        wxSize tbSize(m_pathToolBar->GetSize());
        m_pathToolBar->SetSize(0, h, clientSize.x, tbSize.y);
        wxSize toolSize(m_pathToolBar->GetToolSize());
        wxSize marginSize(m_pathToolBar->GetMargins());
        wxRect comboRect(m_pathCombo->GetRect());
        wxSize comboSize(clientSize.x - comboRect.x - toolSize.x - marginSize.x, comboRect.height);

#ifdef __WXGTK__
        //int sep = m_viewToolBar->GetToolSeparation();
        comboSize.x = clientSize.x - toolSize.x - 2;
        GtkToolbarResizeWindow(m_pathCombo, comboSize);
#else
        //wxPrintf(wxT("Do Size %d %d - %d %d %d\n"), clientSize.x, clientSize.y, clientSize.x, comboRect.width, clientSize.x - comboRect.x - toolSize.x - marginSize.x);
        m_pathCombo->SetSize(comboSize);
        m_pathToolBar->Realize();
#endif

        //wxPrintf(wxT("PathCombo %d %d - %d\n"), clientSize.x - comboRect.x - toolSize.x - marginSize.x, comboRect.height, m_pathCombo->GetSize().y);
        h += tbSize.y;
    }

    m_splitterWin->SetSize(wxRect(0, h, clientSize.x, clientSize.y - h));

    // Make sure the splitter window doesn't unsplit when the window size changes,
    //   even though we already told it not to.
    // FIXME : This should be fixed in the splitter window
    if (m_splitterWin && m_splitterWin->IsSplit())
    {
        int splitter_pos = m_splitterWin->GetSashPosition();
        int splitter_size = (m_splitterWin->GetSplitMode() == wxSPLIT_VERTICAL) ?
              m_splitterWin->GetSize().GetWidth() : m_splitterWin->GetSize().GetHeight();
        int min_size = m_splitterWin->GetMinimumPaneSize();

        if (splitter_pos < min_size)
            m_splitterWin->SetSashPosition(wxMin(splitter_size, min_size));
        else if (splitter_pos > splitter_size - min_size)
            m_splitterWin->SetSashPosition(wxMax(0, splitter_size - min_size));
    }
}

wxSize wxFileBrowser::DoGetBestSize() const
{
    return wxSize(250,400);
    //return wxControl::DoGetBestSize();
}

void wxFileBrowser::OnIdle(wxIdleEvent &event)
{
    event.Skip();

    // Ancient code to display a preview of images

/*
    if ((m_browser_style & wxFILEBROWSER_PREVIEW) &&
        (int(m_previewFiles.GetCount()) > 0))
    {
        int i, count = m_previewFiles.GetCount();
        int flags = 0;
        int prev_index = 0;
        int min_index = m_fileCtrl->HitTest(wxPoint(20,150), flags);
        if (min_index < 0) min_index = 0;
        int max_index = min_index + (m_fileCtrl->GetClientSize().x*m_fileCtrl->GetClientSize().y)/3000;

        for (i=0; i<count; i++)
        {
            if ((m_previewFiles[i]->m_list_index >= min_index) ||
                (m_previewFiles[i]->m_list_index <= max_index))
            {
                prev_index = i;
                break;
            }
        }

        wxFileBrowserFile *browserFile = m_previewFiles[prev_index];

        int icon_index = browserFile->m_icon_index;

        wxString path = GetPath(true);

        wxImage img(path + browserFile->m_name);
        if (img.Ok())
        {
            int w = img.GetWidth(), h = img.GetHeight();
            browserFile->m_image_size.Printf(wxT("(%dx%d)"), w, h);

            int icon_w, icon_h;
            m_normalImageList->GetSize(0, icon_w, icon_h);

            if (w > icon_w || h > icon_h)
            {
                if (w > h)
                {
                    h = (h*icon_w)/w;
                    if (h == 0) h = 1;
                    w = icon_w;
                }
                else
                {
                    w = (w*icon_h)/h;
                    if (w == 0) w = 1;
                    h = icon_h;
                }

                img.Rescale(w, h);
            }

            wxImage img2(icon_w, icon_h);
            unsigned char r=5, g=5, b=5;
            if (img.HasMask())
            {
                r = img.GetMaskRed();
                g = img.GetMaskGreen();
                b = img.GetMaskBlue();
            }
            else
            {
                img.FindFirstUnusedColour(&r, &g, &b);
                img.SetMask(true);
                img.SetMaskColour(r, g, b);
            }
            unsigned char *data = img2.GetData();
            long icon_size = icon_w*icon_h;
            for (i=0; i<icon_size; i++)
            {
                *data++ = r;
                *data++ = g;
                *data++ = b;
            }
            img2.SetMask(true);
            img2.SetMaskColour(r, g, b);
            img2.Paste(img, (icon_w-w)/2, (icon_h-h)/2);

            wxIcon icon;
            icon.CopyFromBitmap(wxBitmap(img2));
            m_normalImageList->Replace(icon_index, icon);

            if (browserFile->m_list_index >= 0) // maybe still filling the listCtrl
            {
                m_fileCtrl->SetItemImage(browserFile->m_list_index,
                                         icon_index, icon_index);
            }
        }
        m_previewFiles.RemoveAt(prev_index);

        if (count > 1) event.RequestMore();
    }
 */
}

#define FILEBRWS_LIST_VIEW_MASK (wxFILEBROWSER_DETAILS|wxFILEBROWSER_LIST|wxFILEBROWSER_SMALL_ICON|wxFILEBROWSER_LARGE_ICON|wxFILEBROWSER_PREVIEW)
#define FILEBRWS_VIEW_MASK (FILEBRWS_LIST_VIEW_MASK|wxFILEBROWSER_TREE)

void wxFileBrowser::SetBrowserStyle( long style )
{
    style &= wxFILEBROWSER_STYLE_MASK;

    int n_styles = 0;
    if (style & wxFILEBROWSER_LIST      ) n_styles++;
    if (style & wxFILEBROWSER_DETAILS   ) n_styles++;
    if (style & wxFILEBROWSER_LARGE_ICON) n_styles++;
    if (style & wxFILEBROWSER_SMALL_ICON) n_styles++;
    if (style & wxFILEBROWSER_PREVIEW   ) n_styles++;
    if (style & wxFILEBROWSER_TREE      ) n_styles++;
    wxCHECK_RET(n_styles == 1, wxT("Only one wxFileBrowser list style allowed"));

    long last_style = m_browser_style;
    m_browser_style = style; // swap to new style immediately

    // Only the tree view will be shown
    if (!HasBrowserStyle(FILEBRWS_LIST_VIEW_MASK))
    {
        m_fileCtrl->Show(false);

        if (m_splitterWin->IsSplit())
            m_splitterWin->Unsplit(m_fileCtrl);
        else // if (m_splitterWin->GetWindow1() != m_dirCtrl)
            m_splitterWin->Initialize(m_dirCtrl);

        // show and reload the files for the dirctrl using the filter
        if ((m_dirCtrl->GetWindowStyleFlag() & wxDIRCTRL_DIR_ONLY) != 0)
        {
            m_ignore_tree_event = true;
            wxString currentPath = GetPath(false);
            m_dirCtrl->SetWindowStyleFlag(m_dirCtrl->GetWindowStyleFlag() & (~wxDIRCTRL_DIR_ONLY));
            m_dirCtrl->SetFilter(GetFilter());
            m_dirCtrl->ReCreateTree();
            m_dirCtrl->ExpandPath(currentPath);
            m_ignore_tree_event = false;
        }

        m_splitterWin->SizeWindows();
        UpdateItems();
        return;
    }

    // Else - show a combination of tree and list views

    // Try to avoid flicker by changing mode only if necessary
    switch (m_browser_style & FILEBRWS_VIEW_MASK)
    {
        case wxFILEBROWSER_DETAILS :
        {
            if ((last_style & wxFILEBROWSER_DETAILS) == 0)
                m_fileCtrl->ChangeToReportMode();
            break;
        }
        case wxFILEBROWSER_LIST :
        {
            if ((last_style & wxFILEBROWSER_LIST) == 0)
                m_fileCtrl->ChangeToListMode();
            break;
        }
        case wxFILEBROWSER_SMALL_ICON :
        {
            if ((last_style & wxFILEBROWSER_SMALL_ICON) == 0)
                m_fileCtrl->ChangeToSmallIconMode();
            break;
        }
        default : break;
    }

    // are both tree and list view shown
    if (GetShowFolders())
    {
        if (GetSplitVertical())
        {
            if (m_splitterWin->GetSplitMode() != wxSPLIT_VERTICAL)
            {
                if (m_splitterWin->IsSplit()) // FIXME need to unsplit to resplit
                    m_splitterWin->Unsplit(m_splitterWin->GetWindow2());
            }

            int sash_pos = m_splitterWin->GetSashPosition();
            m_splitterWin->SplitVertically(m_dirCtrl, m_fileCtrl, sash_pos);
        }
        else
        {
            if (m_splitterWin->GetSplitMode() != wxSPLIT_HORIZONTAL)
            {
                if (m_splitterWin->IsSplit()) // FIXME need to unsplit to resplit
                    m_splitterWin->Unsplit(m_splitterWin->GetWindow2());
            }

            int sash_pos = m_splitterWin->GetSashPosition();
            m_splitterWin->SplitHorizontally(m_dirCtrl, m_fileCtrl, sash_pos);
        }

        m_dirCtrl->Show(true);
    }
    else // just list view
    {
        if (m_splitterWin->IsSplit())
            m_splitterWin->Unsplit(m_dirCtrl);
        else // if (m_splitterWin->GetWindow1() != m_fileCtrl)
            m_splitterWin->Initialize(m_fileCtrl);

        m_dirCtrl->Show(false);
    }

    m_fileCtrl->Show(true);

    // In dir mode always show all dirs
    m_dirCtrl->SetFilter(wxEmptyString);

    // if it used to have wxFILEBROWSER_TREE style, make tree show only dirs
    if ((m_dirCtrl->GetWindowStyleFlag() & wxDIRCTRL_DIR_ONLY) == 0)
    {
        m_ignore_tree_event = true;
        m_dirCtrl->SetFilter(wxT("*|*"));
        m_dirCtrl->SetWindowStyleFlag(m_dirCtrl->GetWindowStyleFlag() | wxDIRCTRL_DIR_ONLY);
        wxString currentPath = GetPath(false);
        m_dirCtrl->ReCreateTree();
        m_dirCtrl->ExpandPath(currentPath);
        m_ignore_tree_event = false;
    }

    m_splitterWin->SizeWindows();
    UpdateItems();
}

bool wxFileBrowser::CanRead(const wxString& filePath) const
{
    return wxFile::Access(filePath, wxFile::read);
}

bool wxFileBrowser::CanWrite(const wxString& filePath) const
{
    return wxFile::Access(filePath, wxFile::write);
}

void wxFileBrowser::UpdateItems()
{
    UpdateMenu(m_listMenu);
    UpdateMenu(m_viewMenu);

    UpdateToolBar(m_viewToolBar);
    UpdateToolBar(m_pathToolBar);
}

void EnableMenuItem(wxMenu *menu, wxWindowID menu_id, bool value)
{
    if (menu)
    {
        wxMenuItem *menuItem = menu->FindItem(menu_id);
        if (menuItem)
            menuItem->Enable(value);
    }
}
void CheckMenuItem(wxMenu *menu, wxWindowID menu_id, bool value)
{
    if (menu)
    {
        wxMenuItem *menuItem = menu->FindItem(menu_id);
        if (menuItem)
            menuItem->Check(value);
    }
}

void wxFileBrowser::UpdateMenu( wxMenu* menu )
{
    if (!menu) return;

    CheckMenuItem(menu, FBStyleToMenuID(m_browser_style), true);

    // Update options menu ---------------------------

    CheckMenuItem(menu,  ID_wxFILEBROWSER_SHOW_HIDDEN, GetShowHidden());
    EnableMenuItem(menu, ID_wxFILEBROWSER_SHOW_FOLDERS, !HasBrowserStyle(wxFILEBROWSER_TREE));
    CheckMenuItem(menu,  ID_wxFILEBROWSER_SHOW_FOLDERS, GetShowFolders());
    CheckMenuItem(menu,  ID_wxFILEBROWSER_SPLIT_VERTICAL, GetSplitVertical());

    // Update go items ---------------------------

    CheckMenuItem(menu, ID_wxFILEBROWSER_GO_BACK,    CanGoPathHistoryBackward());
    CheckMenuItem(menu, ID_wxFILEBROWSER_GO_FORWARD, CanGoPathHistoryForward());
    CheckMenuItem(menu, ID_wxFILEBROWSER_GO_UP,      CanGoUpDir());

    // Update cut, copy, paste ---------------------------

    // wxLIST_STATE_FOCUSED ? nah probably selected
    long item = m_fileCtrl->GetNextItem(-1, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED);

    wxFileData *fd = nullptr;
    if (item >= 0)
        fd = (wxFileData*)m_fileCtrl->GetItemData(item);

    bool is_file = fd && !fd->IsDir() && !fd->IsDrive() && !fd->IsLink();
    bool is_dir  = fd && fd->IsDir();

    // There's nothing you can do with this
    if (fd && (fd->GetFileName() == wxT("..")))
    {
        is_file = is_dir = false;
    }

    bool can_read  = (is_file || is_dir) && fd && CanRead(fd->GetFilePath());
    bool can_write = (is_file || is_dir) && fd && CanWrite(fd->GetFilePath());
    bool can_write_dir = CanWrite(GetPath());

    // can't do anything with drive listing in MSW
    bool is_top = false;
#ifndef __UNIX__
    if (GetPath(false).IsEmpty()) is_top = true;
#endif //__UNIX__

    EnableMenuItem(menu, wxID_OPEN, (is_file || is_dir) && can_read);
    EnableMenuItem(menu, ID_wxFILEBROWSER_LIST_MENU_VIEW_FILE, is_file && can_read);
    EnableMenuItem(menu, ID_wxFILEBROWSER_LIST_MENU_OPEN_WITH, is_file && can_read);

    EnableMenuItem(menu, ID_wxFILEBROWSER_LIST_MENU_NEW_FOLDER, !is_top && can_write_dir);

    EnableMenuItem(menu, wxID_CUT,    is_file && can_write);
    EnableMenuItem(menu, wxID_COPY,   is_file && can_read);
    EnableMenuItem(menu, wxID_PASTE, !is_top && (m_copycutFiles.GetCount() > 0u) && can_write_dir);

    EnableMenuItem(menu, ID_wxFILEBROWSER_LIST_MENU_DELETE, (is_file || is_dir) && can_write);
    EnableMenuItem(menu, ID_wxFILEBROWSER_LIST_MENU_RENAME, (is_file || is_dir) && can_write);

    EnableMenuItem(menu, ID_wxFILEBROWSER_LIST_MENU_PROPERITES, is_file || is_dir);
}
void wxFileBrowser::UpdateToolBar( wxToolBar* toolBar )
{
    if (!toolBar) return;

    toolBar->EnableTool(ID_wxFILEBROWSER_GO_BACK,      CanGoPathHistoryBackward());
    toolBar->EnableTool(ID_wxFILEBROWSER_GO_FORWARD,   CanGoPathHistoryForward());
    toolBar->EnableTool(ID_wxFILEBROWSER_GO_UP,        CanGoUpDir());
    toolBar->EnableTool(ID_wxFILEBROWSER_SHOW_FOLDERS, !HasBrowserStyle(wxFILEBROWSER_TREE));
    toolBar->ToggleTool(ID_wxFILEBROWSER_SHOW_FOLDERS, GetShowFolders());
}

void wxFileBrowser::ShowHidden(bool show_hidden)
{
    m_show_hidden = show_hidden;

    m_ignore_tree_event = true;
    m_dirCtrl->ShowHidden(m_show_hidden);
    m_fileCtrl->ShowHidden(m_show_hidden);
    m_ignore_tree_event = false;

    UpdateItems();
}

void wxFileBrowser::ShowFolders(bool show_folders)
{
    m_browser_style &= (~wxFILEBROWSER_SHOW_FOLDERS);
    if (show_folders)
        m_browser_style |= wxFILEBROWSER_SHOW_FOLDERS;

    SetBrowserStyle(m_browser_style);
}

void wxFileBrowser::SplitVertical(bool split_vertically)
{
    m_browser_style &= (~wxFILEBROWSER_SPLIT_VERTICAL);
    if (split_vertically)
        m_browser_style |= wxFILEBROWSER_SPLIT_VERTICAL;

    SetBrowserStyle(m_browser_style);
}

bool wxFileBrowser::GetPathFromFilePath(const wxString &filepath, wxString &path) const
{
    path = filepath;

    wxFileName filename(filepath);
    if (filename.FileExists())
        path = filename.GetPath();

    if (!wxDirExists(path)) return false;

    if (path.Last() != wxFILE_SEP_PATH)
        path += wxFILE_SEP_PATH;

    return true;
}

wxString AddDelete_wxFILE_SEP_PATH(const wxString &path_, bool add_sep)
{
    wxString path(path_);

#ifdef __UNIX__
    if (path.IsEmpty()) return wxFILE_SEP_PATH;
#endif

    if (add_sep)
    {
        if (path.IsEmpty() || (path.Last() != wxFILE_SEP_PATH))
            path += wxFILE_SEP_PATH;
    }
    else if (!path.IsEmpty() && (path.Last() == wxFILE_SEP_PATH))
        path = path.RemoveLast();

    return path;
}

wxString wxFileBrowser::GetPath(bool add_wxFILE_SEP_PATH) const
{
    return AddDelete_wxFILE_SEP_PATH(m_path, add_wxFILE_SEP_PATH);
}

bool wxFileBrowser::SetPath(const wxString &dirname)
{
    wxString path = dirname;

    if (dirname.IsEmpty())
    {
#ifdef __UNIX__
        path = wxFILE_SEP_PATH;
#endif
    }
    else
    {
        if (!GetPathFromFilePath(dirname, path))
            return false;
    }

    m_path = AddDelete_wxFILE_SEP_PATH(path, true);

    m_ignore_tree_event = true;
    m_dirCtrl->SetPath(GetPath(false)); // doesn't like trailing wxFILE_SEP_PATH - segfault
    m_ignore_tree_event = false;

    if (!HasBrowserStyle(wxFILEBROWSER_TREE)) // don't care otherwise
        m_fileCtrl->GoToDir(m_path);

    InsertComboItem(m_pathCombo, GetPath(true), 0);

    AddPathHistory(GetPath(true));
    UpdateItems();
    return true;
}

bool wxFileBrowser::CanGoUpDir() const
{
    wxString path = GetPath(false).BeforeLast(wxFILE_SEP_PATH);
    return !path.IsEmpty() && wxDirExists(path);
}
bool wxFileBrowser::GoUpDir()
{
    wxString path = GetPath(false).BeforeLast(wxFILE_SEP_PATH);
    if (!path.IsEmpty() && wxDirExists(path))
        return SetPath(path);

    return false;
}

bool wxFileBrowser::GoToHomeDir()
{

    return SetPath(wxFileName::GetHomeDir());
}

bool wxFileBrowser::CanGoPathHistoryForward()
{
    if (m_path_history_index < int(m_pathHistory.GetCount()) - 1)
    {
        // maybe they deleted it?
        if (!wxDirExists(m_pathHistory[m_path_history_index+1]))
        {
            m_path_history_index++;
            m_pathHistory.RemoveAt(m_path_history_index);
            return CanGoPathHistoryForward();
        }

        return true;
    }
    if (m_pathHistory.GetCount() == 0u)
        m_pathHistory.Add(GetPath(true));

    return false;
}
bool wxFileBrowser::CanGoPathHistoryBackward()
{
    if (m_path_history_index != 0)
    {
        // maybe they deleted it?
        if (!wxDirExists(m_pathHistory[m_path_history_index-1]))
        {
            m_path_history_index--;
            m_pathHistory.RemoveAt(m_path_history_index);
            return CanGoPathHistoryBackward();
        }

        return true;
    }
    if (m_pathHistory.GetCount() == 0u)
        m_pathHistory.Add(GetPath(true));

    return false;
}

bool wxFileBrowser::GoPathHistoryForward()
{
    if (!CanGoPathHistoryForward())
        return false;

    m_path_history_index++;
    return SetPath(m_pathHistory[m_path_history_index]);
}
bool wxFileBrowser::GoPathHistoryBackward()
{
    if (!CanGoPathHistoryBackward())
        return false;

    m_path_history_index--;
    return SetPath(m_pathHistory[m_path_history_index]);
}

void wxFileBrowser::AddPathHistory(const wxString& path)
{
    if (!wxDirExists(path) || (path == m_pathHistory[m_path_history_index]))
        return;

    int count = m_pathHistory.GetCount();
    if (count == 0)
    {
        m_path_history_index = 0;
        m_pathHistory.Add(path);
    }
    else if (m_path_history_index == count-1)
    {
        m_path_history_index++;
        m_pathHistory.Add(path);
    }
    else
    {
        m_path_history_index++;
        m_pathHistory[m_path_history_index] = path;
    }
}

void wxFileBrowser::OnTreeItemSelection(wxTreeEvent &event)
{
    if (m_ignore_tree_event)
    {
        event.Skip();
        return;
    }

    wxFileBrowserEvent fbEvent(wxEVT_FILEBROWSER_FILE_SELECTED, this, GetId());

    if (m_dirCtrl->GetFilePath().IsEmpty())
        fbEvent.SetEventType(wxEVT_FILEBROWSER_DIR_SELECTED);

    fbEvent.SetFilePath(m_dirCtrl->GetPath());
    DoSendEvent(fbEvent);
}

void wxFileBrowser::OnTreeItemActivation(wxTreeEvent &event)
{
    if (m_ignore_tree_event)
    {
        event.Skip();
        return;
    }

    wxString path;
    if (!GetPathFromFilePath(m_dirCtrl->GetPath(), path))
        return;

    wxFileBrowserEvent fbEvent(wxEVT_FILEBROWSER_FILE_ACTIVATED, this, GetId());

    m_path = path;
    if (m_dirCtrl->GetFilePath().IsEmpty())
    {
        fbEvent.SetEventType(wxEVT_FILEBROWSER_DIR_ACTIVATED);

        if (!HasBrowserStyle(wxFILEBROWSER_TREE))
            m_fileCtrl->GoToDir(m_path);
    }

    fbEvent.SetFilePath(m_dirCtrl->GetPath());
    DoSendEvent(fbEvent);
}

void wxFileBrowser::OnTreeRightClick(wxTreeEvent &event)
{
    UpdateMenu(m_treeMenu);
    m_dirCtrl->PopupMenu(m_treeMenu, event.GetPoint());
}

void wxFileBrowser::OnListItemActivated(wxListEvent &event)
{
    wxString filename = event.GetLabel();
    if (filename.IsEmpty()) return;

    wxFileData *fd = (wxFileData*)event.GetData();
    wxCHECK_RET(fd, wxT("Invalid filedata"));
    wxString filePath = fd->GetFilePath();

    wxEventType evtType = fd->IsDir() ? wxEVT_FILEBROWSER_DIR_ACTIVATED :
                                        wxEVT_FILEBROWSER_FILE_ACTIVATED;
    wxFileBrowserEvent fbEvent(evtType, this, GetId());

    if (fd->IsDir())
    {
        if (filename == wxT(".."))
            filePath = filePath.RemoveLast().BeforeLast(wxFILE_SEP_PATH);

        SetPath(filePath);
    }

    fbEvent.SetFilePath(filePath);
    DoSendEvent(fbEvent);
}

void wxFileBrowser::OnListItemSelected(wxListEvent &event)
{
    wxFileData *fd = (wxFileData*)event.GetData();
    wxCHECK_RET(fd, wxT("Invalid filedata"));

    wxEventType evtType = fd->IsDir() ? wxEVT_FILEBROWSER_DIR_SELECTED :
                                        wxEVT_FILEBROWSER_FILE_SELECTED;
    wxFileBrowserEvent fbEvent(evtType, this, GetId());
    fbEvent.SetFilePath(fd->GetFilePath());
    DoSendEvent(fbEvent);
}

void wxFileBrowser::OnListRightClick(wxListEvent &event)
{
    UpdateMenu(m_listMenu);
    m_fileCtrl->PopupMenu(m_listMenu, event.GetPoint());
}

wxFileData* wxFileBrowser::GetFocusedListItem() const
{
    long item = m_fileCtrl->GetNextItem(-1, wxLIST_NEXT_ALL, wxLIST_STATE_FOCUSED);

    if (item >= 0)
         return (wxFileData*)m_fileCtrl->GetItemData(item);

    return nullptr;
}

wxArrayInt wxFileBrowser::GetSelectedListItems() const
{
    wxArrayInt selItems;
    long item = m_fileCtrl->GetNextItem(-1, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED);

    while (item >= 0)
    {
        selItems.Add((int)item);
        item = m_fileCtrl->GetNextItem(item, wxLIST_NEXT_ALL, wxLIST_STATE_SELECTED);
    }

    return selItems;
}

wxArrayFileData wxFileBrowser::GetSelectedListFileData() const
{
    wxArrayFileData fileDatas;
    wxArrayInt selItems = GetSelectedListItems();
    if ((int)selItems.GetCount() < 1)
        return fileDatas;

    for (size_t n=0; n<selItems.GetCount(); n++)
    {
        wxFileData *fd = (wxFileData*)m_fileCtrl->GetItemData(selItems[n]);
        wxCHECK_MSG(fd, fileDatas, wxT("Invalid filedata item"));

        wxFileData newFd(*fd);
        fileDatas.Add(newFd);
    }

    return fileDatas;
}


wxFileData wxFileBrowser::CreateFileData(const wxFileName& fileName) const
{
    if (fileName.DirExists())
    {
        return wxFileData(fileName.GetPath(), fileName.GetName(), wxFileData::is_dir, wxFileIconsTable::folder);
    }
    else if (fileName.FileExists())
    {
        return wxFileData(fileName.GetPath(), fileName.GetName(), wxFileData::is_file, wxFileIconsTable::file);
    }
#if defined(__WINDOWS__) || defined(__DOS__) || defined(__WXMAC__) || defined(__OS2__)
    else
    {
        // FIXME this is ugly, but should work
        wxArrayString names, paths;
        wxArrayInt icons;
        size_t n, count = wxGetAvailableDrives(paths, names, icons);
        for (n = 0; n < count; n++)
        {
            if (fileName.GetFullPath() == paths[n])
                return wxFileData(fileName.GetPath(), fileName.GetName(), wxFileData::is_drive, icons[n]);
        }
    }
#endif
    return wxFileData();
}

bool wxFileBrowser::DeleteSelectedListItems(bool WXUNUSED(ask_ok))
{
    wxArrayInt selItems = GetSelectedListItems();
    if ((int)selItems.GetCount() < 1)
        return false;

    size_t n = 0;
    wxArrayString filePaths;
    wxArrayInt isDirs;
    wxString fileNameString;
    int line_length = 0;
    for (n=0; n<selItems.GetCount(); n++)
    {
        wxFileData *fd = (wxFileData*)m_fileCtrl->GetItemData(selItems[n]);
        wxCHECK_MSG(fd, false, wxT("Invalid filedata item"));

        if (fd->IsDrive())
        {
            wxString msg = wxT("Unable to delete drive: \"")+fd->GetFilePath()+wxT("\"\n");
            msg += wxT("Please deselect drive and try again.");
            wxMessageBox(msg,
                         wxT("Error deleting"), wxOK|wxCENTRE|wxICON_INFORMATION, this);
            return false;
        }
        if (fd->GetFileName() == wxT(".."))
        {
            wxString msg = wxT("Unable to delete parent directory: \"")+fd->GetFilePath()+wxT("\"\n");
            msg += wxT("Please deselect dir and try again.");
            wxMessageBox(msg,
                         wxT("Error deleting"), wxOK|wxCENTRE|wxICON_INFORMATION, this);
            return false;
        }

        isDirs.Add(fd->IsDir() ? 1 : 0);
        filePaths.Add(fd->GetFilePath());
        if (line_length > 100)
        {
            fileNameString += wxT("\n");
            line_length = 0;
        }
        fileNameString += fd->GetFileName();
        if (n < selItems.GetCount() - 1)
            fileNameString += wxT(", ");

        line_length += fd->GetFileName().Length() + 2;
    }

    MultilineTextDialog tDialog( this,
                                 wxT("Delete files(s)/dir(s)?"),
                                 wxT("Confirm deleting?"),
                                 fileNameString,
                                 wxTE_READONLY);

    if (tDialog.ShowModal() != wxID_OK)
        return false;

    for (n=0; n<filePaths.GetCount(); n++)
    {
        if (isDirs[n])
        {
            if (CanWrite(filePaths[n]) && wxRmdir(filePaths[n]))
                m_fileCtrl->DeleteItem(selItems[n]);
            else
            {
                wxString msg = wxT("Sorry, unable to delete dir: \"")+filePaths[n]+wxT("\"\n");
                msg += wxT("Perhaps it is not empty?");
                int ret = wxMessageBox(msg, wxT("Error deleting dir"),
                               wxOK|wxCANCEL|wxCENTRE|wxICON_INFORMATION, this);

                if (ret == wxCANCEL) break;
            }
        }
        else
        {
            if (CanWrite(filePaths[n]) && wxRemoveFile(filePaths[n]))
                m_fileCtrl->DeleteItem(selItems[n]);
            else
            {
                int ret = wxMessageBox(wxT("Sorry, unable to delete file : \"")+filePaths[n]+wxT("\""),
                               wxT("Error deleting file"),
                               wxOK|wxCANCEL|wxCENTRE|wxICON_INFORMATION, this);
                if (ret == wxCANCEL) break;
            }
        }
    }

    SetPath(GetPath()); // refresh views
    return true;
}

bool wxFileBrowser::CopyCutSelectedListItems(bool copy_them)
{
    m_last_copy = copy_them;
    m_copycutFiles.Clear();

    wxArrayInt selItems = GetSelectedListItems();
    if ((int)selItems.GetCount() < 1)
        return false;

    for (size_t n=0; n<selItems.GetCount(); n++)
    {
        wxFileData *fd = (wxFileData*)m_fileCtrl->GetItemData(selItems[n]);
        wxCHECK_MSG(fd, false, wxT("Invalid filedata item"));

        if (fd->GetFileName() != wxT(".."))
        {
            if (!m_last_copy)
                m_fileCtrl->SetItemState(selItems[n], wxLIST_MASK_STATE, wxLIST_STATE_CUT);

            m_copycutFiles.Add(wxFileData(*fd));
        }
    }

    return m_copycutFiles.GetCount() > 0U;
}

bool wxFileBrowser::PasteCopyCutSelectedListItems()
{
    wxString path = GetPath(true);

    for (size_t n=0; n<m_copycutFiles.GetCount(); n++)
    {
        wxString srcFilePath = m_copycutFiles[n].GetFilePath();
        wxString srcFileName = m_copycutFiles[n].GetFileName();
        wxString dstFilePath = path + srcFileName;
        bool src_file_exists = wxFileExists(srcFilePath);
        bool dst_file_exists = wxFileExists(dstFilePath);

        if (!src_file_exists)
        {
            wxString msg = wxT("Source file doesn't exist anymore.\n");
            msg += m_copycutFiles[n].GetHint();
            int ret = wxMessageBox(msg,
                         wxT("Error pasting file"), wxOK|wxCANCEL|wxCENTRE|wxICON_INFORMATION, this);

            if (ret == wxCANCEL) break;
            continue;
        }

        if (dst_file_exists)
        {
            wxFileData dstFd(dstFilePath, srcFileName, wxFileData::is_file, wxFileIconsTable::file);

            wxString msg = wxT("Overwrite destination file?\n");
            msg += wxT("Source: ") + m_copycutFiles[n].GetHint() + wxT("\n");
            msg += wxT("Destination: ") + dstFd.GetHint();
            int ret = wxMessageBox(msg,
                    wxT("Overwrite file?"), wxYES_NO|wxCANCEL|wxCENTRE|wxICON_INFORMATION, this);

            if (ret == wxNO) continue;
            if (ret == wxCANCEL) break;
        }

        if (m_last_copy)
        {
            if (!wxCopyFile(srcFilePath, dstFilePath, true))
            {
                wxFileData dstFd(dstFilePath, srcFileName, wxFileData::is_file, wxFileIconsTable::file);

                wxString msg = wxT("Unknown error trying to copy file.\n");
                msg += wxT("Source: ") + m_copycutFiles[n].GetHint() + wxT("\n");
                msg += wxT("Destination: ") + dstFd.GetHint();
                int ret = wxMessageBox(msg,
                            wxT("Error copying file"), wxOK|wxCANCEL|wxCENTRE|wxICON_INFORMATION, this);

                if (ret == wxCANCEL) break;
                continue;
            }
        }
        else // we cut them, so just move them
        {
            //if (!wxRenameFile(srcFilePath, dstFilePath)) // FIXME - why does wxRename copy?
            if (0 != rename((const char*)wxConvUTF8.cWX2MB(srcFilePath.c_str()),
                            (const char*)wxConvUTF8.cWX2MB(dstFilePath.c_str())))
            {
                wxFileData dstFd(dstFilePath, srcFileName, wxFileData::is_file, wxFileIconsTable::file);

                wxString msg = wxT("Unknown error trying to move file.\n");
                msg += wxT("Source: ") + m_copycutFiles[n].GetHint() + wxT("\n");
                msg += wxT("Destination: ") + dstFd.GetHint();
                int ret = wxMessageBox(msg,
                            wxT("Error moving file"), wxOK|wxCANCEL|wxCENTRE|wxICON_INFORMATION, this);

                if (ret == wxCANCEL) break;
                continue;
            }
        }

        long item_id = m_fileCtrl->FindItem(-1, srcFileName);

        if (!dst_file_exists || (item_id < 0))
        {
            wxListItem item;
            item.m_itemId = m_fileCtrl->GetItemCount();
            item.m_col = 0;
            wxFileData *fd = new wxFileData(dstFilePath, srcFileName, wxFileData::is_file, wxFileIconsTable::file);
            m_fileCtrl->Add(fd, item);
        }
        else
        {
            wxListItem item;
            item.m_itemId = item_id;
            m_fileCtrl->UpdateItem(item);
        }
    }

    SetPath(GetPath()); // refresh views
    return true;
}

void wxFileBrowser::OnTreeMenu(wxCommandEvent &event)
{
    switch(event.GetId())
    {
        case ID_wxFILEBROWSER_TREE_MENU_PROPERITES :
        {
            wxFileName fileName(m_dirCtrl->GetPath());
            wxFileData fd(CreateFileData(fileName));
            //if (!fd) return;

            ShowPropertiesDialog(fd);
            break;
        }
        default : break;
    }
}

void wxFileBrowser::OnListMenu(wxCommandEvent &event)
{
    switch(event.GetId())
    {
        case ID_wxFILEBROWSER_GO_BACK :
        {
            if (CanGoPathHistoryBackward())
                GoPathHistoryBackward();

            break;
        }
        case ID_wxFILEBROWSER_GO_FORWARD :
        {
            if (CanGoPathHistoryForward())
                GoPathHistoryForward();

            break;
        }
        case ID_wxFILEBROWSER_GO_UP :
        {
            if (CanGoUpDir())
                GoUpDir();

            break;
        }
        case ID_wxFILEBROWSER_GO_HOME :
        {
            GoToHomeDir();
            break;
        }
        case ID_wxFILEBROWSER_REFRESH :
        {
            SetPath(GetPath(true));
            break;
        }
        case wxID_OPEN :
        {
            wxFileData *fd = GetFocusedListItem();
            if (!fd) return;

            OpenFilePath(fd->GetFilePath());
            break;
        }
        case ID_wxFILEBROWSER_LIST_MENU_NEW_FOLDER :
        {
            m_fileCtrl->MakeDir();
            SetPath(GetPath(true));
            break;
        }
        case wxID_CUT :
        case wxID_COPY :
        {
            m_last_copy = event.GetId() == wxID_COPY;
            CopyCutSelectedListItems(m_last_copy);
            break;
        }
        case wxID_PASTE :
        {
            PasteCopyCutSelectedListItems();
            break;
        }
        case ID_wxFILEBROWSER_LIST_MENU_DELETE :
        {
            DeleteSelectedListItems(true);
            break;
        }
        case ID_wxFILEBROWSER_LIST_MENU_RENAME :
        {
            long item = m_fileCtrl->GetNextItem(-1, wxLIST_NEXT_ALL, wxLIST_STATE_FOCUSED);
            if (item >= 0)
                m_fileCtrl->EditLabel( item );
            break;
        }
        case ID_wxFILEBROWSER_LIST_MENU_OPEN_WITH :
        {
            wxFileData *fd = GetFocusedListItem();
            if (!fd) return;

            wxString cmd = GetOpenWithFileCmd(fd);

            if (!cmd.IsEmpty())
            {
                long ret = wxExecute(cmd, wxEXEC_ASYNC, nullptr);
                if (ret == 0)
                {
                    wxMessageBox(wxT("Error running program"), wxT("Error running program"),
                                 wxOK, this);
                }
            }
            break;
        }
        case ID_wxFILEBROWSER_LIST_MENU_VIEW_FILE :
        {
            wxFileData *fd = GetFocusedListItem();
            if (!fd) return;

            wxLogNull logNull;
            wxImage image(fd->GetFilePath());
            if (image.Ok())
            {
                wxDialog dialog(this, -1, wxT("Preview : ") + fd->GetFilePath(),
                                wxDefaultPosition, wxDefaultSize,
                                wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER );

                wxScrolledWindow *scrWin = new wxScrolledWindow(&dialog, -1);
                new wxStaticBitmap(scrWin, -1, wxBitmap(image));

                // not necessary in MSW
                int ext = 0; //dialog.GetSize().GetWidth() - dialog.GetClientSize().GetWidth();

                wxRect clientRect = wxGetClientDisplayRect();
                wxRect imageRect(0, 0, image.GetWidth()+ext, image.GetHeight()+ext);
                clientRect.Intersect(imageRect);
                dialog.SetClientSize(clientRect.width, clientRect.height);

                scrWin->SetScrollbars(1, 1, image.GetWidth(), image.GetHeight());

                dialog.ShowModal();
                break;
            }
            else  // view as text
            {

                wxFrame* frame = new wxFrame(this, wxID_ANY, wxT("Text Viewer"));
                wxTextCtrl* textCtrl = new wxTextCtrl(frame, wxID_ANY, wxT(""),
                                             wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_RICH|wxTE_READONLY);

                wxString s;
                wxFileInputStream inputStream(fd->GetFilePath());
                wxTextInputStream textStream(inputStream);
                while (!inputStream.Eof())
                {
                    s += textStream.ReadLine() + wxT("\n");
                    if (s.Length() > 1000000)
                    {
                        wxMessageBox(wxT("This file is too large for the text viewer."),
                                     wxT("File truncated"), wxOK, this);
                        break;
                    }
                }

                textCtrl->AppendText(s);
                frame->Show(true);

/*
                wxSTEditorOptions steOptions(STE_DEFAULT_OPTIONS);
                steOptions.GetMenuManager()->CreateForNotebook();
#if wxUSE_CONFIG
                wxConfigBase *config = wxConfigBase::Get(false); // don't create
                if (config)
                    steOptions.LoadConfig(*config);
#endif //wxUSE_CONFIG

                wxSTEditorFrame *editor = new wxSTEditorFrame(this, wxID_ANY, wxT("Editor"));
                editor->CreateOptions(steOptions);
                editor->GetEditor()->LoadFile(fd->GetFilePath());
                editor->Centre();
                editor->Show(true);
*/
            }
            break;
        }
        case ID_wxFILEBROWSER_LIST_MENU_PROPERITES :
        {
            wxFileData *fd = GetFocusedListItem();
            if (!fd) return;

            ShowPropertiesDialog(*fd);
            break;
        }
        case ID_wxFILEBROWSER_SHOW_HIDDEN    : ShowHidden(event.IsChecked()); break;
        case ID_wxFILEBROWSER_SHOW_FOLDERS   : ShowFolders(event.IsChecked()); break;
        case ID_wxFILEBROWSER_SPLIT_VERTICAL : SplitVertical(event.IsChecked()); break;

        default : break;
    }
}

void wxFileBrowser::OnViewButtons(wxCommandEvent &event)
{
    switch (event.GetId())
    {
        case ID_wxFILEBROWSER_VIEW_BUTTON :
        {
            wxWindow *win = (wxWindow*)event.GetEventObject();
            if (win)
                win->PopupMenu(m_viewMenu, wxPoint(0, win->GetSize().y));
/*
            wxToolBar *win = (wxToolBar*)event.GetEventObject();
            {
                wxSize toolSize(win->GetToolSize());
                wxSize marginSize(win->GetMargins());
                int pack = win->GetToolPacking();
                int sep = win->GetToolSeparation();
                wxPrintf(wxT(" %d %d, %d %d, %d %d\n"), toolSize.x, toolSize.y, marginSize.x, marginSize.y, pack, sep);
                win->PopupMenu(m_viewMenu,
                               wxPoint(toolSize.x*5 + 2*marginSize.x,
                                       win->GetSize().y-marginSize.y));
            }
*/
            break;
        }
        case ID_wxFILEBROWSER_VIEW_TREE :
        case ID_wxFILEBROWSER_VIEW_LIST :
        case ID_wxFILEBROWSER_VIEW_DETAILS :
        case ID_wxFILEBROWSER_VIEW_SMALL_ICON :
        case ID_wxFILEBROWSER_VIEW_LARGE_ICON :
        case ID_wxFILEBROWSER_VIEW_PREVIEW :
        {
            long style = MenuIDToFBStyle(event.GetId());
            style |= (m_browser_style & (~wxFILEBROWSER_VIEW_MASK));
            SetBrowserStyle(style);
            break;
        }
        default : event.Skip(); break;
    }
}

void wxFileBrowser::OnPathCombo(wxCommandEvent &event)
{
    // gtk sends events for just dropping the box
    if (m_pathComboSelection == event.GetSelection())
        return;

    m_pathComboSelection = event.GetSelection();

    // be overly cautious, GTK combo sends events and in some cases leads to a loop
    //  it also doesn't like to have selection changed inside this handler
    wxCommandEvent setevent( wxEVT_COMMAND_MENU_SELECTED, ID_wxFILEBROWSER_COMBOSETPATH );
    setevent.SetString(event.GetString());
    GetEventHandler()->AddPendingEvent(setevent);
}
void wxFileBrowser::OnPathComboEnter(wxCommandEvent &WXUNUSED(event))
{
    OpenFilePath(m_pathCombo->GetValue());
}
void wxFileBrowser::OnSetPath( wxCommandEvent &event )
{
    SetPath(event.GetString());
    m_pathComboSelection = m_pathCombo->GetSelection();
}

bool wxFileBrowser::OpenFilePath(const wxString &filePath)
{
    wxString path = filePath;

    if (path.IsEmpty() || (path.Find(wxT('|')) != wxNOT_FOUND))
        return false;

    bool want_dir = (path.Last() == wxFILE_SEP_PATH);
    if (want_dir)
        path = path.RemoveLast();

    // Get home dir of user for ~
#ifdef __UNIX__
    if (path == wxT("~"))
    {
        path = wxGetUserHome();
    }
    else if (path.BeforeFirst(wxT('/')) == wxT("~"))
    {
        path = wxGetUserHome() + path.Remove(0, 1);
    }
#endif // __UNIX__

    // deal with ../ and what not
    if (path.Contains(wxT("..")))
    {
        // path is just ".." or "../"
        if (path == wxT(".."))
            path = GetPath(false).BeforeLast(wxFILE_SEP_PATH);
        // path is "../some/dir" - note: no need to check for at least 3 chars
        else if ((path.Mid(0, 3) == wxString(wxT("..")) + wxFILE_SEP_PATH))
            path = GetPath(false).BeforeLast(wxFILE_SEP_PATH) + wxFILE_SEP_PATH + path.AfterFirst(wxFILE_SEP_PATH);
        // path is "/some/where/.."
        else if (path.Mid(path.Len()-3) == wxFILE_SEP_PATH + wxString(wxT("..")))
            path = path.BeforeLast(wxFILE_SEP_PATH).BeforeLast(wxFILE_SEP_PATH);
        else
             return false;
    }

    wxFileName filename(path);

    if (filename.DirExists())
    {
        SetPath(path);
        return true;
    }

    // They really wanted a dir, but it doesn't exist
    if (want_dir)
        return false;

    // if it's a file then just "load" it
    if (filename.FileExists())
    {
        //SetPath(filename.GetPath());

        long item = m_fileCtrl->FindItem(-1, filename.GetName(), false);
        if (item >= 0)
        {
            m_fileCtrl->SetItemState(item, wxLIST_STATE_SELECTED, wxLIST_STATE_SELECTED);
            m_fileCtrl->EnsureVisible( item );
        }

        wxFileBrowserEvent fbEvent(wxEVT_FILEBROWSER_FILE_ACTIVATED, this, GetId());
        fbEvent.SetFilePath(path);
        DoSendEvent(fbEvent);
    }

    return true;
}

bool wxFileBrowser::InsertComboItem(wxComboBox *combo, const wxString &item, int pos) const
{
    int combo_index = combo->FindString(item);

    if (combo_index == wxNOT_FOUND)
        combo->Insert(item, pos, (void*)nullptr);
    else if ((combo_index == pos) || (combo_index < pos))
        return true;
    else if (combo_index > pos)
    {
        wxString *data = (wxString*) combo->GetClientData(combo_index);
        combo->Delete(combo_index);
        combo->Insert(item, pos, (void*)data);
    }

    if (combo->GetSelection() != pos)
        combo->SetSelection(pos);

    return true;
}

bool wxFileBrowser::SetFilter( int n )
{
    wxCHECK_MSG((n>=0) && (n<int(m_filterCombo->GetCount())), false, wxT("Invalid filter item"));

    wxString filter = m_filterCombo->GetString(n);
    wxString *data = (wxString*)m_filterCombo->GetClientData(n);
    if (data && !data->IsEmpty())
        filter += wxT("|") + (*data);
    else
        filter += wxT("|") + filter;

    AddFilter(filter);
    return true;
}

bool wxFileBrowser::AddFilter(const wxString &filter_)
{
    wxString filter(filter_);
    if (filter.IsEmpty())
        filter = wxFileSelectorDefaultWildcardStr;
    if (filter.Find(wxT('|')) == wxNOT_FOUND)
        filter +=  wxT("|") + filter;

    m_filter = filter;
    InsertComboItem(m_filterCombo, filter.BeforeFirst(wxT('|')), m_init_filters);
    m_filterComboSelection = m_filterCombo->GetSelection();

    if ((m_dirCtrl->GetWindowStyleFlag() & wxDIRCTRL_DIR_ONLY) == 0)
    {
        m_ignore_tree_event = true;
        wxString currentPath = GetPath(false);
        m_dirCtrl->SetFilter(GetFilter());
        m_dirCtrl->ReCreateTree();
        m_dirCtrl->ExpandPath(currentPath);
        m_ignore_tree_event = false;
    }

    m_fileCtrl->SetWild(GetWild());

    return true;
}

bool wxFileBrowser::SetFilters(const wxString &filter, int select)
{
    wxArrayString filterNames;
    wxArrayString filterArray;
    if (filter.IsEmpty() ||
        (::wxParseCommonDialogsFilter(filter, filterNames, filterArray) == 0))
    {
        filterNames.Add(wxT("All files (")+wxString(wxFileSelectorDefaultWildcardStr)+wxT(")"));
        filterArray.Add(wxFileSelectorDefaultWildcardStr);
    }

    wxCHECK_MSG(select < int(filterNames.GetCount()), false, wxT("Invalid filter selection"));

    m_filter = filterNames[select] + wxT("|") + filterArray[select];
    m_init_filters = filterArray.GetCount();

    // delete old filters if any
    int n, count = m_filterCombo->GetCount();
    for ( n = 0; n < count; n++ )
    {
        wxString *data = (wxString*)m_filterCombo->GetClientData(n);
        delete data;
    }

    for ( n = 0; n < m_init_filters; n++ )
        m_filterCombo->Append(filterNames[n], (void*)new wxString(filterArray[n]));

    m_filterCombo->SetSelection(select);
    m_filterComboSelection = select;

    if (!m_dirCtrl) return true; // This is during creation

    return SetFilter(select);
}

void wxFileBrowser::OnFilterComboEnter(wxCommandEvent &event)
{
    if (event.GetString().Find(wxT('|')) == wxNOT_FOUND)
        AddFilter(event.GetString() + wxT("|") + event.GetString());
}
void wxFileBrowser::OnFilterCombo(wxCommandEvent &event)
{
    int sel = event.GetSelection();
    // gtk combo sends events when the combo is opened... ignore them
    if ((sel < 0) || (sel == m_filterComboSelection))
        return;

    m_filterComboSelection = sel;

    wxString filter = event.GetString();
    wxString *data = (wxString*)m_filterCombo->GetClientData(sel);
    if (data && !data->IsEmpty())
        filter += wxT("|") + (*data);
    else
        filter += wxT("|") + filter;

    // see OnPathCombo for why it's done this way
    wxCommandEvent setevent( wxEVT_COMMAND_MENU_SELECTED, ID_wxFILEBROWSER_COMBOSETFILTER );
    setevent.SetString(filter);
    setevent.SetInt(data && !data->IsEmpty() ? sel : -1);
    GetEventHandler()->AddPendingEvent(setevent);
}
void wxFileBrowser::OnSetFilter( wxCommandEvent &event )
{
    if ( event.GetInt() != -1)
        SetFilter(event.GetInt());
    else
        AddFilter(event.GetString());
}

bool wxFileBrowser::DoSendEvent(wxFileBrowserEvent &event) const
{
    return !GetEventHandler()->ProcessEvent(event); // || event.IsAllowed();
}

#if wxUSE_CONFIG
void wxFileBrowser::LoadConfig(wxConfigBase& config, bool paths, bool filters,
                               const wxString &configPath )
{
    wxString value, key;
    key = configPath + wxT("/style");
    if (config.Read(key, &value) && (!value.IsEmpty()))
    {
        long lvalue = 0;
        if (value.ToLong(&lvalue))
        {
            SetBrowserStyle(lvalue);
        }
    }

    if (paths)
    {
        int n = 0;
        key = configPath + wxString::Format(wxT("/dir%d"), 1+n);
        while ((n < 21) && config.Read(key, &value) && (!value.IsEmpty()))
        {
            value.Trim(false).Trim(true);
            if (!value.IsEmpty())
            {
                if (value.Last() != wxFILE_SEP_PATH)
                    value += wxFILE_SEP_PATH;
                //if (wxDirExists(value))
                {
                    if (m_pathCombo->FindString(value) == wxNOT_FOUND)
                        m_pathCombo->Append(value, (void*)nullptr);
                }
            }
            n++;
            key = configPath + wxString::Format(wxT("/dir%d"), 1+n);
            value = wxEmptyString;
        }
    }
    if (filters)
    {
        int n = 0;
        key = configPath + wxString::Format(wxT("/filter%d"), 1+n);
        while ((n < 21) && config.Read(key, &value) && (!value.IsEmpty()))
        {
            value.Trim(false).Trim(true);
            if (!value.IsEmpty())
            {
                if (m_filterCombo->FindString(value) == wxNOT_FOUND)
                    m_filterCombo->Append(value, (void*)nullptr);
            }
            n++;
            key = configPath + wxString::Format(wxT("/filter%d"), 1+n);
            value = wxEmptyString;
        }
    }
}
void wxFileBrowser::SaveConfig(wxConfigBase& config, int n_paths, int n_filters,
                               const wxString &configPath)
{
    wxString value, key;
    key = configPath + wxT("/style");
    config.Write(key, GetBrowserStyle());

    if (n_paths > 0)
    {
        int n, item = 1, count = m_pathCombo->GetCount();
        for (n = 0; (n < count) && (item < n_paths); n++)
        {
            value = m_pathCombo->GetString(n);
            if (!value.IsEmpty())
            {
                config.Write(configPath + wxString::Format(wxT("/dir%d"), item), value);
                item++;
            }
        }
    }
    if (n_filters > 0)
    {
        int n, item = 1, count = m_filterCombo->GetCount();
        for (n = 0; (n < count) && (item < n_filters); n++)
        {
            // don't save the initial filters since they are programmed in
            wxString *data = (wxString*)m_filterCombo->GetClientData(n);
            if (data) continue;

            value = m_filterCombo->GetString(n);
            if (!value.IsEmpty())
            {
                config.Write(configPath + wxString::Format(wxT("/filter%d"), item), value);
                item++;
            }
        }
    }
}
#endif // wxUSE_CONFIG

void wxFileBrowser::ShowPropertiesDialog(const wxFileData &fileData) const
{
    wxFilePropertiesDialog propDialog((wxWindow*)this, wxID_ANY,
                                    fileData,
                                    fileData.GetFileName() + wxT(" Properties"),
                                    wxDEFAULT_DIALOG_STYLE, wxDefaultPosition);

    propDialog.Centre();
    propDialog.ShowModal();
}

wxString wxFileBrowser::GetOpenWithFileCmd(wxFileData* fd) const
{
    wxCHECK_MSG(fd, wxEmptyString, wxT("Invalid wxFileData for GetOpenWithFileCmd"));
    OpenWithDialog dialog((wxWindow*)this, wxID_ANY, *fd);
    dialog.ShowModal();
    return dialog.GetOpenCommand();
}

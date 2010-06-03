/***************************************************************
 * Name:      navlog_viewer_GUI_designApp.h
 * Purpose:   Defines Application Class
 * Author:    J.L. Blanco (jlblanco@ctima.uma.es)
 * Created:   2010-02-03
 * Copyright: J.L. Blanco (http://www.isa.uma.es/jlblanco)
 * License:
 **************************************************************/

#ifndef NAVLOG_VIEWER_GUI_DESIGNAPP_H
#define NAVLOG_VIEWER_GUI_DESIGNAPP_H

#include <wx/app.h>

class navlog_viewer_GUI_designApp : public wxApp
{
    public:
        virtual bool OnInit();
		virtual int OnRun();
};

#endif // NAVLOG_VIEWER_GUI_DESIGNAPP_H

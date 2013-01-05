/***************************************************************
 * Name:      robotic_arm_kinematicsApp.cpp
 * Purpose:   Code for Application Class
 * Author:     ()
 * Created:   2013-01-05
 * Copyright:  ()
 * License:
 **************************************************************/

#include "robotic_arm_kinematicsApp.h"

//(*AppHeaders
#include "robotic_arm_kinematicsMain.h"
#include <wx/image.h>
//*)

IMPLEMENT_APP(robotic_arm_kinematicsApp);

bool robotic_arm_kinematicsApp::OnInit()
{
    //(*AppInitialize
    bool wxsOK = true;
    wxInitAllImageHandlers();
    if ( wxsOK )
    {
    	robotic_arm_kinematicsFrame* Frame = new robotic_arm_kinematicsFrame(0);
    	Frame->Show();
    	SetTopWindow(Frame);
    }
    //*)
    return wxsOK;

}

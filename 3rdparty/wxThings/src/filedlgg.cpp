//////////////////////////////////////////////////////////////////////////////
// Name:        filedlgg.cpp
// Purpose:     wxGenericFileDialog stub to include the source
// Author:      John Labenski, Robert Roebling
// Modified by:
// Created:     12/12/98
// RCS-ID:
// Copyright:   (c) Robert Roebling
// Licence:     wxWindows licence
/////////////////////////////////////////////////////////////////////////////

// MSVC cannot use environment var $(WXWIN)/src/generic/filedlgg.cpp in their
// project files to allow you to compile the generic file dialog.
// You have to specify the relative or absolute path to the wxWidgets
// distribution which is annoying since everyone will probably have it in a
// different place. By #including the source from this file we can avoid this
// and use the $(WXWIN) environment var in the -I include path.


#include "wx/defs.h"
#include "wx/filedlg.h"
//#include "wx/generic/filedlgg.h"

// If we didn't include the generic filedlgg then include the source here
// NOTE: There may be special cases where more sophisticated tests may be
//       required.

#ifndef _WX_FILEDLGG_H_

    // Typically we have $(WXWIN)/include in the search path so this should
    // find the path to filedlgg correctly.
    #include "../src/generic/filedlgg.cpp"

#endif //_WX_FILEDLGG_H_

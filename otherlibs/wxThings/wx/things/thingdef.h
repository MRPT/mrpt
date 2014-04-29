/////////////////////////////////////////////////////////////////////////////
// Name:        thingdef.h
// Purpose:     Definitions for wxThings
// Author:      John Labenski
// Modified by:
// Created:     1/08/1999
// RCS-ID:      $Id: thingdef.h,v 1.3 2006/08/24 19:35:58 jrl1 Exp $
// Copyright:   (c) John Labenski
// Licence:     wxWidgets licence
/////////////////////////////////////////////////////////////////////////////

#ifndef __WX_THINGDEF_H__
#define __WX_THINGDEF_H__

#include "wx/defs.h"

//-----------------------------------------------------------------------------
// The version of wxThings
//-----------------------------------------------------------------------------

#define WXTHINGS_MAJOR_VERSION      1
#define WXTHINGS_MINOR_VERSION      0
#define WXTHINGS_RELEASE_VERSION    0
#define WXTHINGS_SUBRELEASE_VERSION 0
#define WXTHINGS_VERSION_STRING    _T("wxThings 1.0.0")

// For non-Unix systems (i.e. when building without a configure script),
// users of this component can use the following macro to check if the
// current version is at least major.minor.release
#define wxCHECK_WXTHINGS_VERSION(major,minor,release) \
    (WXTHINGS_MAJOR_VERSION > (major) || \
    (WXTHINGS_MAJOR_VERSION == (major) && WXTHINGS_MINOR_VERSION > (minor)) || \
    (WXTHINGS_MAJOR_VERSION == (major) && WXTHINGS_MINOR_VERSION == (minor) && WXTHINGS_RELEASE_VERSION >= (release)))

// ----------------------------------------------------------------------------
// DLLIMPEXP macros
// ----------------------------------------------------------------------------

// These are our DLL macros (see the contrib libs like wxPlot)
#ifdef WXMAKINGDLL_THINGS
    #define WXDLLIMPEXP_THINGS WXEXPORT
    #define WXDLLIMPEXP_DATA_THINGS(type) WXEXPORT type
#elif 0 && defined(WXUSINGDLL)   // ******** JLBC for ROADBOTGUI ******** 
    #define WXDLLIMPEXP_THINGS WXIMPORT
    #define WXDLLIMPEXP_DATA_THINGS(type) WXIMPORT type
#else // not making nor using DLL
    #define WXDLLIMPEXP_THINGS
    #define WXDLLIMPEXP_DATA_THINGS(type) type
#endif

// ----------------------------------------------------------------------------
// wxWidgets backwards compatibility macros
// ----------------------------------------------------------------------------

#include "wx/dynarray.h"
#ifndef WX_DECLARE_OBJARRAY_WITH_DECL // for wx2.4 backwards compatibility
    #define WX_DECLARE_OBJARRAY_WITH_DECL(T, name, expmode) WX_DECLARE_USER_EXPORTED_OBJARRAY(T, name, WXDLLIMPEXP_THINGS)
#endif

#endif  // __WX_THINGDEF_H__

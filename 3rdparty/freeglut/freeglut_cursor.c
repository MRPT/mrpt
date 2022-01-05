/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+ */

#include <GL/freeglut.h>
#include "freeglut_internal.h"

#if TARGET_HOST_UNIX_X11
  #include <X11/cursorfont.h>
#endif

/*
 * TODO BEFORE THE STABLE RELEASE:
 *  glutSetCursor()     -- Win32 mappings are incomplete.
 *
 * It would be good to use custom mouse cursor shapes, and introduce
 * an option to display them using glBitmap() and/or texture mapping,
 * apart from the windowing system version.
 */

/* -- PRIVATE FUNCTIONS --------------------------------------------------- */

#if TARGET_HOST_UNIX_X11
/*
 * A factory method for an empty cursor
 */
static Cursor getEmptyCursor( void )
{
    static Cursor cursorNone = None;
    if( cursorNone == None ) {
        char cursorNoneBits[ 32 ];
        XColor dontCare;
        Pixmap cursorNonePixmap;
        memset( cursorNoneBits, 0, sizeof( cursorNoneBits ) );
        memset( &dontCare, 0, sizeof( dontCare ) );
        cursorNonePixmap = XCreateBitmapFromData ( fgDisplay.Display,
                                                   fgDisplay.RootWindow,
                                                   cursorNoneBits, 16, 16 );
        if( cursorNonePixmap != None ) {
            cursorNone = XCreatePixmapCursor( fgDisplay.Display,
                                              cursorNonePixmap, cursorNonePixmap,
                                              &dontCare, &dontCare, 0, 0 );
            XFreePixmap( fgDisplay.Display, cursorNonePixmap );
        }
    }
    return cursorNone;
}

typedef struct tag_cursorCacheEntry cursorCacheEntry;
struct tag_cursorCacheEntry {
    unsigned int cursorShape;    /* an XC_foo value */
    Cursor cachedCursor;         /* None if the corresponding cursor has
                                    not been created yet */
};

/*
 * Note: The arrangement of the table below depends on the fact that
 * the "normal" GLUT_CURSOR_* values start a 0 and are consecutive.
 */
static cursorCacheEntry cursorCache[] = {
    { XC_arrow,               None }, /* GLUT_CURSOR_RIGHT_ARROW */
    { XC_top_left_arrow,      None }, /* GLUT_CURSOR_LEFT_ARROW */
    { XC_hand1,               None }, /* GLUT_CURSOR_INFO */
    { XC_pirate,              None }, /* GLUT_CURSOR_DESTROY */
    { XC_question_arrow,      None }, /* GLUT_CURSOR_HELP */
    { XC_exchange,            None }, /* GLUT_CURSOR_CYCLE */
    { XC_spraycan,            None }, /* GLUT_CURSOR_SPRAY */
    { XC_watch,               None }, /* GLUT_CURSOR_WAIT */
    { XC_xterm,               None }, /* GLUT_CURSOR_TEXT */
    { XC_crosshair,           None }, /* GLUT_CURSOR_CROSSHAIR */
    { XC_sb_v_double_arrow,   None }, /* GLUT_CURSOR_UP_DOWN */
    { XC_sb_h_double_arrow,   None }, /* GLUT_CURSOR_LEFT_RIGHT */
    { XC_top_side,            None }, /* GLUT_CURSOR_TOP_SIDE */
    { XC_bottom_side,         None }, /* GLUT_CURSOR_BOTTOM_SIDE */
    { XC_left_side,           None }, /* GLUT_CURSOR_LEFT_SIDE */
    { XC_right_side,          None }, /* GLUT_CURSOR_RIGHT_SIDE */
    { XC_top_left_corner,     None }, /* GLUT_CURSOR_TOP_LEFT_CORNER */
    { XC_top_right_corner,    None }, /* GLUT_CURSOR_TOP_RIGHT_CORNER */
    { XC_bottom_right_corner, None }, /* GLUT_CURSOR_BOTTOM_RIGHT_CORNER */
    { XC_bottom_left_corner,  None }  /* GLUT_CURSOR_BOTTOM_LEFT_CORNER */
};
#endif

/* -- INTERNAL FUNCTIONS ---------------------------------------------------- */

/*
 * Set the cursor image to be used for the current window
 */
void fgSetCursor ( SFG_Window *window, int cursorID )
{
#if TARGET_HOST_UNIX_X11
    {
        Cursor cursor;
        /*
         * XXX FULL_CROSSHAIR demotes to plain CROSSHAIR. Old GLUT allows
         * for this, but if there is a system that easily supports a full-
         * window (or full-screen) crosshair, we might consider it.
         */
        int cursorIDToUse =
            ( cursorID == GLUT_CURSOR_FULL_CROSSHAIR ) ? GLUT_CURSOR_CROSSHAIR : cursorID;

        if( ( cursorIDToUse >= 0 ) &&
            ( cursorIDToUse < sizeof( cursorCache ) / sizeof( cursorCache[0] ) ) ) {
            cursorCacheEntry *entry = &cursorCache[ cursorIDToUse ];
            if( entry->cachedCursor == None ) {
                entry->cachedCursor =
                    XCreateFontCursor( fgDisplay.Display, entry->cursorShape );
            }
            cursor = entry->cachedCursor;
        } else {
            switch( cursorIDToUse )
            {
            case GLUT_CURSOR_NONE:
                cursor = getEmptyCursor( );
                break;

            case GLUT_CURSOR_INHERIT:
                cursor = None;
                break;

            default:
                fgError( "Unknown cursor type: %d", cursorIDToUse );
                return;
            }
        }

        if ( ( cursorIDToUse != GLUT_CURSOR_NONE ) && ( cursor == None ) ) {
            fgError( "Failed to create cursor" );
        }
        XDefineCursor( fgDisplay.Display,
                       window->Window.Handle, cursor );
    }

#elif TARGET_HOST_WIN32 || TARGET_HOST_WINCE

#ifndef GCL_HCURSOR  // Added by JLBC for MRPT
			#define GCL_HCURSOR (-12)
#endif

    /*
     * This is a temporary solution only...
     */
    /* Set the cursor AND change it for this window class. */
#       define MAP_CURSOR(a,b)                                   \
        case a:                                                  \
            SetCursor( LoadCursor( NULL, b ) );                  \
            SetClassLong( window->Window.Handle,                 \
                          GCL_HCURSOR,                           \
                          ( LONG )LoadCursor( NULL, b ) );       \
        break;

    /* Nuke the cursor AND change it for this window class. */
#       define ZAP_CURSOR(a,b)                                   \
        case a:                                                  \
            SetCursor( NULL );                                   \
            SetClassLong( window->Window.Handle,                 \
                          GCL_HCURSOR, ( LONG )NULL );           \
        break;

#ifdef _MSC_VER
#pragma warning(disable:4311)
#endif

    switch( cursorID )
    {
        MAP_CURSOR( GLUT_CURSOR_RIGHT_ARROW,         IDC_ARROW     );
        MAP_CURSOR( GLUT_CURSOR_LEFT_ARROW,          IDC_ARROW     );
        MAP_CURSOR( GLUT_CURSOR_INFO,                IDC_HELP      );
        MAP_CURSOR( GLUT_CURSOR_DESTROY,             IDC_CROSS     );
        MAP_CURSOR( GLUT_CURSOR_HELP,                IDC_HELP      );
        MAP_CURSOR( GLUT_CURSOR_CYCLE,               IDC_SIZEALL   );
        MAP_CURSOR( GLUT_CURSOR_SPRAY,               IDC_CROSS     );
        MAP_CURSOR( GLUT_CURSOR_WAIT,                IDC_WAIT      );
        MAP_CURSOR( GLUT_CURSOR_TEXT,                IDC_IBEAM     );
        MAP_CURSOR( GLUT_CURSOR_CROSSHAIR,           IDC_CROSS     );
        MAP_CURSOR( GLUT_CURSOR_UP_DOWN,             IDC_SIZENS    );
        MAP_CURSOR( GLUT_CURSOR_LEFT_RIGHT,          IDC_SIZEWE    );
        MAP_CURSOR( GLUT_CURSOR_TOP_SIDE,            IDC_ARROW     ); /* XXX ToDo */
        MAP_CURSOR( GLUT_CURSOR_BOTTOM_SIDE,         IDC_ARROW     ); /* XXX ToDo */
        MAP_CURSOR( GLUT_CURSOR_LEFT_SIDE,           IDC_ARROW     ); /* XXX ToDo */
        MAP_CURSOR( GLUT_CURSOR_RIGHT_SIDE,          IDC_ARROW     ); /* XXX ToDo */
        MAP_CURSOR( GLUT_CURSOR_TOP_LEFT_CORNER,     IDC_SIZENWSE  );
        MAP_CURSOR( GLUT_CURSOR_TOP_RIGHT_CORNER,    IDC_SIZENESW  );
        MAP_CURSOR( GLUT_CURSOR_BOTTOM_RIGHT_CORNER, IDC_SIZENWSE  );
        MAP_CURSOR( GLUT_CURSOR_BOTTOM_LEFT_CORNER,  IDC_SIZENESW  );
        MAP_CURSOR( GLUT_CURSOR_INHERIT,             IDC_ARROW     ); /* XXX ToDo */
        ZAP_CURSOR( GLUT_CURSOR_NONE,                NULL          );
        MAP_CURSOR( GLUT_CURSOR_FULL_CROSSHAIR,      IDC_CROSS     ); /* XXX ToDo */

    default:
        fgError( "Unknown cursor type: %d", cursorID );
        break;
    }
#endif

    window->State.Cursor = cursorID;
}

/* -- INTERFACE FUNCTIONS -------------------------------------------------- */

/*
 * Set the cursor image to be used for the current window
 */
void FGAPIENTRY glutSetCursor( int cursorID )
{
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutSetCursor" );
    FREEGLUT_EXIT_IF_NO_WINDOW ( "glutSetCursor" );

    fgSetCursor ( fgStructure.CurrentWindow, cursorID );
}

/*
 * Moves the mouse pointer to given window coordinates
 */
void FGAPIENTRY glutWarpPointer( int x, int y )
{
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutWarpPointer" );
    FREEGLUT_EXIT_IF_NO_WINDOW ( "glutWarpPointer" );

#if TARGET_HOST_UNIX_X11

    XWarpPointer(
        fgDisplay.Display,
        None,
        fgStructure.CurrentWindow->Window.Handle,
        0, 0, 0, 0,
        x, y
    );
    /* Make the warp visible immediately. */
    XFlush( fgDisplay.Display );

#elif TARGET_HOST_WIN32 || TARGET_HOST_WINCE

    {
        POINT coords;
        coords.x = x;
        coords.y = y;

        /* ClientToScreen() translates {coords} for us. */
        ClientToScreen( fgStructure.CurrentWindow->Window.Handle, &coords );
        SetCursorPos( coords.x, coords.y );
    }

#endif
}

/*** END OF FILE ***/

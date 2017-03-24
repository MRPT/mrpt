/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <GL/freeglut.h>
#include "freeglut_internal.h"

/* -- INTERFACE FUNCTIONS -------------------------------------------------- */

/*
 * Marks the current window to have the redisplay performed when possible...
 */
void FGAPIENTRY glutPostRedisplay( void )
{
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutPostRedisplay" );
    FREEGLUT_EXIT_IF_NO_WINDOW ( "glutPostRedisplay" );
    fgStructure.CurrentWindow->State.Redisplay = GL_TRUE;
}

/*
 * Swaps the buffers for the current window (if any)
 */
void FGAPIENTRY glutSwapBuffers( void )
{
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutSwapBuffers" );
    FREEGLUT_EXIT_IF_NO_WINDOW ( "glutSwapBuffers" );

    glFlush( );
    if( ! fgStructure.CurrentWindow->Window.DoubleBuffered )
        return;

#if TARGET_HOST_UNIX_X11
    glXSwapBuffers( fgDisplay.Display, fgStructure.CurrentWindow->Window.Handle );
#elif TARGET_HOST_WIN32 || TARGET_HOST_WINCE
    SwapBuffers( fgStructure.CurrentWindow->Window.Device );
#endif

    /* GLUT_FPS env var support */
    if( fgState.FPSInterval )
    {
        GLint t = glutGet( GLUT_ELAPSED_TIME );
        fgState.SwapCount++;
        if( fgState.SwapTime == 0 )
            fgState.SwapTime = t;
        else if( t - fgState.SwapTime > fgState.FPSInterval )
        {
            float time = 0.001f * ( t - fgState.SwapTime );
            float fps = ( float )fgState.SwapCount / time;
            fprintf( stderr,
                     "freeglut: %d frames in %.2f seconds = %.2f FPS\n",
                     fgState.SwapCount, time, fps );
            fgState.SwapTime = t;
            fgState.SwapCount = 0;
        }
    }
}

/*
 * Mark appropriate window to be displayed
 */
void FGAPIENTRY glutPostWindowRedisplay( int windowID )
{
    SFG_Window* window;

    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutPostWindowRedisplay" );
    window = fgWindowByID( windowID );
    freeglut_return_if_fail( window );
    window->State.Redisplay = GL_TRUE;
}

/*** END OF FILE ***/

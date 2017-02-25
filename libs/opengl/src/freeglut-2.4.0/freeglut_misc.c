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

/*
 * TODO BEFORE THE STABLE RELEASE:
 *
 *  glutSetColor()     --
 *  glutGetColor()     --
 *  glutCopyColormap() --
 *  glutSetKeyRepeat() -- this is evil and should be removed from API
 */

/* -- INTERFACE FUNCTIONS -------------------------------------------------- */

/*
 * This functions checks if an OpenGL extension is supported or not
 *
 * XXX Wouldn't this be simpler and clearer if we used strtok()?
 */
int FGAPIENTRY glutExtensionSupported( const char* extension )
{
  const char *extensions, *start;
  const int len = (int)strlen( extension );

  /* Make sure there is a current window, and thus a current context available */
  FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutExtensionSupported" );
  freeglut_return_val_if_fail( fgStructure.CurrentWindow != NULL, 0 );

  if (strchr(extension, ' '))
    return 0;
  start = extensions = (const char *) glGetString(GL_EXTENSIONS);

  /* XXX consider printing a warning to stderr that there's no current
   * rendering context.
   */
  freeglut_return_val_if_fail( extensions != NULL, 0 );

  while (1) {
     const char *p = strstr(extensions, extension);
     if (!p)
        return 0;  /* not found */
     /* check that the match isn't a super string */
     if ((p == start || p[-1] == ' ') && (p[len] == ' ' || p[len] == 0))
        return 1;
     /* skip the false match and continue */
     extensions = p + len;
  }

  return 0 ;
}

/*
 * This function reports all the OpenGL errors that happened till now
 */
void FGAPIENTRY glutReportErrors( void )
{
    GLenum error;
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutReportErrors" );
    while( ( error = glGetError() ) != GL_NO_ERROR )
        fgWarning( "GL error: %s", gluErrorString( error ) );
}

/*
 * Control the auto-repeat of keystrokes to the current window
 */
void FGAPIENTRY glutIgnoreKeyRepeat( int ignore )
{
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutIgnoreKeyRepeat" );
    FREEGLUT_EXIT_IF_NO_WINDOW ( "glutIgnoreKeyRepeat" );

    fgStructure.CurrentWindow->State.IgnoreKeyRepeat = ignore ? GL_TRUE : GL_FALSE;
}

/*
 * Set global auto-repeat of keystrokes
 *
 * RepeatMode should be either:
 *    GLUT_KEY_REPEAT_OFF
 *    GLUT_KEY_REPEAT_ON
 *    GLUT_KEY_REPEAT_DEFAULT
 */
void FGAPIENTRY glutSetKeyRepeat( int repeatMode )
{
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutSetKeyRepeat" );

    switch( repeatMode )
    {
    case GLUT_KEY_REPEAT_OFF:
    case GLUT_KEY_REPEAT_ON:
     fgState.KeyRepeat = repeatMode;
     break;

    case GLUT_KEY_REPEAT_DEFAULT:
     fgState.KeyRepeat = GLUT_KEY_REPEAT_ON;
     break;

    default:
        fgError ("Invalid glutSetKeyRepeat mode: %d", repeatMode);
        break;
    }
}

/*
 * Forces the joystick callback to be executed
 */
void FGAPIENTRY glutForceJoystickFunc( void )
{
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutForceJoystickFunc" );
#if !TARGET_HOST_WINCE
    freeglut_return_if_fail( fgStructure.CurrentWindow != NULL );
    freeglut_return_if_fail( FETCH_WCB( *( fgStructure.CurrentWindow ), Joystick ) );
    fgJoystickPollWindow( fgStructure.CurrentWindow );
#endif /* !TARGET_HOST_WINCE */
}

/*
 *
 */
void FGAPIENTRY glutSetColor( int nColor, GLfloat red, GLfloat green, GLfloat blue )
{
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutSetColor" );
    /* We really need to do something here. */
}

/*
 *
 */
GLfloat FGAPIENTRY glutGetColor( int color, int component )
{
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutGetColor" );
    /* We really need to do something here. */
    return( 0.0f );
}

/*
 *
 */
void FGAPIENTRY glutCopyColormap( int window )
{
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutCopyColormap" );
    /* We really need to do something here. */
}

/*** END OF FILE ***/

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

/*
 * NOTE: functions declared in this file probably will not be implemented.
 */

/* -- INTERFACE FUNCTIONS -------------------------------------------------- */

int  FGAPIENTRY glutVideoResizeGet( GLenum eWhat )            {    return( 0x00 );    }
void FGAPIENTRY glutSetupVideoResizing( void )                { /* Not implemented */ }
void FGAPIENTRY glutStopVideoResizing( void )                 { /* Not implemented */ }
void FGAPIENTRY glutVideoResize( int x, int y, int w, int h ) { /* Not implemented */ }
void FGAPIENTRY glutVideoPan( int x, int y, int w, int h )    { /* Not implemented */ }

/*** END OF FILE ***/








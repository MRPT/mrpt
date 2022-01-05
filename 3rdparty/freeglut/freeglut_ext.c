/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+ */

#define GLX_GLXEXT_PROTOTYPES
#include <GL/freeglut.h>
#include "freeglut_internal.h"

static GLUTproc fghGetProcAddress( const char* procName )
{
    /* optimization: quick initial check */
    if( strncmp( procName, "glut", 4 ) != 0 )
        return NULL;

#define CHECK_NAME(x) if( strcmp( procName, #x ) == 0) return (GLUTproc)x;
    CHECK_NAME(glutInit);
    CHECK_NAME(glutInitDisplayMode);
    CHECK_NAME(glutInitDisplayString);
    CHECK_NAME(glutInitWindowPosition);
    CHECK_NAME(glutInitWindowSize);
    CHECK_NAME(glutMainLoop);
    CHECK_NAME(glutCreateWindow);
    CHECK_NAME(glutCreateSubWindow);
    CHECK_NAME(glutDestroyWindow);
    CHECK_NAME(glutPostRedisplay);
    CHECK_NAME(glutPostWindowRedisplay);
    CHECK_NAME(glutSwapBuffers);
    CHECK_NAME(glutGetWindow);
    CHECK_NAME(glutSetWindow);
    CHECK_NAME(glutSetWindowTitle);
    CHECK_NAME(glutSetIconTitle);
    CHECK_NAME(glutPositionWindow);
    CHECK_NAME(glutReshapeWindow);
    CHECK_NAME(glutPopWindow);
    CHECK_NAME(glutPushWindow);
    CHECK_NAME(glutIconifyWindow);
    CHECK_NAME(glutShowWindow);
    CHECK_NAME(glutHideWindow);
    CHECK_NAME(glutFullScreen);
    CHECK_NAME(glutSetCursor);
    CHECK_NAME(glutWarpPointer);
    CHECK_NAME(glutEstablishOverlay);
    CHECK_NAME(glutRemoveOverlay);
    CHECK_NAME(glutUseLayer);
    CHECK_NAME(glutPostOverlayRedisplay);
    CHECK_NAME(glutPostWindowOverlayRedisplay);
    CHECK_NAME(glutShowOverlay);
    CHECK_NAME(glutHideOverlay);
    CHECK_NAME(glutCreateMenu);
    CHECK_NAME(glutDestroyMenu);
    CHECK_NAME(glutGetMenu);
    CHECK_NAME(glutSetMenu);
    CHECK_NAME(glutAddMenuEntry);
    CHECK_NAME(glutAddSubMenu);
    CHECK_NAME(glutChangeToMenuEntry);
    CHECK_NAME(glutChangeToSubMenu);
    CHECK_NAME(glutRemoveMenuItem);
    CHECK_NAME(glutAttachMenu);
    CHECK_NAME(glutDetachMenu);
    CHECK_NAME(glutDisplayFunc);
    CHECK_NAME(glutReshapeFunc);
    CHECK_NAME(glutKeyboardFunc);
    CHECK_NAME(glutMouseFunc);
    CHECK_NAME(glutMotionFunc);
    CHECK_NAME(glutPassiveMotionFunc);
    CHECK_NAME(glutEntryFunc);
    CHECK_NAME(glutVisibilityFunc);
    CHECK_NAME(glutIdleFunc);
    CHECK_NAME(glutTimerFunc);
    CHECK_NAME(glutMenuStateFunc);
    CHECK_NAME(glutSpecialFunc);
    CHECK_NAME(glutSpaceballMotionFunc);
    CHECK_NAME(glutSpaceballRotateFunc);
    CHECK_NAME(glutSpaceballButtonFunc);
    CHECK_NAME(glutButtonBoxFunc);
    CHECK_NAME(glutDialsFunc);
    CHECK_NAME(glutTabletMotionFunc);
    CHECK_NAME(glutTabletButtonFunc);
    CHECK_NAME(glutMenuStatusFunc);
    CHECK_NAME(glutOverlayDisplayFunc);
    CHECK_NAME(glutWindowStatusFunc);
    CHECK_NAME(glutKeyboardUpFunc);
    CHECK_NAME(glutSpecialUpFunc);
#if !TARGET_HOST_WINCE
    CHECK_NAME(glutJoystickFunc);
#endif /* !TARGET_HOST_WINCE */
    CHECK_NAME(glutSetColor);
    CHECK_NAME(glutGetColor);
    CHECK_NAME(glutCopyColormap);
    CHECK_NAME(glutGet);
    CHECK_NAME(glutDeviceGet);
    CHECK_NAME(glutExtensionSupported);
    CHECK_NAME(glutGetModifiers);
    CHECK_NAME(glutLayerGet);
    CHECK_NAME(glutBitmapCharacter);
    CHECK_NAME(glutBitmapWidth);
    CHECK_NAME(glutStrokeCharacter);
    CHECK_NAME(glutStrokeWidth);
    CHECK_NAME(glutBitmapLength);
    CHECK_NAME(glutStrokeLength);
    CHECK_NAME(glutWireSphere);
    CHECK_NAME(glutSolidSphere);
    CHECK_NAME(glutWireCone);
    CHECK_NAME(glutSolidCone);
    CHECK_NAME(glutWireCube);
    CHECK_NAME(glutSolidCube);
    CHECK_NAME(glutWireTorus);
    CHECK_NAME(glutSolidTorus);
    CHECK_NAME(glutWireDodecahedron);
    CHECK_NAME(glutSolidDodecahedron);
    CHECK_NAME(glutWireTeapot);
    CHECK_NAME(glutSolidTeapot);
    CHECK_NAME(glutWireOctahedron);
    CHECK_NAME(glutSolidOctahedron);
    CHECK_NAME(glutWireTetrahedron);
    CHECK_NAME(glutSolidTetrahedron);
    CHECK_NAME(glutWireIcosahedron);
    CHECK_NAME(glutSolidIcosahedron);
    CHECK_NAME(glutVideoResizeGet);
    CHECK_NAME(glutSetupVideoResizing);
    CHECK_NAME(glutStopVideoResizing);
    CHECK_NAME(glutVideoResize);
    CHECK_NAME(glutVideoPan);
    CHECK_NAME(glutReportErrors);
    CHECK_NAME(glutIgnoreKeyRepeat);
    CHECK_NAME(glutSetKeyRepeat);
#if !TARGET_HOST_WINCE
    CHECK_NAME(glutForceJoystickFunc);
    CHECK_NAME(glutGameModeString);
    CHECK_NAME(glutEnterGameMode);
    CHECK_NAME(glutLeaveGameMode);
    CHECK_NAME(glutGameModeGet);
#endif /* !TARGET_HOST_WINCE */
    /* freeglut extensions */
    CHECK_NAME(glutMainLoopEvent);
    CHECK_NAME(glutLeaveMainLoop);
    CHECK_NAME(glutCloseFunc);
    CHECK_NAME(glutWMCloseFunc);
    CHECK_NAME(glutMenuDestroyFunc);
    CHECK_NAME(glutSetOption);
    CHECK_NAME(glutSetWindowData);
    CHECK_NAME(glutGetWindowData);
    CHECK_NAME(glutSetMenuData);
    CHECK_NAME(glutGetMenuData);
    CHECK_NAME(glutBitmapHeight);
    CHECK_NAME(glutStrokeHeight);
    CHECK_NAME(glutBitmapString);
    CHECK_NAME(glutStrokeString);
    CHECK_NAME(glutWireRhombicDodecahedron);
    CHECK_NAME(glutSolidRhombicDodecahedron);
    CHECK_NAME(glutWireSierpinskiSponge);
    CHECK_NAME(glutSolidSierpinskiSponge);
    CHECK_NAME(glutWireCylinder);
    CHECK_NAME(glutSolidCylinder);
    CHECK_NAME(glutGetProcAddress);
    CHECK_NAME(glutMouseWheelFunc);
#undef CHECK_NAME

    return NULL;
}


GLUTproc FGAPIENTRY
glutGetProcAddress( const char *procName )
{
    GLUTproc p;
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutGetProcAddress" );

    /* Try GLUT functions first */
    p = fghGetProcAddress( procName );
    if( p != NULL )
        return p;

    /* Try core GL functions */
#if TARGET_HOST_WIN32 || TARGET_HOST_WINCE
    return(GLUTproc)wglGetProcAddress( ( LPCSTR )procName );
#elif TARGET_HOST_UNIX_X11 && defined( GLX_ARB_get_proc_address )
    return(GLUTproc)glXGetProcAddressARB( ( const GLubyte * )procName );
#else
    return NULL;
#endif
}

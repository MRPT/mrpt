REM Script for building wxWidgets with MS Visual C++
nmake -f makefile.vc BUILD=release SHARED=1 RUNTIME_LIBS=dynamic DEBUG_INFO=0 VENDOR=mrpt USE_OPENGL=1 TARGET_CPU=amd64
nmake -f makefile.vc BUILD=debug SHARED=1 RUNTIME_LIBS=dynamic DEBUG_INFO=1 VENDOR=mrpt USE_OPENGL=1 TARGET_CPU=amd64
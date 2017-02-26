/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*
 * This file is necessary for the *nix version of "freeglut" because the
 * original GLUT defined its font variables in rather an unusual way.
 * Publicly, in "glut.h", they were defined as "void *".  Privately,
 * in one of the source code files, they were defined as pointers to a
 * structure.  Most compilers and linkers are satisfied with the "void *"
 * and don't go any farther, but some of them balked.  In particular,
 * when compiling with "freeglut" and then trying to run using the GLUT
 * ".so" library, some of them would give an error.  So we are having to
 * create this file to define the variables as pointers to an unusual
 * structure to match GLUT.
 */

#include "freeglut_internal.h"

#if TARGET_HOST_UNIX_X11

struct freeglutStrokeFont
{
  const char *name ;
  int num_chars ;
  void *ch ;
  float top ;
  float bottom ;
};

struct freeglutBitmapFont
{
  const char *name ;
  const int num_chars ;
  const int first ;
  const void *ch ;
};


struct freeglutStrokeFont glutStrokeRoman ;
struct freeglutStrokeFont glutStrokeMonoRoman ;

struct freeglutBitmapFont glutBitmap9By15 ;
struct freeglutBitmapFont glutBitmap8By13 ;
struct freeglutBitmapFont glutBitmapTimesRoman10 ;
struct freeglutBitmapFont glutBitmapTimesRoman24 ;
struct freeglutBitmapFont glutBitmapHelvetica10 ;
struct freeglutBitmapFont glutBitmapHelvetica12 ;
struct freeglutBitmapFont glutBitmapHelvetica18 ;

#endif


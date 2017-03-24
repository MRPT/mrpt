/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_UTILS_BOOSTJOIN_H
#define  MRPT_UTILS_BOOSTJOIN_H

// Use a few lines from boost here
//  (C) Copyright John Maddock 2001 - 2003.
//  (C) Copyright Darin Adler 2001.
//  (C) Copyright Peter Dimov 2001.
//  (C) Copyright Bill Kempf 2002.
//  (C) Copyright Jens Maurer 2002.
//  (C) Copyright David Abrahams 2002 - 2003.
//  (C) Copyright Gennaro Prota 2003.
//  (C) Copyright Eric Friedman 2003.
// License: Boost Software License   See LICENSE.txt for the full license.
#ifndef BOOST_JOIN
#define BOOST_JOIN( X, Y ) BOOST_DO_JOIN( X, Y )
#define BOOST_DO_JOIN( X, Y ) BOOST_DO_JOIN2(X,Y)
#define BOOST_DO_JOIN2( X, Y ) X##Y
#endif

#endif

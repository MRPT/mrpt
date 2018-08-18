/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

/** \defgroup mrpt_img_grp [mrpt-img]

Basic computer vision data structures and tools: bitmap images, canvas, color
maps, and pinhole camera models.

[TOC]

# Library `mrpt-img`
<small> [New in MRPT 2.0.0] </small>

This C++ library is part of MRPT and can be installed in Debian-based systems
with:

		sudo apt install libmrpt-img-dev

Find below some examples of use.

## Image handling

The class mrpt::img::CImage represents a wrapper around OpenCV images, plus
extra functionality such as on-the-fly loading of images stored in disk upon
first usage. The internal `IplImage` or `cv::Mat` is always available so
OpenCV's functions can be still used to operate on MRPT images.



*/

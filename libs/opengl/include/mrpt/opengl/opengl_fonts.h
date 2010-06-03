/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#ifndef mrpt_opengl_fonts_H
#define mrpt_opengl_fonts_H


namespace mrpt
{
	namespace opengl
	{
		/** Existing fonts for 2D texts in mrpt::opengl methods.
		  * \sa mrpt::opengl::CMyGLCanvasBase::renderTextBitmap
		  */
		enum TOpenGLFont 
		{
			MRPT_GLUT_BITMAP_TIMES_ROMAN_10 = 0,
			MRPT_GLUT_BITMAP_TIMES_ROMAN_24 = 1,
			MRPT_GLUT_BITMAP_HELVETICA_10 = 2,
			MRPT_GLUT_BITMAP_HELVETICA_12 = 3,
			MRPT_GLUT_BITMAP_HELVETICA_18 = 4
		}; 
	}
}

#endif

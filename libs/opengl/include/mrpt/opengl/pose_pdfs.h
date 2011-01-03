/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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
#ifndef pose_pdfs_H
#define pose_pdfs_H

#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/opengl_frwd_decl.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuatPDF.h>

namespace mrpt
{
	namespace opengl
	{
		/** @name A set of functions to obtain a 3D representation of a pose PDF (these functions are in the mrpt-opengl library)
		    @{  */

		/** Returns a representation of a the PDF - this is just an auxiliary function, it's more natural to call
		  *    mrpt::poses::CPosePDF::getAs3DObject     */
		template <>
		CSetOfObjectsPtr OPENGL_IMPEXP posePDF2opengl<CPosePDF,CSetOfObjectsPtr>(const CPosePDF &o);

		/** Returns a representation of a the PDF - this is just an auxiliary function, it's more natural to call
		  *    mrpt::poses::CPointPDF::getAs3DObject     */
		template <>
		CSetOfObjectsPtr OPENGL_IMPEXP posePDF2opengl<CPointPDF,CSetOfObjectsPtr>(const CPointPDF &o);

		/** Returns a representation of a the PDF - this is just an auxiliary function, it's more natural to call
		  *    mrpt::poses::CPose3DPDF::getAs3DObject     */
		template <>
		CSetOfObjectsPtr OPENGL_IMPEXP posePDF2opengl<CPose3DPDF,CSetOfObjectsPtr>(const CPose3DPDF &o);

		/** Returns a representation of a the PDF - this is just an auxiliary function, it's more natural to call
		  *    mrpt::poses::CPose3DQuatPDF::getAs3DObject     */
		template <>
		CSetOfObjectsPtr OPENGL_IMPEXP posePDF2opengl<CPose3DQuatPDF,CSetOfObjectsPtr>(const CPose3DQuatPDF &o);

		/**  @}  */
	}

} // End of namespace


#endif

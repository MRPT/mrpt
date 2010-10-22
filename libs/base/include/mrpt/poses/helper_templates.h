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
#ifndef MRPT_POSES_HELPERS_H
#define MRPT_POSES_HELPERS_H

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3DQuat.h>

#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>

namespace mrpt
{
	namespace poses
	{
		/**  @name Helper templates to convert a pose or a pose PDF to its mean value at compile time.
			 @{
		  */
		template <class POSE> inline const typename POSE::type_value &getPoseMean(const POSE &p);

		template <> inline const CPose2D &getPoseMean<CPose2D>(const CPose2D &p) { return p;}
		template <> inline const CPose3D &getPoseMean<CPose3D>(const CPose3D &p) { return p;}
		template <> inline const CPose3DQuat &getPoseMean<CPose3DQuat>(const CPose3DQuat &p) { return p;}

		template <> inline const CPose2D &getPoseMean<CPosePDFGaussian>(const CPosePDFGaussian &p) { return p.mean;}
		template <> inline const CPose3D &getPoseMean<CPose3DPDFGaussian>(const CPose3DPDFGaussian &p) { return p.mean;}
		template <> inline const CPose3DQuat &getPoseMean<CPose3DQuatPDFGaussian>(const CPose3DQuatPDFGaussian &p) { return p.mean;}

		template <> inline const CPose2D &getPoseMean<CPosePDFGaussianInf>(const CPosePDFGaussianInf &p) { return p.mean;}
		template <> inline const CPose3D &getPoseMean<CPose3DPDFGaussianInf>(const CPose3DPDFGaussianInf &p) { return p.mean;}
		/** @}  */

	} // End of namespace
} // End of namespace

#endif

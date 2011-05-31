/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
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

#include <mrpt/detectors.h>  // Precompiled headers

#include "do_opencv_includes.h"

#include <mrpt/detectors/CObjectDetection.h>
#include <mrpt/obs.h>
#include <mrpt/slam/CObservationImage.h>
#include <mrpt/utils/CStartUpClassesRegister.h>

using namespace mrpt::detectors;

extern CStartUpClassesRegister  mrpt_detectors_class_reg;

void CObjectDetection::detectObjects(const CImage *img, vector_detectable_object &detected)
{
	//static const int dumm2 =
	mrpt_detectors_class_reg.do_nothing(); // Avoid compiler removing this class in static linking

	mrpt::slam::CObservationImage o;
	o.timestamp = mrpt::system::now();
	o.image.setFromImageReadOnly(*img);
	this->detectObjects_Impl(&o,detected);
}

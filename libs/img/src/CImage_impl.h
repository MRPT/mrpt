/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/get_env.h>
#include <mrpt/img/CImage.h>

// Universal include for all versions of OpenCV
#include <mrpt/3rdparty/do_opencv_includes.h>

struct mrpt::img::CImage::Impl
{
#if MRPT_HAS_OPENCV
	cv::Mat img;
#endif

	~Impl()
	{
		const thread_local bool SHOW_DEBUG_MSG =
			mrpt::get_env<bool>("MRPT_DEBUG_IMG_LAZY_LOAD", false);
		if (SHOW_DEBUG_MSG)
		{
			std::cout << "[CImage::dtor] Called on this="
					  << reinterpret_cast<const void*>(this) << std::endl;
		}
	}
};

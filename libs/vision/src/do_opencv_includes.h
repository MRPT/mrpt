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

#ifndef MRPT_VISION_INTERNAL_OPECV_INCL_H
#define MRPT_VISION_INTERNAL_OPECV_INCL_H

#include <mrpt/config.h>

#if MRPT_HAS_OPENCV
	// OPENCV HEADERS
#	define CV_NO_CVV_IMAGE // Avoid CImage name crash

#	if MRPT_OPENCV_VERSION_NUM>=0x211
#		include <opencv2/core/core.hpp>
#		include <opencv2/highgui/highgui.hpp>
#		include <opencv2/imgproc/imgproc.hpp>
#		include <opencv2/imgproc/imgproc_c.h>
#		include <opencv2/features2d/features2d.hpp>
#		include <opencv2/video/tracking.hpp>
#		include <opencv2/calib3d/calib3d.hpp>
#		include <opencv2/objdetect/objdetect.hpp>

#		include <opencv2/legacy/legacy.hpp>  // CvImage
#		include <opencv2/legacy/compat.hpp>
#	else
		// For OpenCV <=2.1
#		include <cv.h>
#		include <highgui.h>
#		include <cvaux.h>
#	endif

	#ifdef CImage	// For old OpenCV versions (<=1.0.0)
	#undef CImage
	#endif

	#include <mrpt/utils/CImage.h>
	using mrpt::utils::CImage;

	typedef std::vector<CvPoint2D32f> CvPoint2D32fVector;

#	if MRPT_OPENCV_VERSION_NUM >= 0x200

#	include <mrpt/math/KDTreeCapable.h>

	/** Helper class: KD-tree search class for vector<KeyPoint>:
	  */
	class CSimple2DKDTree : public mrpt::math::KDTreeCapable
	{
	public:
		const std::vector<cv::KeyPoint> & m_data;
		CSimple2DKDTree(const std::vector<cv::KeyPoint> & data) : m_data(data) {  }

	protected:
		/** Must return the number of data points */
		virtual size_t kdtree_get_point_count() const {
			return m_data.size();
		}
		/** Must fill out the data points in "data", such as the i'th point will be stored in (data[i][0],...,data[i][nDims-1]). */
		virtual void kdtree_fill_point_data(ANNpointArray &data, const int nDims) const
		{
			const size_t N = m_data.size();
			for (size_t i=0;i<N;i++) {
				data[i][0] = m_data[i].pt.x;
				data[i][1] = m_data[i].pt.y;
			}
		}
	}; // end CSimple2DKDTree

	/** A helper struct to sort keypoints by their response */
	struct KeypointCompCache : public std::binary_function<size_t,size_t,bool>
	{
		const std::vector<cv::KeyPoint> &m_data;
		KeypointCompCache( const std::vector<cv::KeyPoint> &data ) : m_data(data) { }
		bool operator() (size_t k1, size_t k2 ) const {
			return (m_data[k1].response > m_data[k2].response);
		}
	};

#	endif // MRPT_OPENCV_VERSION_NUM >= 0x211

#endif // MRPT_HAS_OPENCV

/** A helper struct to sort lists of responses */
struct KeypointCompCache2 : public std::binary_function<size_t,size_t,bool>
{
	const std::vector<float> &m_data;
	KeypointCompCache2( const std::vector<float> &data ) : m_data(data) { }
	bool operator() (size_t k1, size_t k2 ) const {
		return (m_data[k1] > m_data[k2]);
	}
};

#endif

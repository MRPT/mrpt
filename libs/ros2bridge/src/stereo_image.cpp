/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: mrpt_ros bridge
	FILE: stereo_image.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#include <mrpt/ros2bridge/image.h>
#include <mrpt/ros2bridge/stereo_image.h>

#if CV_BRIDGE_VERSION <= 0x030400
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace sensor_msgs;
using namespace cv;
using namespace cv_bridge;

bool mrpt::ros2bridge::toROS(
	const mrpt::obs::CObservationStereoImages& obj,
	const std_msgs::msg::Header& msg_header, sensor_msgs::msg::Image& left,
	sensor_msgs::msg::Image& right, stereo_msgs::msg::DisparityImage& disparity)
{
	// left image
	const Mat& cvImgL = obj.imageLeft.asCvMatRef();

	cv_bridge::CvImage img_bridge;
	img_bridge =
		CvImage(left.header, sensor_msgs::image_encodings::BGR8, cvImgL);
	img_bridge.toImageMsg(left);
	left.encoding = "bgr8";
	left.header = msg_header;
	left.height = obj.imageLeft.getHeight();
	left.width = obj.imageLeft.getWidth();

	// right image
	const Mat& cvImgR = obj.imageLeft.asCvMatRef();

	cv_bridge::CvImage img_bridge2;
	img_bridge2 =
		CvImage(right.header, sensor_msgs::image_encodings::BGR8, cvImgR);
	img_bridge2.toImageMsg(right);
	right.encoding = "bgr8";
	right.header = msg_header;
	right.height = obj.imageRight.getHeight();
	right.width = obj.imageRight.getWidth();

	if (obj.hasImageDisparity)
	{
		const Mat& cvImgD = obj.imageDisparity.asCvMatRef();

		cv_bridge::CvImage img_bridge3;
		img_bridge3 = CvImage(
			disparity.header, sensor_msgs::image_encodings::BGR8, cvImgD);
		img_bridge3.toImageMsg(disparity.image);
		disparity.image.encoding = "bgr8";
		disparity.image.header = msg_header;
		disparity.image.height = obj.imageDisparity.getHeight();
		disparity.image.width = obj.imageDisparity.getWidth();
	}
	return true;
}

//
/*
std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
 */

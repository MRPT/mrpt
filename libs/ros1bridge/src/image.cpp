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
	FILE: image.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#if CV_BRIDGE_VERSION <= 0x030400
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <mrpt/ros1bridge/image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

using namespace mrpt::img;
using namespace ros;
using namespace sensor_msgs;
using namespace cv;
using namespace cv_bridge;

mrpt::img::CImage mrpt::ros1bridge::fromROS(const sensor_msgs::Image& i)
{
	return mrpt::img::CImage(
		cv_bridge::toCvCopy(i, "bgr8").get()->image, mrpt::img::DEEP_COPY);
}

sensor_msgs::Image mrpt::ros1bridge::toROS(
	const mrpt::img::CImage& i, const std_msgs::Header& msg_header)
{
	const Mat& cvImg = i.asCvMatRef();

	cv_bridge::CvImage img_bridge;

	sensor_msgs::Image msg;
	img_bridge = CvImage(msg.header, sensor_msgs::image_encodings::BGR8, cvImg);

	img_bridge.toImageMsg(msg);

	msg.encoding = "bgr8";
	msg.header = msg_header;
	msg.height = i.getHeight();
	msg.width = i.getWidth();

	return msg;
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

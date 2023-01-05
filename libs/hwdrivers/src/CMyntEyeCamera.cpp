/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"	// Precompiled headers
//
#include <mrpt/3rdparty/do_opencv_includes.h>
#include <mrpt/hwdrivers/CMyntEyeCamera.h>

#include <chrono>
#include <iostream>
#include <thread>

#if MRPT_HAS_MYNTEYE_D
#include <mynteyed/camera.h>
#include <mynteyed/utils.h>
#endif

using namespace mrpt;
using namespace mrpt::hwdrivers;

struct CMyntEyeCamera::Impl
{
#if MRPT_HAS_MYNTEYE_D
	std::shared_ptr<mynteyed::Camera> cam =
		std::make_shared<mynteyed::Camera>();
	mynteyed::DeviceInfo dev_info;
#endif
};

#if MRPT_HAS_MYNTEYE_D
static mrpt::img::TCamera convertIntrinsics(const mynteyed::CameraIntrinsics& i)
{
	mrpt::img::TCamera p;
	p.ncols = i.width;
	p.nrows = i.height;

	p.cx(i.cx);
	p.cy(i.cy);
	p.fx(i.fx);
	p.fy(i.fy);
	// The distortion coefficients: k1,k2,p1,p2,k3
	for (int k = 0; k < 5; k++)
		p.dist[k] = i.coeffs[k];

	return p;
}
#endif

CMyntEyeCamera::CMyntEyeCamera(const TMyntEyeCameraParameters& p)
	: m_capture(mrpt::make_impl<CMyntEyeCamera::Impl>())
{
	MRPT_START
#if MRPT_HAS_MYNTEYE_D
	if (!mynteyed::util::select(*m_capture->cam, &m_capture->dev_info))
	{
		THROW_EXCEPTION("No MYNTEYE-D cameras was found!");
	}
	mynteyed::util::print_stream_infos(
		*m_capture->cam, m_capture->dev_info.index);

	mynteyed::OpenParams params(m_capture->dev_info.index);
	params.color_mode = mynteyed::ColorMode::COLOR_RECTIFIED;
	params.stream_mode = mynteyed::StreamMode::STREAM_1280x720;
	params.ir_intensity = p.ir_intensity;

	mynteyed::StreamMode stream_mode = params.stream_mode;
	m_capture->cam->Open(params);
	std::cout << std::endl;
	if (!m_capture->cam->IsOpened())
	{
		THROW_EXCEPTION("Error: Open camera failed");
	}
	std::cout << "[CMyntEyeCamera] Open device successful.\n";

	mynteyed::StreamIntrinsics si =
		m_capture->cam->GetStreamIntrinsics(stream_mode);

	m_intrinsics_left = convertIntrinsics(si.left);
	m_intrinsics_right = convertIntrinsics(si.right);

	std::cout << "LEFT camera intrinsics:\n"
			  << m_intrinsics_left.dumpAsText() << "\n";
	std::cout << "RIGHT camera intrinsics:\n"
			  << m_intrinsics_right.dumpAsText() << "\n";

	std::cout << "Waiting for streams...\n";
	m_capture->cam->WaitForStreams();
	std::cout << "Streams started OK.\n";

	m_bInitialized = true;

#else
	THROW_EXCEPTION("MRPT was built without MYNTEYE-D SDK");
#endif
	MRPT_END
}

CMyntEyeCamera::~CMyntEyeCamera() {}

bool CMyntEyeCamera::getObservation(mrpt::obs::CObservation3DRangeScan& out)
{
#if MRPT_HAS_MYNTEYE_D && MRPT_HAS_OPENCV
	ASSERT_(m_bInitialized);

	mynteyed::StreamData image_color, image_depth;

	for (int nRetries = 0; nRetries < 100; nRetries++)
	{
		if (!image_color.img)
			image_color = m_capture->cam->GetStreamData(
				mynteyed::ImageType::IMAGE_LEFT_COLOR);

		if (!image_depth.img)
			image_depth =
				m_capture->cam->GetStreamData(mynteyed::ImageType::IMAGE_DEPTH);

		if (image_color.img && image_depth.img) break;
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}

	// Empty output obs:
	out = mrpt::obs::CObservation3DRangeScan();

	if (!image_color.img && !image_depth.img) return false;

	// all is good:
	out.timestamp = mrpt::Clock::now();

	// RGB= 3xu8
	if (image_color.img)
	{
		auto i = image_color.img->To(mynteyed::ImageFormat::COLOR_BGR);

		ASSERT_EQUAL_(
			i->data_size(), static_cast<size_t>(i->width() * i->height() * 3));

		cv::Mat m(i->height(), i->width(), CV_8UC3, i->data());

		out.hasIntensityImage = true;
		out.cameraParamsIntensity = m_intrinsics_left;
		out.intensityImage = mrpt::img::CImage(m, mrpt::img::DEEP_COPY);
	}

	// Depth= u16
	if (image_depth.img)
	{
		auto i = image_depth.img->To(mynteyed::ImageFormat::DEPTH_RAW);

		const auto h = i->height(), w = i->width();

		out.hasRangeImage = true;
		out.cameraParams = m_intrinsics_left;

		out.range_is_depth = true;
		// This method will try to exploit memory pooling if possible:
		out.rangeImage_setSize(h, w);
		out.rangeUnits = 1e-3f;	 // we use mm as units

		ASSERT_EQUAL_(
			i->data_size(), static_cast<size_t>(i->width() * i->height() * 2));
		cv::Mat m(i->height(), i->width(), CV_16UC1, i->data());

		for (int r = 0; r < h; r++)
		{
			for (int c = 0; c < w; c++)
			{
				const uint16_t v = m.at<uint16_t>(r, c);
				out.rangeImage.coeffRef(r, c) = v;
			}
		}
	}

	return true;
#else
	THROW_EXCEPTION("MRPT was built without MYNTEYE-D SDK or OpenCV");
#endif
}

void TMyntEyeCameraParameters::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& c, const std::string& s)
{
	MRPT_LOAD_CONFIG_VAR_CS(ir_intensity, int);
}

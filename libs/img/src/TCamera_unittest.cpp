/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/config.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/img/TCamera.h>

template class mrpt::CTraitsTest<mrpt::img::TCamera>;

using mrpt::img::TCamera;

static TCamera getSampleCameraParamsBase()
{
	TCamera c1;
	c1.ncols = 800;
	c1.nrows = 600;
	c1.fx(350);
	c1.fy(340);
	c1.cx(400);
	c1.cy(300);
	return c1;
}

static TCamera getSampleCameraParamsPlumbBob()
{
	TCamera c1 = getSampleCameraParamsBase();
	c1.distortion = mrpt::img::DistortionModel::plumb_bob;
	c1.k1(1e-3);
	c1.k2(2e-3);
	c1.k3(3e-3);
	c1.p1(4.5e-3);
	c1.p2(5.6e-9);
	return c1;
}

static TCamera getSampleCameraParamsFishEye()
{
	TCamera c1 = getSampleCameraParamsBase();
	c1.distortion = mrpt::img::DistortionModel::kannala_brandt;
	c1.k1(1e-3);
	c1.k2(2e-3);
	c1.k3(3e-3);
	c1.k4(-5e-6);
	return c1;
}

TEST(TCamera, EqualOperator)
{
	const auto c1 = getSampleCameraParamsPlumbBob();
	auto c2 = c1;
	EXPECT_TRUE(c1 == c2);

	c2.fx(c1.fx() + 0.1);
	EXPECT_FALSE(c1 == c2);
}

TEST(TCamera, CopyCtor)
{
	const auto c1 = getSampleCameraParamsPlumbBob();
	const auto c2 = c1;
	EXPECT_EQ(c2, c1);

	EXPECT_EQ(c2.ncols, 800U);
	EXPECT_EQ(c2.nrows, 600U);
	EXPECT_EQ(c2.fx(), 350);
	EXPECT_EQ(c2.fy(), 340);
	EXPECT_EQ(c2.cx(), 400);
	EXPECT_EQ(c2.cy(), 300);
	EXPECT_EQ(c2.k1(), 1e-3);
	EXPECT_EQ(c2.k2(), 2e-3);
	EXPECT_EQ(c2.k3(), 3e-3);
	EXPECT_EQ(c2.p1(), 4.5e-3);
	EXPECT_EQ(c2.p2(), 5.6e-9);
}

TEST(TCamera, ToFromINI_PlumbBob)
{
	const auto c1 = getSampleCameraParamsPlumbBob();

	mrpt::config::CConfigFileMemory cfg;
	c1.saveToConfigFile("cam1", cfg);

	mrpt::img::TCamera c2;
	c2.loadFromConfigFile("cam1", cfg);

	EXPECT_TRUE(c1 == c2) << "c1:\n"
						  << c1.dumpAsText() << "\nc2:\n"
						  << c2.dumpAsText();
}

TEST(TCamera, ToFromINI_FishEye)
{
	const auto c1 = getSampleCameraParamsFishEye();

	mrpt::config::CConfigFileMemory cfg;
	c1.saveToConfigFile("cam1", cfg);

	mrpt::img::TCamera c2;
	c2.loadFromConfigFile("cam1", cfg);

	EXPECT_TRUE(c1 == c2) << "c1:\n"
						  << c1.dumpAsText() << "\nc2:\n"
						  << c2.dumpAsText();
}

#if MRPT_HAS_FYAML
TEST(TCamera, FromYAML_PlumbBob)
{
	const std::string sampleParams = R"XXX(#yaml camera in OpenCV format
image_width: 2448
image_height: 2050
camera_name: prosilica
camera_matrix:
  rows: 3
  cols: 3
  data: [4827.94, 0, 1223.5, 0, 4835.62, 1024.5, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.41527, 0.31874, -0.00197, 0.00071, 0]
)XXX";

	const auto cfg = mrpt::containers::yaml::FromText(sampleParams);

	const auto c = TCamera::FromYAML(cfg);

	EXPECT_EQ(c.distortion, mrpt::img::DistortionModel::plumb_bob);

	EXPECT_EQ(c.ncols, 2448U);
	EXPECT_EQ(c.nrows, 2050U);
	EXPECT_EQ(c.fx(), 4827.94);
	EXPECT_EQ(c.fy(), 4835.62);
	EXPECT_EQ(c.cx(), 1223.5);
	EXPECT_EQ(c.cy(), 1024.5);
	EXPECT_EQ(c.k1(), -0.41527);
	EXPECT_EQ(c.k2(), 0.31874);
	EXPECT_EQ(c.p1(), -0.00197);
	EXPECT_EQ(c.p2(), 0.00071);
	EXPECT_EQ(c.k3(), 0);
}

TEST(TCamera, FromYAML_FishEye)
{
	const std::string sampleParams = R"XXX(#yaml camera in OpenCV format
image_width: 2448
image_height: 2050
camera_name: prosilica
camera_matrix:
  rows: 3
  cols: 3
  data: [4827.94, 0, 1223.5, 0, 4835.62, 1024.5, 0, 0, 1]
distortion_model: kannala_brandt
distortion_coefficients:
  rows: 1
  cols: 4
  data: [-0.41527, 0.31874, -0.00197, 0.00071]
)XXX";

	const auto cfg = mrpt::containers::yaml::FromText(sampleParams);

	const auto c = TCamera::FromYAML(cfg);

	EXPECT_EQ(c.distortion, mrpt::img::DistortionModel::kannala_brandt);

	EXPECT_EQ(c.ncols, 2448U);
	EXPECT_EQ(c.nrows, 2050U);
	EXPECT_EQ(c.fx(), 4827.94);
	EXPECT_EQ(c.fy(), 4835.62);
	EXPECT_EQ(c.cx(), 1223.5);
	EXPECT_EQ(c.cy(), 1024.5);
	EXPECT_EQ(c.k1(), -0.41527);
	EXPECT_EQ(c.k2(), 0.31874);
	EXPECT_EQ(c.k3(), -0.00197);
	EXPECT_EQ(c.k4(), 0.00071);
}

TEST(TCamera, ToFromYAML_PlumbBob)
{
	const auto c1 = getSampleCameraParamsPlumbBob();
	std::stringstream ss;
	ss << c1.asYAML();

	const auto c2 =
		TCamera::FromYAML(mrpt::containers::yaml::FromText(ss.str()));

	EXPECT_TRUE(c1 == c2) << "c1:\n" << c1.asYAML() << "\nc2:\n" << c2.asYAML();
}
TEST(TCamera, ToFromYAML_FishEye)
{
	const auto c1 = getSampleCameraParamsFishEye();
	std::stringstream ss;
	ss << c1.asYAML();

	const auto c2 =
		TCamera::FromYAML(mrpt::containers::yaml::FromText(ss.str()));

	EXPECT_TRUE(c1 == c2) << "c1:\n" << c1.asYAML() << "\nc2:\n" << c2.asYAML();
}
#endif

/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/CMatrixFixed.h>

#include <Eigen/Dense>
#include <sstream>

#include "mrpt_test.h"

MRPT_TEST(MatrixYaml, FromToEigen)
{
	const Eigen::MatrixXd m = Eigen::MatrixXd::Identity(3, 4);
	const auto y1 = mrpt::containers::yaml::FromMatrix(m);

	mrpt::containers::yaml d = mrpt::containers::yaml::Map();
	d["K"] = y1;

	EXPECT_TRUE(y1.isMap());
	EXPECT_TRUE(y1.has("rows"));
	EXPECT_TRUE(y1.has("cols"));
	EXPECT_TRUE(y1.has("data"));

	EXPECT_TRUE(y1["rows"].isScalar());
	EXPECT_TRUE(y1["cols"].isScalar());
	EXPECT_TRUE(y1["data"].isSequence());
	EXPECT_EQ(y1["data"].asSequence().size(), 12UL);

	const std::string expectedStr = R"(K:
  cols: 4
  data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]
  rows: 3
)";

	mrpt::containers::YamlEmitOptions eo;
	eo.emitHeader = false;

	std::stringstream ss;
	d.printAsYAML(ss, eo);
	EXPECT_EQ(expectedStr, ss.str());

	// 2nd part: to Eigen:
	{
		Eigen::MatrixXd m2;
		y1.toMatrix(m2);
		EXPECT_EQ(m2.cols(), m.cols());
		EXPECT_EQ(m2.rows(), m.rows());
		EXPECT_EQ(m2, m);
	}
	{
		Eigen::Matrix<double, 3, 4> m2;
		y1.toMatrix(m2);
		EXPECT_EQ(m2.cols(), m.cols());
		EXPECT_EQ(m2.rows(), m.rows());
		EXPECT_EQ(m2, m);
	}
	{
		Eigen::Matrix<double, 3, 3> m2;
		EXPECT_ANY_THROW(y1.toMatrix(m2));
	}
}
MRPT_TEST_END()

MRPT_TEST(MatrixYaml, Vector)
{
	const Eigen::Vector3d m = Eigen::Vector3d::Constant(1.0);
	const auto y1 = mrpt::containers::yaml::FromMatrix(m);

	EXPECT_TRUE(y1.isMap());
	EXPECT_TRUE(y1.has("rows"));
	EXPECT_TRUE(y1.has("cols"));
	EXPECT_TRUE(y1.has("data"));

	EXPECT_EQ(y1["rows"].as<int>(), 3);
	EXPECT_EQ(y1["cols"].as<int>(), 1);
	EXPECT_TRUE(y1["data"].isSequence());
	EXPECT_EQ(y1["data"].asSequence().size(), 3UL);
}
MRPT_TEST_END()

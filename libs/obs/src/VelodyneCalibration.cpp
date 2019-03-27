/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/config.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/VelodyneCalibration.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#if MRPT_HAS_YAMLCPP
#include <yaml-cpp/yaml.h>
#endif

#undef _UNICODE  // JLBC, for xmlParser
#include "xmlparser/xmlParser.h"

// ======= Default calibration files ========================
#include "velodyne_default_calib_HDL-32.h"
#include "velodyne_default_calib_VLP-16.h"
#include "velodyne_default_calib_hdl64e-s3.h"
// ======= End of default calibration files =================

using namespace std;
using namespace mrpt::obs;

VelodyneCalibration::PerLaserCalib::PerLaserCalib()

	= default;

VelodyneCalibration::VelodyneCalibration() : laser_corrections(0) {}
bool VelodyneCalibration::internal_loadFromXMLNode(void* node_ptr)
{
	XMLNode& root = *reinterpret_cast<XMLNode*>(node_ptr);

	XMLNode node_bs = root.getChildNode("boost_serialization");
	if (node_bs.isEmpty())
		throw std::runtime_error("Cannot find XML node: 'boost_serialization'");

	XMLNode node_DB = node_bs.getChildNode("DB");
	if (node_DB.isEmpty())
		throw std::runtime_error("Cannot find XML node: 'DB'");

	XMLNode node_enabled_ = node_DB.getChildNode("enabled_");
	if (node_enabled_.isEmpty())
		throw std::runtime_error("Cannot find XML node: 'enabled_'");

	// Clear previous contents:
	clear();

	XMLNode node_enabled_count = node_enabled_.getChildNode("count");
	if (node_enabled_count.isEmpty())
		throw std::runtime_error("Cannot find XML node: 'enabled_::count'");
	const int nEnabled = atoi(node_enabled_count.getText());
	if (nEnabled <= 0 || nEnabled > 10000)
		throw std::runtime_error(
			"Senseless value found reading 'enabled_::count'");

	int enabledCount = 0;
	for (int i = 0; i < nEnabled; i++)
	{
		XMLNode node_enabled_ith = node_enabled_.getChildNode("item", i);
		if (node_enabled_ith.isEmpty())
			throw std::runtime_error(
				"Cannot find the expected number of XML nodes: "
				"'enabled_::item'");
		const int enable_val = atoi(node_enabled_ith.getText());
		if (enable_val) ++enabledCount;
	}

	// enabledCount = number of lasers in the LIDAR
	this->laser_corrections.resize(enabledCount);

	XMLNode node_points_ = node_DB.getChildNode("points_");
	if (node_points_.isEmpty())
		throw std::runtime_error("Cannot find XML node: 'points_'");

	for (int i = 0;; ++i)
	{
		XMLNode node_points_item = node_points_.getChildNode("item", i);
		if (node_points_item.isEmpty()) break;

		XMLNode node_px = node_points_item.getChildNode("px");
		if (node_px.isEmpty())
			throw std::runtime_error(
				"Cannot find XML node: 'points_::item::px'");

		XMLNode node_px_id = node_px.getChildNode("id_");
		if (node_px_id.isEmpty())
			throw std::runtime_error(
				"Cannot find XML node: 'points_::item::px::id_'");
		const int id = atoi(node_px_id.getText());
		ASSERT_ABOVEEQ_(id, 0);
		if (id >= enabledCount) continue;  // ignore

		PerLaserCalib* plc = &laser_corrections[id];

		{
			XMLNode node = node_px.getChildNode("rotCorrection_");
			if (node.isEmpty())
				throw std::runtime_error(
					"Cannot find XML node: "
					"'points_::item::px::rotCorrection_'");
			plc->azimuthCorrection = atof(node.getText());
		}
		{
			XMLNode node = node_px.getChildNode("vertCorrection_");
			if (node.isEmpty())
				throw std::runtime_error(
					"Cannot find XML node: "
					"'points_::item::px::vertCorrection_'");
			plc->verticalCorrection = atof(node.getText());
		}
		{
			XMLNode node = node_px.getChildNode("distCorrection_");
			if (node.isEmpty())
				throw std::runtime_error(
					"Cannot find XML node: "
					"'points_::item::px::distCorrection_'");
			plc->distanceCorrection = 0.01f * atof(node.getText());
		}
		{
			XMLNode node = node_px.getChildNode("vertOffsetCorrection_");
			if (node.isEmpty())
				throw std::runtime_error(
					"Cannot find XML node: "
					"'points_::item::px::vertOffsetCorrection_'");
			plc->verticalOffsetCorrection = 0.01f * atof(node.getText());
		}
		{
			XMLNode node = node_px.getChildNode("horizOffsetCorrection_");
			if (node.isEmpty())
				throw std::runtime_error(
					"Cannot find XML node: "
					"'points_::item::px::horizOffsetCorrection_'");
			plc->horizontalOffsetCorrection = 0.01f * atof(node.getText());
		}

		plc->sinVertCorrection =
			std::sin(mrpt::DEG2RAD(plc->verticalCorrection));
		plc->cosVertCorrection =
			std::cos(mrpt::DEG2RAD(plc->verticalCorrection));

		plc->sinVertOffsetCorrection =
			plc->sinVertCorrection * plc->sinVertOffsetCorrection;
		plc->cosVertOffsetCorrection =
			plc->cosVertCorrection * plc->sinVertOffsetCorrection;
	}

	return true;  // Ok
}

bool VelodyneCalibration::loadFromXMLText(const std::string& xml_file_contents)
{
	try
	{
		XMLResults results;
		XMLNode root =
			XMLNode::parseString(xml_file_contents.c_str(), nullptr, &results);

		if (results.error != eXMLErrorNone)
		{
			cerr << "[VelodyneCalibration::loadFromXMLText] Error parsing XML "
					"content: "
				 << XMLNode::getError(results.error) << " at line "
				 << results.nLine << ":" << results.nColumn << endl;
			return false;
		}

		return internal_loadFromXMLNode(reinterpret_cast<void*>(&root));
	}
	catch (exception& e)
	{
		cerr << "[VelodyneCalibration::loadFromXMLFile] Exception:" << endl
			 << e.what() << endl;
		return false;
	}
}

/** Loads calibration from file. \return false on any error, true on success */
// See reference code in:
// vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile()
bool VelodyneCalibration::loadFromXMLFile(
	const std::string& velodyne_calibration_xml_filename)
{
	try
	{
		XMLResults results;
		XMLNode root = XMLNode::parseFile(
			velodyne_calibration_xml_filename.c_str(), nullptr, &results);

		if (results.error != eXMLErrorNone)
		{
			cerr << "[VelodyneCalibration::loadFromXMLFile] Error loading XML "
					"file: "
				 << XMLNode::getError(results.error) << " at line "
				 << results.nLine << ":" << results.nColumn << endl;
			return false;
		}
		return internal_loadFromXMLNode(reinterpret_cast<void*>(&root));
	}
	catch (exception& e)
	{
		cerr << "[VelodyneCalibration::loadFromXMLFile] Exception:" << endl
			 << e.what() << endl;
		return false;
	}
}

bool VelodyneCalibration::loadFromYAMLText(const std::string& str)
{
#if MRPT_HAS_YAMLCPP
	try
	{
		YAML::Node root = YAML::Load(str);

		// Clear previous contents:
		clear();

		const auto num_lasers = root["num_lasers"].as<unsigned int>(0);
		ASSERT_(num_lasers > 0);

		this->laser_corrections.resize(num_lasers);

		auto lasers = root["lasers"];
		ASSERT_EQUAL_(lasers.size(), num_lasers);

		for (auto item : lasers)
		{
			const auto id = item["laser_id"].as<unsigned int>(9999999);
			ASSERT_(id < num_lasers);

			PerLaserCalib* plc = &laser_corrections[id];

			plc->azimuthCorrection = item["rot_correction"].as<double>();
			plc->verticalCorrection = item["vert_correction"].as<double>();

			plc->distanceCorrection = item["dist_correction"].as<double>();
			plc->verticalOffsetCorrection =
				item["vert_offset_correction"].as<double>();
			plc->horizontalOffsetCorrection =
				item["horiz_offset_correction"].as<double>();

			plc->sinVertCorrection = std::sin(plc->verticalCorrection);
			plc->cosVertCorrection = std::cos(plc->verticalCorrection);

			plc->sinVertOffsetCorrection =
				plc->sinVertCorrection * plc->sinVertOffsetCorrection;
			plc->cosVertOffsetCorrection =
				plc->cosVertCorrection * plc->sinVertOffsetCorrection;
		}

		return true;  // all ok
	}
	catch (const std::exception& e)
	{
		std::cerr << "[VelodyneCalibration::loadFromYAMLText]" << e.what()
				  << "\n";
		return false;
	}
#else
	THROW_EXCEPTION("This method requires building MRPT with YAML-CPP.");
#endif
}

bool VelodyneCalibration::loadFromYAMLFile(const std::string& filename)
{
#if MRPT_HAS_YAMLCPP
	try
	{
		std::ifstream f(filename);
		if (!f.is_open())
			THROW_EXCEPTION_FMT("Cannot open file: '%s'", filename.c_str());

		// Load file:
		std::string str(
			(std::istreambuf_iterator<char>(f)),
			std::istreambuf_iterator<char>());

		return loadFromYAMLText(str);
	}
	catch (const std::exception& e)
	{
		std::cerr << "[VelodyneCalibration::loadFromYAMLFile]" << e.what()
				  << "\n";
		return false;
	}
#else
	THROW_EXCEPTION("This method requires building MRPT with YAML-CPP.");
#endif
}

bool VelodyneCalibration::empty() const { return laser_corrections.empty(); }
void VelodyneCalibration::clear() { laser_corrections.clear(); }
static std::map<std::string, VelodyneCalibration> cache_default_calibs;

// It always return a calibration structure, but it may be empty if the model
// name is unknown.
const VelodyneCalibration& VelodyneCalibration::LoadDefaultCalibration(
	const std::string& lidar_model)
{
	// Cached calib data?
	auto it = cache_default_calibs.find(lidar_model);
	if (it != cache_default_calibs.end()) return it->second;

	VelodyneCalibration result;  // Leave empty to indicate unknown model
	std::string xml_contents, yaml_contents;

	if (lidar_model == "VLP16")
		xml_contents = velodyne_default_calib_VLP16;
	else if (lidar_model == "HDL32")
		xml_contents = velodyne_default_calib_HDL32;
	else if (lidar_model == "HDL64")
		yaml_contents = velodyne_default_calib_HDL64E_S3;

	if (!xml_contents.empty())
	{
		if (!result.loadFromXMLText(xml_contents))
			std::cerr << "[VelodyneCalibration::LoadDefaultCalibration] Error "
						 "parsing default XML calibration file for model '"
					  << lidar_model << "'\n";
	}
	else if (!yaml_contents.empty())
	{
		if (!result.loadFromYAMLText(yaml_contents))
			std::cerr << "[VelodyneCalibration::LoadDefaultCalibration] Error "
						 "parsing default YAML calibration file for model '"
					  << lidar_model << "'\n";
	}

	cache_default_calibs[lidar_model] = result;
	return cache_default_calibs[lidar_model];
}

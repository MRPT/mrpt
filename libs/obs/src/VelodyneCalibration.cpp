/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers
//
#include <mrpt/containers/yaml.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/VelodyneCalibration.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>

#include <mrpt/config.h>
#if MRPT_HAS_TINYXML2
#include <tinyxml2.h>
#endif

// ======= Default calibration files ========================
#include "velodyne_default_calib_HDL-32.h"
#include "velodyne_default_calib_VLP-16.h"
#include "velodyne_default_calib_hdl64e-s3.h"
// ======= End of default calibration files =================

using namespace std;
using namespace mrpt::obs;

VelodyneCalibration::PerLaserCalib::PerLaserCalib() = default;

#if MRPT_HAS_TINYXML2
static const tinyxml2::XMLElement* get_xml_children(
	const tinyxml2::XMLNode* e, const char* name)
{
	ASSERT_(e != nullptr);
	// It's ok if name is nullptr for tinyxml2

	auto ret = e->FirstChildElement(name);
	if (!ret)
		throw std::runtime_error(
			mrpt::format("Cannot find XML tag `%s`", name));
	return ret;
}
static const char* get_xml_children_as_str(
	const tinyxml2::XMLNode* e, const char* name)
{
	auto n = get_xml_children(e, name);
	ASSERTMSG_(n, mrpt::format("Cannot find XML tag `%s`", name));
	const char* txt = n->GetText();
	ASSERTMSG_(
		txt, mrpt::format("Cannot convert XML tag `%s` to string", name));
	return txt;
}
static double get_xml_children_as_double(
	const tinyxml2::XMLNode* e, const char* name)
{
	return ::atof(get_xml_children_as_str(e, name));
}
static int get_xml_children_as_int(const tinyxml2::XMLNode* e, const char* name)
{
	return ::atoi(get_xml_children_as_str(e, name));
}
#endif

VelodyneCalibration::VelodyneCalibration() : laser_corrections(0) {}
bool VelodyneCalibration::internal_loadFromXMLNode(void* node_ptr)
{
#if MRPT_HAS_TINYXML2

	ASSERT_(node_ptr != nullptr);

	const tinyxml2::XMLDocument& root =
		*reinterpret_cast<tinyxml2::XMLDocument*>(node_ptr);

	auto node_bs = get_xml_children(&root, "boost_serialization");
	auto node_DB = get_xml_children(node_bs, "DB");
	auto node_enabled_ = get_xml_children(node_DB, "enabled_");

	// Clear previous contents:
	clear();

	const int nEnabled = get_xml_children_as_int(node_enabled_, "count");
	ASSERT_GT_(nEnabled, 0);
	ASSERT_LT_(nEnabled, 10000);

	int enabledCount = 0;
	const tinyxml2::XMLElement* node_enabled_ith = nullptr;
	for (int i = 0; i < nEnabled; i++)
	{
		if (node_enabled_ith)
			node_enabled_ith = node_enabled_ith->NextSiblingElement("item");
		else
		{
			ASSERT_EQUAL_(i, 0);
			node_enabled_ith = node_enabled_->FirstChildElement("item");
		}

		if (!node_enabled_ith)
			throw std::runtime_error(
				"Cannot find the expected number of XML nodes: "
				"'enabled_::item'");
		const int enable_val = atoi(node_enabled_ith->GetText());
		if (enable_val) ++enabledCount;
	}

	// enabledCount = number of lasers in the LIDAR
	this->laser_corrections.resize(enabledCount);

	auto node_points_ = get_xml_children(node_DB, "points_");
	const tinyxml2::XMLElement* node_points_item = nullptr;

	for (int i = 0;; ++i)
	{
		if (!node_points_item)
			node_points_item = node_points_->FirstChildElement("item");
		else
			node_points_item = node_points_item->NextSiblingElement("item");

		if (!node_points_item) break;

		auto node_px = get_xml_children(node_points_item, "px");
		auto node_px_id = get_xml_children(node_px, "id_");
		const int id = atoi(node_px_id->GetText());
		ASSERT_GE_(id, 0);
		if (id >= enabledCount) continue;  // ignore

		PerLaserCalib& plc = laser_corrections[id];

		plc.azimuthCorrection =
			get_xml_children_as_double(node_px, "rotCorrection_");
		plc.verticalCorrection =
			get_xml_children_as_double(node_px, "vertCorrection_");

		plc.distanceCorrection =
			0.01 * get_xml_children_as_double(node_px, "distCorrection_");
		plc.verticalOffsetCorrection =
			0.01 * get_xml_children_as_double(node_px, "vertOffsetCorrection_");
		plc.horizontalOffsetCorrection =
			0.01 *
			get_xml_children_as_double(node_px, "horizOffsetCorrection_");

		plc.sinVertCorrection = std::sin(mrpt::DEG2RAD(plc.verticalCorrection));
		plc.cosVertCorrection = std::cos(mrpt::DEG2RAD(plc.verticalCorrection));

		plc.sinVertOffsetCorrection =
			plc.sinVertCorrection * plc.sinVertOffsetCorrection;
		plc.cosVertOffsetCorrection =
			plc.cosVertCorrection * plc.sinVertOffsetCorrection;
	}

	return true;  // Ok
#else
	return false;
#endif
}

bool VelodyneCalibration::loadFromXMLText(const std::string& xml_file_contents)
{
	try
	{
#if MRPT_HAS_TINYXML2
		tinyxml2::XMLDocument doc;
		const auto err = doc.Parse(xml_file_contents.c_str());

		if (err != tinyxml2::XML_SUCCESS)
		{
			std::cerr
				<< "[VelodyneCalibration::loadFromXMLText] Error parsing XML "
				   "content: "
#if TIXML2_MAJOR_VERSION >= 7 || \
	(TIXML2_MAJOR_VERSION >= 6 && TIXML2_MINOR_VERSION >= 2)
				<< tinyxml2::XMLDocument::ErrorIDToName(err)
#else
				<< doc.ErrorName()
#endif
				<< std::endl;
			return false;
		}

		return internal_loadFromXMLNode(reinterpret_cast<void*>(&doc));
#else
		THROW_EXCEPTION("MRPT built without tinyxml2");
#endif
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
	const std::string& velodyne_calibration_xml_fil)
{
	try
	{
#if MRPT_HAS_TINYXML2
		tinyxml2::XMLDocument doc;
		const auto err = doc.LoadFile(velodyne_calibration_xml_fil.c_str());

		if (err != tinyxml2::XML_SUCCESS)
		{
			std::cerr << "[VelodyneCalibration::loadFromXMLFile] Error loading "
						 "XML file: "
#if TIXML2_MAJOR_VERSION >= 7 || \
	(TIXML2_MAJOR_VERSION >= 6 && TIXML2_MINOR_VERSION >= 2)
					  << tinyxml2::XMLDocument::ErrorIDToName(err)
#else
					  << doc.ErrorName()
#endif
					  << std::endl;
			return false;
		}
		return internal_loadFromXMLNode(reinterpret_cast<void*>(&doc));
#else
		THROW_EXCEPTION("MRPT built without tinyxml2");
#endif
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
	try
	{
		mrpt::containers::yaml root = mrpt::containers::yaml::FromFile(str);

		// Clear previous contents:
		clear();

		const auto num_lasers =
			root.getOrDefault<unsigned int>("num_lasers", 0);
		ASSERT_(num_lasers > 0);

		this->laser_corrections.resize(num_lasers);

		auto lasers = root["lasers"];
		ASSERT_EQUAL_(lasers.size(), num_lasers);

		for (auto it : lasers.asSequence())
		{
			auto& item = it.asMap();

			const auto id = item["laser_id"].as<unsigned int>();
			ASSERT_(id < num_lasers);

			PerLaserCalib& plc = laser_corrections[id];

			plc.azimuthCorrection = item["rot_correction"].as<double>();
			plc.verticalCorrection = item["vert_correction"].as<double>();

			plc.distanceCorrection = item["dist_correction"].as<double>();
			plc.verticalOffsetCorrection =
				item["vert_offset_correction"].as<double>();
			plc.horizontalOffsetCorrection =
				item["horiz_offset_correction"].as<double>();

			plc.sinVertCorrection = std::sin(plc.verticalCorrection);
			plc.cosVertCorrection = std::cos(plc.verticalCorrection);

			plc.sinVertOffsetCorrection =
				plc.sinVertCorrection * plc.sinVertOffsetCorrection;
			plc.cosVertOffsetCorrection =
				plc.cosVertCorrection * plc.sinVertOffsetCorrection;
		}

		return true;  // all ok
	}
	catch (const std::exception& e)
	{
		std::cerr << "[VelodyneCalibration::loadFromYAMLText]" << e.what()
				  << "\n";
		return false;
	}
}

bool VelodyneCalibration::loadFromYAMLFile(const std::string& filename)
{
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

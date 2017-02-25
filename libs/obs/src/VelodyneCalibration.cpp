/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/VelodyneCalibration.h>
#include <mrpt/utils/bits.h>
#include <iostream>
#include <map>

#undef _UNICODE			// JLBC, for xmlParser
#include "xmlparser/xmlParser.h"

// ======= Default calibration files ========================
#include "velodyne_default_calib_VLP-16.h"
#include "velodyne_default_calib_HDL-32.h"
// ======= End of default calibration files =================


using namespace std;
using namespace mrpt::obs;

VelodyneCalibration::PerLaserCalib::PerLaserCalib() :
	azimuthCorrection          (.0),
	verticalCorrection         (.0),
	distanceCorrection         (.0),
	verticalOffsetCorrection   (.0),
	horizontalOffsetCorrection (.0),
	sinVertCorrection          (.0),
	cosVertCorrection          (1.0),
	sinVertOffsetCorrection    (.0),
	cosVertOffsetCorrection    (1.0)
{
}

VelodyneCalibration::VelodyneCalibration() :
	laser_corrections(0)
{
}

bool VelodyneCalibration::internal_loadFromXMLNode(void *node)
{
	XMLNode &root = *reinterpret_cast<XMLNode*>(node);

	XMLNode node_bs = root.getChildNode("boost_serialization");
	if (node_bs.isEmpty()) throw std::runtime_error("Cannot find XML node: 'boost_serialization'");

	XMLNode node_DB = node_bs.getChildNode("DB");
	if (node_DB.isEmpty()) throw std::runtime_error("Cannot find XML node: 'DB'");

	XMLNode node_enabled_ = node_DB.getChildNode("enabled_");
	if (node_enabled_.isEmpty()) throw std::runtime_error("Cannot find XML node: 'enabled_'");

	// Clear previous contents:
	clear();

	XMLNode node_enabled_count = node_enabled_.getChildNode("count");
	if (node_enabled_count.isEmpty()) throw std::runtime_error("Cannot find XML node: 'enabled_::count'");
	const int nEnabled = atoi( node_enabled_count.getText() );
	if (nEnabled<=0 || nEnabled>10000)
		throw std::runtime_error("Senseless value found reading 'enabled_::count'");

	int enabledCount = 0;
	for (int i=0;i<nEnabled;i++)
	{
		XMLNode node_enabled_ith = node_enabled_.getChildNode("item",i);
		if (node_enabled_ith.isEmpty()) throw std::runtime_error("Cannot find the expected number of XML nodes: 'enabled_::item'");
		const int enable_val = atoi( node_enabled_ith.getText() );
		if (enable_val)
			++enabledCount;
	}

	// enabledCount = number of lasers in the LIDAR
	this->laser_corrections.resize(enabledCount);

	XMLNode node_points_ = node_DB.getChildNode("points_");
	if (node_points_.isEmpty()) throw std::runtime_error("Cannot find XML node: 'points_'");

	for (int i=0; ; ++i)
	{
		XMLNode node_points_item = node_points_.getChildNode("item",i);
		if (node_points_item.isEmpty()) break;

		XMLNode node_px = node_points_item.getChildNode("px");
		if (node_px.isEmpty()) throw std::runtime_error("Cannot find XML node: 'points_::item::px'");

		XMLNode node_px_id = node_px.getChildNode("id_");
		if (node_px_id.isEmpty()) throw std::runtime_error("Cannot find XML node: 'points_::item::px::id_'");
		const int id = atoi( node_px_id.getText() );
		ASSERT_ABOVEEQ_(id,0);
		if (id>=enabledCount)
			continue; // ignore

		PerLaserCalib *plc = &laser_corrections[id];
				
		{
			XMLNode node = node_px.getChildNode("rotCorrection_");
			if (node.isEmpty()) throw std::runtime_error("Cannot find XML node: 'points_::item::px::rotCorrection_'");
			plc->azimuthCorrection = atof( node.getText() );
		}
		{
			XMLNode node = node_px.getChildNode("vertCorrection_");
			if (node.isEmpty()) throw std::runtime_error("Cannot find XML node: 'points_::item::px::vertCorrection_'");
			plc->verticalCorrection = atof( node.getText() );
		}
		{
			XMLNode node = node_px.getChildNode("distCorrection_");
			if (node.isEmpty()) throw std::runtime_error("Cannot find XML node: 'points_::item::px::distCorrection_'");
			plc->distanceCorrection = 0.01f * atof( node.getText() );
		}
		{
			XMLNode node = node_px.getChildNode("vertOffsetCorrection_");
			if (node.isEmpty()) throw std::runtime_error("Cannot find XML node: 'points_::item::px::vertOffsetCorrection_'");
			plc->verticalOffsetCorrection = 0.01f * atof( node.getText() );
		}
		{
			XMLNode node = node_px.getChildNode("horizOffsetCorrection_");
			if (node.isEmpty()) throw std::runtime_error("Cannot find XML node: 'points_::item::px::horizOffsetCorrection_'");
			plc->horizontalOffsetCorrection = 0.01f * atof( node.getText() );
		}

		plc->sinVertCorrection = std::sin( mrpt::utils::DEG2RAD( plc->verticalCorrection ) );
		plc->cosVertCorrection = std::cos( mrpt::utils::DEG2RAD( plc->verticalCorrection ) );

		plc->sinVertOffsetCorrection = plc->sinVertCorrection * plc->sinVertOffsetCorrection;
		plc->cosVertOffsetCorrection = plc->cosVertCorrection * plc->sinVertOffsetCorrection;
	}

	return true; // Ok
}

bool VelodyneCalibration::loadFromXMLText(const std::string & xml_file_contents)
{
	try
	{
		XMLResults 	results;
		XMLNode 	root = XMLNode::parseString(xml_file_contents.c_str(), NULL, &results );

		if (results.error != eXMLErrorNone) {
			cerr << "[VelodyneCalibration::loadFromXMLText] Error parsing XML content: " <<
					XMLNode::getError( results.error ) << " at line " << results.nLine << ":" << results.nColumn << endl;
			return false;
		}

		return internal_loadFromXMLNode( reinterpret_cast<void*>(&root) );
	}
	catch (exception &e)
	{
		cerr << "[VelodyneCalibration::loadFromXMLFile] Exception:" << endl << e.what() << endl;
		return false;
	}
}

/** Loads calibration from file. \return false on any error, true on success */
// See reference code in:  vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile()
bool VelodyneCalibration::loadFromXMLFile(const std::string & velodyne_calibration_xml_filename)
{
	try
	{
		XMLResults 	results;
		XMLNode 	root = XMLNode::parseFile( velodyne_calibration_xml_filename.c_str(), NULL, &results );

		if (results.error != eXMLErrorNone) {
			cerr << "[VelodyneCalibration::loadFromXMLFile] Error loading XML file: " <<
					XMLNode::getError( results.error ) << " at line " << results.nLine << ":" << results.nColumn << endl;
			return false;
		}
		return internal_loadFromXMLNode( reinterpret_cast<void*>(&root) );
	}
	catch (exception &e)
	{
		cerr << "[VelodyneCalibration::loadFromXMLFile] Exception:" << endl << e.what() << endl;
		return false;
	}
}

bool VelodyneCalibration::empty() const {
	return laser_corrections.empty();
}

void VelodyneCalibration::clear() {
	laser_corrections.clear();
}


std::map<std::string,VelodyneCalibration> cache_default_calibs;

// It always return a calibration structure, but it may be empty if the model name is unknown.
const VelodyneCalibration & VelodyneCalibration::LoadDefaultCalibration(const std::string & lidar_model)
{
	// Cached calib data?
	std::map<std::string,VelodyneCalibration>::const_iterator it = cache_default_calibs.find(lidar_model);
	if (it != cache_default_calibs.end())
		return it->second;

	VelodyneCalibration result; // Leave empty to indicate unknown model
	std::string xml_contents;

	if (lidar_model=="VLP16")
		xml_contents = velodyne_default_calib_VLP16;
	else if (lidar_model=="HDL32")
		xml_contents = velodyne_default_calib_HDL32;
	else
	{}


	if (!xml_contents.empty()) {
		if (!result.loadFromXMLText( xml_contents ) )
			std::cerr << "[VelodyneCalibration::LoadDefaultCalibration] Error parsing default XML calibration file for model '" << lidar_model << "'\n";
	}

	cache_default_calibs[lidar_model] = result;
	return cache_default_calibs[lidar_model];
}


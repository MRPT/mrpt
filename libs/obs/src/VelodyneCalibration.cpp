/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/VelodyneCalibration.h>

#undef _UNICODE			// JLBC, for xmlParser
#include "xmlparser/xmlParser.h"

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

VelodyneCalibration::VelodyneCalibration()
{
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

		XMLNode node_bs = root.getChildNode("boost_serialization");
		if (node_bs.isEmpty()) throw std::runtime_error("Cannot find XML node: 'boost_serialization'");

		XMLNode node_DB = node_bs.getChildNode("DB");
		if (node_DB.isEmpty()) throw std::runtime_error("Cannot find XML node: 'DB'");

		XMLNode node_enabled_ = node_DB.getChildNode("enabled_");
		if (node_enabled_.isEmpty()) throw std::runtime_error("Cannot find XML node: 'enabled_'");

		// Clear previous contents:
		clear();

#if 0
		// Get tables:
		size_t i,j, nTables = root.nChildNode("table");
		for (i=0;i<nTables;i++)
		{
			XMLNode tabNod = root.getChildNode("table",(int)i);
			ASSERT_(!tabNod.isEmpty())

			// Create table:
			CSimpleDatabaseTablePtr t = createTable( tabNod.getAttribute("name") );

			// Create fields:
			XMLNode fNod = tabNod.getChildNode("fields");
			ASSERT_(!fNod.isEmpty())

			size_t  nFields = fNod.nChildNode();
			for (j=0;j<nFields;j++)
			{
				t->addField( fNod.getChildNode((int)j).getName() );
			} // end for each field

			// Add record data:
			size_t nRecs = tabNod.nChildNode("record");
			for (size_t k=0;k<nRecs;k++)
			{
				size_t recIdx = t->appendRecord();

				XMLNode recNod = tabNod.getChildNode("record",(int)k);
				ASSERT_(!recNod.isEmpty())

				for (j=0;j<nFields;j++)
				{
					XMLCSTR  str=recNod.getChildNode(t->getFieldName(j).c_str() ).getText();
					t->set(recIdx,j, str!=NULL ?  string(str) : string() );
				}

			} // end for each record

		} // for each table
#endif

		return true; // Ok
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

bool VelodyneCalibration::loadFromXMLText(const std::string & xml_file_contents)
{
	MRPT_TODO("impl xml txt")
	return false;
}

std::map<std::string,VelodyneCalibration> cache_default_calibs;

const VelodyneCalibration & VelodyneCalibration::LoadDefaultCalibration(const std::string & lidar_model)
{
	// Cached calib data?
	std::map<std::string,VelodyneCalibration>::const_iterator it = cache_default_calibs.find(lidar_model);
	if (it != cache_default_calibs.end())
		return it->second;

	MRPT_TODO("impl")
	THROW_EXCEPTION("to do")
	// ...
	//bool VelodyneCalibration::loadFromXMLText(const std::string & xml_file_contents)

}




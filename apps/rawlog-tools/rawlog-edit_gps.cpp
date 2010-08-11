/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;

// STL data must have global scope:
struct TGPSDataPoint
{
	double  lon,lat,alt; // degrees, degrees, meters
	uint8_t fix; // 1: standalone, 2: DGPS, 4: RTK fix, 5: RTK float, ...
};

struct TDataPerGPS
{
	map<TTimeStamp,TGPSDataPoint> path;
};

// ======================================================================
//		op_export_gps_kml
// ======================================================================
DECLARE_OP_FUNCTION(op_export_gps_kml)
{
	// A class to do this operation:
	class CRawlogProcessor_ExportGPS_KML : public CRawlogProcessorOnEachObservation
	{
	protected:
		string	m_inFile;
		string 	outDir;

		map<string,TDataPerGPS> m_gps_paths;  // sensorLabel -> data

	public:

		CRawlogProcessor_ExportGPS_KML(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose)
		{
			getArgValue<string>(cmdline,"input",m_inFile);
		}

		// return false on any error.
		bool processOneObservation(CObservationPtr  &o)
		{
			if (!IS_CLASS(o, CObservationGPS ) )
				return true;

			const CObservationGPS* obs = CObservationGPSPtr(o).pointer();

			if (!obs->has_GGA_datum)
				return true; // Nothing to do...

			// Insert the new entries:
			TDataPerGPS   &D = m_gps_paths[obs->sensorLabel];
			TGPSDataPoint &d = D.path[o->timestamp];

			d.lon = obs->GGA_datum.longitude_degrees;
			d.lat = obs->GGA_datum.latitude_degrees;
			d.alt = obs->GGA_datum.altitude_meters;

			return true; // All ok
		}

		void generate_KML()
		{
			const bool save_altitude = false;

			// For each sensor label:
			for (map<string,TDataPerGPS>::const_iterator it=m_gps_paths.begin();it!=m_gps_paths.end();++it )
			{
				const string  &label = it->first;
				const TDataPerGPS &D = it->second;

				const string outfilname =
					mrpt::system::extractFileDirectory(m_inFile) +
					mrpt::system::extractFileName(m_inFile) +
					format("_%s.kml",label.c_str());

				VERBOSE_COUT << "Writing KML file: " << outfilname << endl;

				CFileOutputStream f(outfilname);

				// Header:
				f.printf(
					"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
					"<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n"
					"  <Document>\n"
					"    <name>Paths</name>\n"
					"    <description>Examples of paths...</description>\n"
					"    <Style id=\"yellowLineGreenPoly\">\n"
					"      <LineStyle>\n"
					//"        <color>7f00ffff</color>\n"
					"        <color>b000ffff</color>\n"
					"        <width>3</width>\n"
					"      </LineStyle>\n"
					"      <PolyStyle>\n"
					"        <color>b000ff00</color>\n"
					"      </PolyStyle>\n"
					"    </Style>\n"
					"    <Placemark>\n"
					"      <name>GPS path...</name>\n"
					"      <description>Path for sensor %s</description>\n"
					"      <styleUrl>#yellowLineGreenPoly</styleUrl>\n"
					"      <LineString>\n"
					"        <extrude>0</extrude>\n"
					"        <tessellate>0</tessellate>\n"
					"        %s\n"
					"       <coordinates> \n"
					,
					save_altitude ? "<altitudeMode>absolute</altitudeMode>" : "",
					label.c_str()
					);

				for (map<TTimeStamp,TGPSDataPoint>::const_iterator itP=D.path.begin();itP!=D.path.end();++itP)
				{
					const TGPSDataPoint &d = itP->second;
					// Format is: lon,lat[,alt]
					if (save_altitude)
							f.printf(" %.15f,%15f,%.3f\n",d.lon,d.lat,d.alt);
					else 	f.printf(" %.15f,%15f\n",d.lon,d.lat);
				}

				// end part:
				f.printf(
					"        </coordinates>\n"
					"      </LineString>\n"
					"    </Placemark>\n"
					"  </Document>\n"
					"</kml>\n");

			} // end for each sensor label

		} // end generate_KML

	}; // end CRawlogProcessor_ExportGPS_KML

	// Process
	// ---------------------------------
	CRawlogProcessor_ExportGPS_KML proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Now that the entire rawlog is parsed, do the actual output:
	proc.generate_KML();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";

}

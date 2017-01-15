/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPOINTSMAP_LIBLAS_H
#define CPOINTSMAP_LIBLAS_H

/** \file Include this file in your user application only if you have libLAS installed in your system */
#include <liblas/liblas.hpp>
#include <liblas/reader.hpp>
#include <liblas/writer.hpp>

#include <mrpt/maps/CPointsMap.h>
#include <string>
#include <iostream>
#include <fstream>

namespace mrpt
{
/** \ingroup mrpt_maps_grp */
namespace maps
{
	/** \addtogroup mrpt_maps_liblas_grp libLAS interface for CPointsMap (in #include <mrpt/maps/CPointsMaps_liblas.h>)
		*  \ingroup mrpt_maps_grp
		* @{ */

	/** Optional settings for saveLASFile() */
	struct LAS_WriteParams
	{
		// None.
	};

	/** Optional settings for loadLASFile() */
	struct LAS_LoadParams
	{
		// None.
	};

	/** Extra information gathered from the LAS file header */
	struct LAS_HeaderInfo
	{
		std::string FileSignature;
		std::string SystemIdentifier;
		std::string SoftwareIdentifier;
		std::string project_guid;
		std::string spatial_reference_proj4;  //!< Proj.4 string describing the Spatial Reference System.
		uint16_t    creation_year;//!< Creation date (Year number)
		uint16_t    creation_DOY; //!< Creation day of year

		LAS_HeaderInfo() : creation_year(0),creation_DOY(0)
		{}
	};

	/** Save the point cloud as an ASPRS LAS binary file (requires MRPT built against liblas). Refer to http://www.liblas.org/
		* \return false on any error */
	template <class POINTSMAP>
	bool saveLASFile(
		const POINTSMAP &ptmap,
		const std::string &filename, 
		const LAS_WriteParams & params = LAS_WriteParams() )
	{
		std::ofstream ofs;
		ofs.open(filename.c_str(), std::ios::out | std::ios::binary);

		if (!ofs.is_open())
		{
			std::cerr << "[saveLASFile] Couldn't write to file: " << filename << std::endl;
			return false;
		}

		// Fill-in header:
		// ---------------------------------
		liblas::Header header;
		const size_t nPts = ptmap.size();

		header.SetPointRecordsCount(nPts);

		// Create writer:
		// ---------------------------------
		liblas::Writer writer(ofs,header);

		const bool has_color = ptmap.hasColorPoints();
		const float col_fract = 255.0f;

		liblas::Point pt(&header);
		liblas::Color col;
		for (size_t i=0;i<nPts;i++)
		{
			float x,y,z,R,G,B;
			ptmap.getPoint(i, x,y,z, R,G,B);

			pt.SetX(x);
			pt.SetY(y);
			pt.SetZ(z);

			if (has_color)
			{
				col.SetRed( static_cast<uint16_t>(R*col_fract) );
				col.SetGreen( static_cast<uint16_t>(G*col_fract) );
				col.SetBlue( static_cast<uint16_t>(B*col_fract) );
				pt.SetColor(col);
			}

			if (!writer.WritePoint(pt)) 
			{
				std::cerr << "[saveLASFile] liblas returned error writing point #" << i << " to file.\n";
				return false;
			}
		}
		return true; // All ok.
	}

	/** Load the point cloud from an ASPRS LAS binary file (requires MRPT built against liblas). Refer to http://www.liblas.org/
		* \note Color (RGB) information will be taken into account if using the derived class mrpt::maps::CColouredPointsMap
		* \return false on any error */
	template <class POINTSMAP>
	bool loadLASFile(
		POINTSMAP &ptmap,
		const std::string &filename, 
		LAS_HeaderInfo &out_headerInfo, 
		const LAS_LoadParams &params = LAS_LoadParams() )
	{
		using namespace std;
		ptmap.clear();

		std::ifstream ifs;
		ifs.open(filename.c_str(), std::ios::in | std::ios::binary);

		if (!ifs.is_open())
		{
			std::cerr << "[loadLASFile] Couldn't open file: " << filename << std::endl;
			return false;
		}

		// Create LAS reader:
		// ---------------------
		liblas::Reader reader(ifs);

		// Parse header info:
		// ---------------------
		liblas::Header const& header = reader.GetHeader();
		const size_t nPts = header.GetPointRecordsCount();
		ptmap.reserve(nPts);

		out_headerInfo.FileSignature      = header.GetFileSignature();
		out_headerInfo.SystemIdentifier   = header.GetSystemId();
		out_headerInfo.SoftwareIdentifier = header.GetSoftwareId();
#if LIBLAS_VERSION_NUM < 1800
		out_headerInfo.project_guid       = header.GetProjectId().to_string();
#else
		out_headerInfo.project_guid       = boost::lexical_cast<std::string>(header.GetProjectId());
#endif
		out_headerInfo.spatial_reference_proj4 = header.GetSRS().GetProj4();
		out_headerInfo.creation_year = header.GetCreationYear();
		out_headerInfo.creation_DOY  = header.GetCreationDOY();

		// Load points:
		// ---------------------
		const bool has_color = ptmap.hasColorPoints();
		const float col_fract = 1.0f/255.0f;
		while (reader.ReadNextPoint())
		{
			liblas::Point const& p = reader.GetPoint();

			if (has_color)
			{
				liblas::Color const& col = p.GetColor();
				ptmap.insertPoint( p.GetX(),p.GetY(),p.GetZ(), col.GetRed()*col_fract,col.GetGreen()*col_fract,col.GetBlue()*col_fract );
			}
			else
			{
				ptmap.insertPoint( p.GetX(),p.GetY(),p.GetZ() );
			}
		}

		if (ptmap.size()!=nPts)
			cerr << "[loadLASFile] Header says point count is " << nPts << " but only " << ptmap.size() << " were really parsed in.\n";

		ptmap.mark_as_modified();

		return true; // All ok.
	}
	/** @} */
}
} // End of namespaces

#endif

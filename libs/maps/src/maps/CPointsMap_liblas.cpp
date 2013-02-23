/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/maps.h>  // Precompiled header

#include <mrpt/slam/CPointsMap.h>
#include <mrpt/slam/CSimplePointsMap.h>

#if MRPT_HAS_LIBLAS
#   include <liblas/liblas.hpp>
#	include <liblas/lasreader.hpp>
#	include <liblas/laswriter.hpp>
#endif

using namespace mrpt::slam;
using namespace std;

/*---------------------------------------------------------------
				saveLASFile
  ---------------------------------------------------------------*/
bool CPointsMap::saveLASFile(const std::string &filename, const LAS_WriteParams & params  ) const
{
#if MRPT_HAS_LIBLAS
	std::ofstream ofs;
	ofs.open(filename.c_str(), ios::out | ios::binary);

	if (!ofs.is_open())
	{
		cerr << "[CPointsMap::saveLASFile] Couldn't write to file: " << filename << endl;
		return false;
	}

	// Fill-in header:
	// ---------------------------------
	liblas::LASHeader header;
	const size_t nPts = this->size();

	header.SetPointRecordsCount(nPts);

	// Create writer:
	// ---------------------------------
	liblas::LASWriter writer(ofs,header);

	const bool has_color = this->hasColorPoints();
	const float col_fract = 255.0f;

	liblas::LASPoint pt;
	liblas::LASColor col;
	for (size_t i=0;i<nPts;i++)
	{
		float x,y,z,R,G,B;
		this->getPoint(i, x,y,z, R,G,B);

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
			cerr << "[CPointsMap::saveLASFile] liblas returned error writing point #" << i << " to file.\n";
			return false;
		}
	}

	return true; // All ok.
#else
	THROW_EXCEPTION("Sorry: MRPT wasn't compiled with LAS support. Recompile with liblas.")
#endif
}

/*---------------------------------------------------------------
				loadLASFile
  ---------------------------------------------------------------*/
bool CPointsMap::loadLASFile(const std::string &filename, LAS_HeaderInfo &out_headerInfo, const LAS_LoadParams &params )
{
#if MRPT_HAS_LIBLAS
	this->clear();

	ifstream ifs;
	ifs.open(filename.c_str(), ios::in | ios::binary);

	if (!ifs.is_open())
	{
		cerr << "[CPointsMap::loadLASFile] Couldn't open file: " << filename << endl;
		return false;
	}

	// Create LAS reader:
	// ---------------------
	liblas::LASReader reader(ifs);

	// Parse header info:
	// ---------------------
	liblas::LASHeader const& header = reader.GetHeader();
	const size_t nPts = header.GetPointRecordsCount();
	this->reserve(nPts);

	out_headerInfo.FileSignature      = header.GetFileSignature();
	out_headerInfo.SystemIdentifier   = header.GetSystemId();
	out_headerInfo.SoftwareIdentifier = header.GetSoftwareId();
	out_headerInfo.project_guid       = header.GetProjectId().to_string();
	out_headerInfo.spatial_reference_proj4 = header.GetSRS().GetProj4();
	out_headerInfo.creation_year = header.GetCreationYear();
	out_headerInfo.creation_DOY  = header.GetCreationDOY();

	// Load points:
	// ---------------------
	const bool has_color = this->hasColorPoints();
	const float col_fract = 1.0f/255.0f;
	while (reader.ReadNextPoint())
	{
		liblas::LASPoint const& p = reader.GetPoint();

		if (has_color)
		{
			liblas::LASColor const& col = p.GetColor();
			this->insertPoint( p.GetX(),p.GetY(),p.GetZ(), col.GetRed()*col_fract,col.GetGreen()*col_fract,col.GetBlue()*col_fract );
		}
		else
		{
			this->insertPoint( p.GetX(),p.GetY(),p.GetZ() );
		}
	}

	if (this->size()!=nPts)
		cerr << "[CPointsMap::loadLASFile] Header says point count is " << nPts << " but only " << this->size() << " were really parsed in.\n";

	this->mark_as_modified();

	return true; // All ok.
#else
	THROW_EXCEPTION("Sorry: MRPT wasn't compiled with LAS support. Recompile with liblas.")
#endif
}


/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/CMatrixD.h>
#include <mrpt/utils/CTextFileLinesParser.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/stl_serialization.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system.h>

// Specializations of this class will also inherit from
// CDatasetParserBase to build a working dataset parser.
template <class OBS_TYPE> struct CDatasetParserTempl;

// Base of all parsers:
struct CDatasetParserBase
{
	CDatasetParserBase(RBASLAM_Params &cfg) :
		m_cfg(cfg),
		m_verbose_level(m_cfg.arg_verbose.getValue()),
		m_has_GT_map(false), //!< Do we have a ground truth map?
		m_has_GT_path(false), //!< Do we have a ground truth path?
		m_add_noise(false)
	{
		using namespace std;

		// Params check:
		// ------------------------------
		if (!m_cfg.arg_dataset.isSet())
			throw std::runtime_error("Error: argument -d or --dataset is mandatory.\nRun with --help to see all the options or visit http://www.mrpt.org/srba for docs and examples.\n");

		m_add_noise = m_cfg.arg_add_noise.getValue();

		load_obs();
		load_GT_map();
		load_GT_path();
	}

	// ========= Virtual methods: =========
	virtual void checkObsProperSize() const = 0;
	// ====================================


	// Load simulated sensor observations
	// ------------------------------------------------------
	void load_obs()
	{
		// Try with binary cached version (much faster to load!)
		bool obs_cache_found = false;

		const std::string sFil_OBS    = m_cfg.arg_dataset.getValue();
		const std::string sFil_OBSbin = mrpt::system::fileNameChangeExtension(sFil_OBS,"bin");

		if (mrpt::system::fileExists(sFil_OBSbin) && mrpt::system::fileExists(sFil_OBS) &&
			mrpt::system::getFileModificationTime(sFil_OBSbin)>mrpt::system::getFileModificationTime(sFil_OBS) )
		{
			try
			{
				mrpt::utils::CFileGZInputStream f( sFil_OBSbin ); // will throw if file not found

				if (m_verbose_level>=1) { cout << "Loading binary cache version of dataset...\n"; cout.flush(); }
				f >> m_OBS;

				obs_cache_found=true;
			}
			catch (std::exception &)
			{
				// Cache not found or corrupt, etc.
			}
		}

		// m_OBS: Columns are: FRAME_ID   FEAT_ID  [SENSOR-SPECIFIC FIELDS]
		if (!obs_cache_found)
		{
			// Cache not found: load from text:
			ASSERT_FILE_EXISTS_(sFil_OBS)

			if (m_verbose_level>=1) { cout << "Loading dataset file: \n -> "<<sFil_OBS<<" ...\n"; cout.flush();}
			m_OBS.loadFromTextFile(sFil_OBS);

			ASSERT_ABOVE_(m_OBS.getRowCount(),2)
		}
		const size_t nTotalObs = m_OBS.getRowCount();
		if (m_verbose_level>=1) { cout << "Loaded " << nTotalObs << " observations.\n";}


		if (!obs_cache_found)
		{
			// Save cache:
			try
			{
				mrpt::utils::CFileGZOutputStream f( sFil_OBSbin );
				if (m_verbose_level>=1) { cout << "Saving binary cache version of dataset...\n"; cout.flush();}
				f << m_OBS;
				if (m_verbose_level>=1) { cout << "done.\n"; cout.flush();}
			}
			catch(std::exception &)
			{
				cerr << "Warning: Ignoring error writing binary cache version of dataset.\n";
			}
		}
	}


	// Load GT map of landmarks (so we can compute relative LMs poses)
	// -----------------------------------------------------------------
	void load_GT_map()
	{
		if (!m_cfg.arg_gt_map.isSet())
			return;

		m_has_GT_map = true;

		bool map_cache_found = false;

		const std::string sFil_MAP    = m_cfg.arg_gt_map.getValue();
		const std::string sFil_MAPbin = mrpt::system::fileNameChangeExtension(sFil_MAP,"bin");

		if (mrpt::system::fileExists(sFil_MAPbin) && mrpt::system::fileExists(sFil_MAP) &&
			mrpt::system::getFileModificationTime(sFil_MAPbin)>mrpt::system::getFileModificationTime(sFil_MAP) )
		{
			try
			{
				mrpt::utils::CFileGZInputStream f( sFil_MAPbin ); // will throw if file not found

				if (m_verbose_level>=1) { cout << "Loading binary cache version of map...\n"; cout.flush();}
				f >> m_GT_MAP;

				map_cache_found=true;
			}
			catch (std::exception &)
			{
				// Cache not found or corrupt, etc.
			}
		}

		if (!map_cache_found)
		{
			ASSERT_FILE_EXISTS_(sFil_MAP)

			if (m_verbose_level>=1) { cout << "Loading dataset file: \n -> "<<sFil_MAP<<" ...\n"; cout.flush();}
			m_GT_MAP.loadFromTextFile(sFil_MAP);

			ASSERT_ABOVE_(m_GT_MAP.getRowCount(),0)
			ASSERT_(m_GT_MAP.getColCount()==2 || m_GT_MAP.getColCount()==3)
		}
		const size_t nTotalLMs = m_GT_MAP.size();
		if (m_verbose_level>=1) { cout << "Loaded " << nTotalLMs << " landmarks (ground truth map).\n";}

		if (!map_cache_found)
		{
			// Save cache:
			try
			{
				mrpt::utils::CFileGZOutputStream f( sFil_MAPbin );
				if (m_verbose_level>=1) { cout << "Saving binary cache version of map...\n"; cout.flush();}
				f << m_GT_MAP;
				if (m_verbose_level>=1) { cout << "done.\n"; cout.flush();}
			}
			catch(std::exception &)
			{
				cerr << "Warning: Ignoring error writing binary cache version of map.\n";
			}
		}
	}

	// Load GT poses (so we can compute relative LMs poses)
	// -----------------------------------------------------------------
	void load_GT_path()
	{
		if (!m_cfg.arg_gt_path.isSet())
			return;

		m_has_GT_path = true;

		bool map_cache_found = false;

		const std::string sFil_PATH    = m_cfg.arg_gt_path.getValue();
		const std::string sFil_PATHbin = mrpt::system::fileNameChangeExtension(sFil_PATH,"bin");

		if (mrpt::system::fileExists(sFil_PATHbin) && mrpt::system::fileExists(sFil_PATH) &&
			mrpt::system::getFileModificationTime(sFil_PATHbin)>mrpt::system::getFileModificationTime(sFil_PATH) )
		{
			try
			{
				mrpt::utils::CFileGZInputStream f( sFil_PATHbin ); // will throw if file not found

				if (m_verbose_level>=1) { cout << "Loading binary cache version of path...\n"; cout.flush();}
				f >> m_GT_path;

				map_cache_found=true;
			}
			catch (std::exception &)
			{
				// Cache not found or corrupt, etc.
			}
		}

		if (!map_cache_found)
		{
			ASSERT_FILE_EXISTS_(sFil_PATH)

			mrpt::utils::CTextFileLinesParser flp(sFil_PATH);
			std::istringstream ss;
			while (flp.getNextLine(ss))
			{
				unsigned int idx;
				double x,y,z,qr,qx,qy,qz;
				if ( (ss >> idx >> x >> y >> z >> qr >> qx >> qy >> qz) )
				{
					if (idx!=m_GT_path.size())
						THROW_EXCEPTION("Reading _GT_PATH file: Pose IDs expected in ascending order and starting at 0.")

					m_GT_path.resize(m_GT_path.size()+1);
					*m_GT_path.rbegin() = mrpt::poses::CPose3DQuat(x,y,z, mrpt::math::CQuaternionDouble(qr,qx,qy,qz) ) ;
				}
			}
		}
		const size_t nTotalPoses = m_GT_path.size();
		if (m_verbose_level>=1) { cout << "Loaded " << nTotalPoses << " trajectory poses (ground truth path).\n";}

		if (!map_cache_found)
		{
			// Save cache:
			try
			{
				mrpt::utils::CFileGZOutputStream f( sFil_PATHbin );
				if (m_verbose_level>=1) { cout << "Saving binary cache version of path...\n"; cout.flush();}
				f << m_GT_path;
				if (m_verbose_level>=1) { cout << "done.\n"; cout.flush();}
			}
			catch(std::exception &)
			{
				cerr << "Warning: Ignoring error writing binary cache version of path.\n";
			}
		}
	}

	inline const mrpt::math::CMatrixD & obs() const  { return m_OBS; }
	inline bool has_GT_map() const { return m_has_GT_map; }
	inline bool has_GT_path() const { return m_has_GT_path; }

	inline const mrpt::math::CMatrixD & gt_map() const  { return m_GT_MAP; }
	inline const mrpt::poses::CPose3DQuat & gt_path(size_t timestep) const {  ASSERT_BELOW_(timestep,m_GT_path.size()) return m_GT_path[timestep]; }

protected:
	RBASLAM_Params &m_cfg;
	int m_verbose_level;

	bool  m_has_GT_map; //!< Do we have a ground truth map?
	bool  m_has_GT_path; //!< Do we have a ground truth path?

	bool  m_add_noise;

	mrpt::math::CMatrixD   m_OBS;
	mrpt::math::CMatrixD   m_GT_MAP;
	mrpt::aligned_containers<mrpt::poses::CPose3DQuat>::vector_t  m_GT_path;


};


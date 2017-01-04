/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "rawlog-edit-declarations.h"

#include <mrpt/obs/CObservationOdometry.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;

// ======================================================================
//		op_export_odometry_txt
// ======================================================================
DECLARE_OP_FUNCTION(op_export_odometry_txt)
{
	// A class to do this operation:
	class CRawlogProcessor_ExportODO_TXT : public CRawlogProcessorOnEachObservation
	{
	protected:
		string	m_inFile;

		map<string, FILE*>	lstFiles;
		string				m_filPrefix;

	public:
		size_t				m_entriesSaved;


		CRawlogProcessor_ExportODO_TXT(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose),
			m_entriesSaved(0)
		{
			getArgValue<string>(cmdline,"input",m_inFile);

			m_filPrefix =
				extractFileDirectory(m_inFile) +
				extractFileName(m_inFile);
		}

		// return false on any error.
		bool processOneObservation(CObservationPtr  &o)
		{
			if (!IS_CLASS(o, CObservationOdometry ) )
				return true;

			const CObservationOdometry* obs = CObservationOdometryPtr(o).pointer();

			map<string, FILE*>::const_iterator  it = lstFiles.find( obs->sensorLabel );

			FILE *f_this;

			if ( it==lstFiles.end() )	// A new file for this sensorlabel??
			{
				const std::string fileName =
					m_filPrefix+
					string("_") +
					fileNameStripInvalidChars( obs->sensorLabel.empty() ? string("ODOMETRY") : obs->sensorLabel ) +
					string(".txt");

				VERBOSE_COUT << "Writing odometry TXT file: " << fileName << endl;

				f_this = lstFiles[ obs->sensorLabel ] = os::fopen( fileName.c_str(), "wt");
				if (!f_this)
					THROW_EXCEPTION_CUSTOM_MSG1("Cannot open output file for write: %s", fileName.c_str() );

				// The first line is a description of the columns:
				::fprintf(f_this,
					"%% "
					"%14s "				// TIMESTAMP
					"%18s %18s %18s "	// GLOBAL_ODO_{x,y,phi}
					"%18s %18s %18s "	// HAS, TICKS_L/R
					"%18s %18s %18s "	// HAS, V,W
					"\n"
					,
					"Time",
					"GLOBAL_ODO_X","GLOBAL_ODO_Y","GLOBAL_ODO_PHI_RAD",
					"HAS_ENCODERS","LEFT_ENC_INCR_TICKS","RIGHT_ENC_INCR_TICKS",
					"HAS_VELOCITIES","LIN_SPEED_MetPerSec","ANG_SPEED_RadPerSec"
					);
			}
			else
				f_this = it->second;

            // For each entry in this sequence: Compute the timestamp and save all 15 values:
            ASSERT_(obs->timestamp!=INVALID_TIMESTAMP);
            TTimeStamp	t  = obs->timestamp;

            double 	sampleTime = timestampTotime_t(t);

            // Time:
			::fprintf(f_this,
				"%14.4f " // TIMESTAMP
				"%18.5f %18.5f %18.5f "	// GLOBAL_ODO_{x,y,phi}
				"%18i %18i %18i "	// HAS, TICKS_L/R
				"%18i %18.5f %18.5f"	// HAS, V,W
				"\n"
				,
				sampleTime, 
				obs->odometry.x(), obs->odometry.y(), obs->odometry.phi(),
				static_cast<int>(obs->hasEncodersInfo ? 1:0), static_cast<int>(obs->encoderLeftTicks), static_cast<int>(obs->encoderRightTicks), 
				static_cast<int>(obs->hasVelocities ? 1:0), obs->velocityLocal.vx, obs->velocityLocal.omega
				);
			m_entriesSaved++;
			return true; // All ok
		}

		// Destructor: close files and generate summary files:
		~CRawlogProcessor_ExportODO_TXT()
		{
			for (map<string, FILE*>::const_iterator  it=lstFiles.begin();it!=lstFiles.end();++it)
			{
				os::fclose(it->second);
			}

			// Save the joint file:
			// -------------------------
			VERBOSE_COUT << "Number of different odometry sensorLabels  : " << lstFiles.size() << endl;

			lstFiles.clear();
		} // end of destructor

	};

	// Process
	// ---------------------------------
	CRawlogProcessor_ExportODO_TXT proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Number of records saved           : " << proc.m_entriesSaved << "\n";
}


// ======================================================================
//		op_recalc_odometry
// ======================================================================
DECLARE_OP_FUNCTION(op_recalc_odometry)
{
	// A class to do this operation:
	class CRawlogProcessor_RecalcODO : public CRawlogProcessorOnEachObservation
	{
	protected:
		TOutputRawlogCreator	outrawlog;

		double KL,KR,D;
		mrpt::poses::CPose2D  m_odo_accum;
		bool m_odo_accum_valid;

	public:
		size_t				m_entriesSaved;

		CRawlogProcessor_RecalcODO(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose),
			m_odo_accum_valid(false),
			m_entriesSaved(0)
		{
			if (!getArgValue<double>(cmdline,"odo-KL",KL)) throw std::runtime_error("This operation requires --odo-KL");
			if (!getArgValue<double>(cmdline,"odo-KR",KR)) throw std::runtime_error("This operation requires --odo-KR");
			if (!getArgValue<double>(cmdline,"odo-D",D)) throw std::runtime_error("This operation requires --odo-D");
		}

		// return false on any error.
		bool processOneObservation(CObservationPtr  &o)
		{
			if (!IS_CLASS(o, CObservationOdometry ) )
				return true;

			CObservationOdometry* obs = CObservationOdometryPtr(o).pointer();

			if (!obs->hasEncodersInfo)
				throw std::runtime_error("CObservationOdometry entry does not contain encoder info, cannot recalculate odometry!"); 

			CActionRobotMovement2D auxOdoIncr;
			auxOdoIncr.hasEncodersInfo = true;
			auxOdoIncr.encoderLeftTicks = obs->encoderLeftTicks;
			auxOdoIncr.encoderRightTicks = obs->encoderRightTicks;
			auxOdoIncr.computeFromEncoders( KL,KR, D );

			if (!m_odo_accum_valid)
			{
				m_odo_accum_valid=true;
				m_odo_accum = obs->odometry;
				// and don't modify this odo val.
			}
			else
			{
				obs->odometry = m_odo_accum + auxOdoIncr.rawOdometryIncrementReading;
				m_odo_accum = obs->odometry;
			}
			m_entriesSaved++;
			return true; // All ok
		}

		// This method can be reimplemented to save the modified object to an output stream.
		virtual void OnPostProcess(
			mrpt::obs::CActionCollectionPtr &actions,
			mrpt::obs::CSensoryFramePtr     &SF,
			mrpt::obs::CObservationPtr      &obs)
		{
			ASSERT_((actions && SF) || obs)
			if (actions)
					outrawlog.out_rawlog << actions << SF;
			else	outrawlog.out_rawlog << obs;
		}

	};

	// Process
	// ---------------------------------
	CRawlogProcessor_RecalcODO proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Number of records saved           : " << proc.m_entriesSaved << "\n";
}

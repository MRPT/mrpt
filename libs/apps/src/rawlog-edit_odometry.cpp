/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "apps-precomp.h"  // Precompiled headers
//
#include <mrpt/obs/CObservationOdometry.h>

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::apps;
using namespace std;
using namespace mrpt::io;

DECLARE_OP_FUNCTION(op_export_txt);

// ======================================================================
//		op_export_odometry_txt
// ======================================================================
DECLARE_OP_FUNCTION(op_export_odometry_txt)
{
	// Forward:
	op_export_txt(in_rawlog, cmdline, verbose);
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
		TOutputRawlogCreator outrawlog;

		double KL, KR, D;
		mrpt::poses::CPose2D m_odo_accum;
		bool m_odo_accum_valid;

	   public:
		size_t m_entriesSaved;

		CRawlogProcessor_RecalcODO(
			CFileGZInputStream& in_rawlog, TCLAP::CmdLine& cmdline,
			bool Verbose)
			: CRawlogProcessorOnEachObservation(in_rawlog, cmdline, Verbose),
			  m_odo_accum_valid(false),
			  m_entriesSaved(0)
		{
			if (!getArgValue<double>(cmdline, "odo-KL", KL))
				throw std::runtime_error("This operation requires --odo-KL");
			if (!getArgValue<double>(cmdline, "odo-KR", KR))
				throw std::runtime_error("This operation requires --odo-KR");
			if (!getArgValue<double>(cmdline, "odo-D", D))
				throw std::runtime_error("This operation requires --odo-D");
		}

		// return false on any error.
		bool processOneObservation(CObservation::Ptr& o) override
		{
			if (!IS_CLASS(*o, CObservationOdometry)) return true;

			auto* obs = dynamic_cast<CObservationOdometry*>(o.get());

			if (!obs->hasEncodersInfo)
				throw std::runtime_error(
					"CObservationOdometry entry does not contain encoder info, "
					"cannot recalculate odometry!");

			CActionRobotMovement2D auxOdoIncr;
			auxOdoIncr.hasEncodersInfo = true;
			auxOdoIncr.encoderLeftTicks = obs->encoderLeftTicks;
			auxOdoIncr.encoderRightTicks = obs->encoderRightTicks;
			auxOdoIncr.computeFromEncoders(KL, KR, D);

			if (!m_odo_accum_valid)
			{
				m_odo_accum_valid = true;
				m_odo_accum = obs->odometry;
				// and don't modify this odo val.
			}
			else
			{
				obs->odometry =
					m_odo_accum + auxOdoIncr.rawOdometryIncrementReading;
				m_odo_accum = obs->odometry;
			}
			m_entriesSaved++;
			return true;  // All ok
		}

		// This method can be reimplemented to save the modified object to an
		// output stream.
		void OnPostProcess(
			mrpt::obs::CActionCollection::Ptr& actions,
			mrpt::obs::CSensoryFrame::Ptr& SF,
			mrpt::obs::CObservation::Ptr& obs) override
		{
			ASSERT_((actions && SF) || obs);
			if (actions) (*outrawlog.out_rawlog) << actions << SF;
			else
				(*outrawlog.out_rawlog) << obs;
		}
	};

	// Process
	// ---------------------------------
	CRawlogProcessor_RecalcODO proc(in_rawlog, cmdline, verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse
				 << "\n";
	VERBOSE_COUT << "Number of records saved           : "
				 << proc.m_entriesSaved << "\n";
}

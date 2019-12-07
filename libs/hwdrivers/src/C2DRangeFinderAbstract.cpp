/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"  // Precompiled headers

#include <mrpt/hwdrivers/C2DRangeFinderAbstract.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CPlanarLaserScan.h>  // in library mrpt-maps
#include <mrpt/system/os.h>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::io;
using namespace mrpt::hwdrivers;

/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
C2DRangeFinderAbstract::C2DRangeFinderAbstract()
	: mrpt::system::COutputLogger("C2DRangeFinderAbstract")
{
}

/*-------------------------------------------------------------
						Destructor
-------------------------------------------------------------*/
C2DRangeFinderAbstract::~C2DRangeFinderAbstract() = default;

void C2DRangeFinderAbstract::bindIO(const std::shared_ptr<CStream>& streamIO)
{
	m_csChangeStream.lock();
	m_stream = streamIO;
	m_csChangeStream.unlock();
}

/*-------------------------------------------------------------
						getObservation
-------------------------------------------------------------*/
void C2DRangeFinderAbstract::getObservation(
	bool& outThereIsObservation,
	mrpt::obs::CObservation2DRangeScan& outObservation, bool& hardwareError)
{
	m_csLastObservation.lock();

	hardwareError = m_hardwareError;
	outThereIsObservation = m_lastObservationIsNew;

	if (outThereIsObservation) outObservation = m_lastObservation;

	m_csLastObservation.unlock();
}

/*-------------------------------------------------------------
						doProcess
-------------------------------------------------------------*/
void C2DRangeFinderAbstract::doProcess()
{
	bool thereIs, hwError;

	if (!m_nextObservation)
		m_nextObservation = std::make_shared<CObservation2DRangeScan>();

	doProcessSimple(thereIs, *m_nextObservation, hwError);

	if (hwError)
	{
		m_state = ssError;
		MRPT_LOG_THROTTLE_ERROR(
			5.0, "Error reading from the sensor hardware. Will retry.");
	}

	if (thereIs)
	{
		m_state = ssWorking;

		appendObservation(m_nextObservation);
		m_nextObservation.reset();  // Create a new object in the next call
	}
}

void C2DRangeFinderAbstract::internal_notifyGoodScanNow()
{
	const auto new_t = mrpt::system::now();

	if (m_last_good_scan != INVALID_TIMESTAMP)
	{
		m_estimated_scan_period =
			0.9 * m_estimated_scan_period +
			0.1 * mrpt::system::timeDifference(m_last_good_scan, new_t);
	}
	m_last_good_scan = new_t;
	m_failure_waiting_scan_counter = 0;
}

// Returns true if ok, false if this seems to be an error
bool C2DRangeFinderAbstract::internal_notifyNoScanReceived()
{
	if (m_last_good_scan == INVALID_TIMESTAMP) return true;

	const double dt =
		mrpt::system::timeDifference(m_last_good_scan, mrpt::system::now());

	if (dt > 1.50 * m_estimated_scan_period)
		if (++m_failure_waiting_scan_counter >= m_max_missed_scan_failures)
			return false;

	return true;
}

/*-------------------------------------------------------------
						loadExclusionAreas
-------------------------------------------------------------*/
void C2DRangeFinderAbstract::loadCommonParams(
	const mrpt::config::CConfigFileBase& configSource,
	const std::string& iniSection)
{
	// Params:
	m_showPreview = configSource.read_bool(iniSection, "preview", false);

	// Load exclusion areas:
	m_lstExclusionPolys.clear();
	m_lstExclusionAngles.clear();

	unsigned int N = 1;

	for (;;)
	{
		vector<double> x, y, z_range;
		configSource.read_vector(
			iniSection, format("exclusionZone%u_x", N), vector<double>(0), x);
		configSource.read_vector(
			iniSection, format("exclusionZone%u_y", N), vector<double>(0), y);
		configSource.read_vector(
			iniSection, format("exclusionZone%u_z", N++), vector<double>(0),
			z_range);

		if (!x.empty() && !y.empty())
		{
			ASSERT_(x.size() == y.size());
			CObservation2DRangeScan::TListExclusionAreasWithRanges::value_type
				dat;

			dat.first.setAllVertices(x, y);
			if (z_range.empty())
			{
				dat.second.first = -std::numeric_limits<double>::max();
				dat.second.second = std::numeric_limits<double>::max();
			}
			else
			{
				ASSERTMSG_(
					z_range.size() == 2,
					"exclusionZone%u_z must be a range [z_min z_max]");
				ASSERT_(z_range[0] <= z_range[1]);

				dat.second.first = z_range[0];
				dat.second.second = z_range[1];
			}

			m_lstExclusionPolys.push_back(dat);
		}
		else
			break;
	}

	// Load forbiden angles;
	N = 1;

	for (;;)
	{
		const double ini = DEG2RAD(configSource.read_double(
			iniSection, format("exclusionAngles%u_ini", N), -1000));
		const double end = DEG2RAD(configSource.read_double(
			iniSection, format("exclusionAngles%u_end", N++), -1000));

		if (ini > -M_PI && end > -M_PI)
			m_lstExclusionAngles.emplace_back(ini, end);
		else
			break;
	}

	// Max. missed scan failures:
	m_max_missed_scan_failures = configSource.read_int(
		iniSection, "maxMissedScansToDeclareError", m_max_missed_scan_failures);
}

/*-------------------------------------------------------------
						filterByExclusionAreas
-------------------------------------------------------------*/
void C2DRangeFinderAbstract::filterByExclusionAreas(
	mrpt::obs::CObservation2DRangeScan& obs) const
{
	obs.filterByExclusionAreas(m_lstExclusionPolys);
}

/*-------------------------------------------------------------
						filterByExclusionAngles
-------------------------------------------------------------*/
void C2DRangeFinderAbstract::filterByExclusionAngles(
	mrpt::obs::CObservation2DRangeScan& obs) const
{
	obs.filterByExclusionAngles(m_lstExclusionAngles);
}

void C2DRangeFinderAbstract::processPreview(
	const mrpt::obs::CObservation2DRangeScan& obs)
{
	using namespace mrpt::opengl;

	// show laser scan
	if (m_showPreview)
	{
		if (!m_win)
		{
			string caption = string("Preview of ") + m_sensorLabel;
			m_win = mrpt::gui::CDisplayWindow3D::Create(caption, 640, 480);
			m_win->setCameraAzimuthDeg(180);
			m_win->setCameraElevationDeg(90);
			COpenGLScene::Ptr& theScene = m_win->get3DSceneAndLock();
			theScene->insert(std::make_shared<CAxis>(
				-300, -300, -50, 300, 300, 50, 1.0, 3, true));
			m_win->unlockAccess3DScene();
		}

		if (m_win && m_win->isOpen())
		{
			COpenGLScene::Ptr& theScene = m_win->get3DSceneAndLock();
			opengl::CPlanarLaserScan::Ptr laser;
			CRenderizable::Ptr obj = theScene->getByName("laser");
			if (!obj)
			{
				laser = std::make_shared<opengl::CPlanarLaserScan>();
				laser->setName("laser");
				laser->setScan(obs);
				theScene->insert(laser);
			}
			else
			{
				laser = std::dynamic_pointer_cast<CPlanarLaserScan>(obj);
				laser->setScan(obs);
			}
			m_win->unlockAccess3DScene();
			m_win->forceRepaint();
		}  // end if
	}  // end if
}

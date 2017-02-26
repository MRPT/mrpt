/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/math/wrap2pi.h>
#if MRPT_HAS_MATLAB
#	include <mexplus.h>
#endif

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservation2DRangeScan, CObservation,mrpt::obs)

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CObservation2DRangeScan::CObservation2DRangeScan( ) :
	m_scan(),
	m_intensity(),
	m_validRange(),
	m_has_intensity(false),
	scan( m_scan ), // proxy ctor
	intensity( m_intensity ), // proxy ctor
	validRange( m_validRange ), // proxy ctor
	aperture( M_PIf ),
	rightToLeft( true ),
	maxRange( 80.0f ),
	sensorPose(),
	stdError( 0.01f ),
	beamAperture(0),
	deltaPitch(0),
	m_cachedMap()
{
}

CObservation2DRangeScan::CObservation2DRangeScan(const CObservation2DRangeScan &o ) :
	scan( m_scan ), // proxy ctor
	intensity( m_intensity ), // proxy ctor
	validRange( m_validRange ) // proxy ctor
{
	*this = o; // rely on compiler-generated copy op + the custom = operator in proxies.
}

/*---------------------------------------------------------------
							Destructor
 ---------------------------------------------------------------*/
CObservation2DRangeScan::~CObservation2DRangeScan()
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservation2DRangeScan::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 7;
	else
	{
		// The data
		out << aperture << rightToLeft << maxRange << sensorPose;
		uint32_t	N = scan.size();
		out << N;
		ASSERT_(validRange.size() == scan.size() );
		if (N)
		{
			out.WriteBufferFixEndianness( &m_scan[0], N );
			out.WriteBuffer( &m_validRange[0],sizeof(m_validRange[0])*N );
		}
		out << stdError;
		out << timestamp;
		out << beamAperture;

		out << sensorLabel;

		out << deltaPitch;

		out << hasIntensity();
		if(hasIntensity())
			out.WriteBufferFixEndianness( &m_intensity[0], N );
	}
}

/*---------------------------------------------------------------
  Filter out invalid points by a minimum distance, a maximum angle and a certain distance at the end (z-coordinate of the lasers must be provided)
 ---------------------------------------------------------------*/
void CObservation2DRangeScan::truncateByDistanceAndAngle(float min_distance, float max_angle, float min_height, float max_height, float h )
{
	// FILTER OUT INVALID POINTS!!
	std::vector<float>::iterator		itScan;
	std::vector<char>::iterator itValid;
	CPose3D						pose;
	unsigned int				k;
	unsigned int				nPts = scan.size();

	for( itScan = m_scan.begin(), itValid = m_validRange.begin(), k = 0;
		 itScan != m_scan.end();
		 itScan++, itValid++, k++ )
	{
		float ang	= fabs(k*aperture/nPts - aperture*0.5);
		float x		= (*itScan)*cos(ang);

		if( min_height != 0 || max_height != 0 )
		{
			ASSERT_( max_height > min_height );
			if( *itScan < min_distance || ang > max_angle || x > h - min_height || x < h - max_height )
				*itValid = false;
		} // end if
		else
			if( *itScan < min_distance || ang > max_angle )
				*itValid = false;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservation2DRangeScan::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		{
			CMatrix covSensorPose;
			in >> aperture >> rightToLeft  >> maxRange >> sensorPose >> covSensorPose;
			uint32_t		N,i;

			in >> N;

			this->resizeScan(N);
			if (N)
				in.ReadBufferFixEndianness( &m_scan[0], N);

			if (version>=1)
			{	// Load validRange:
				if (N)
					in.ReadBuffer( &m_validRange[0], sizeof(m_validRange[0])*N );
			}
			else
			{
				// validRange: Default values: If distance is not maxRange
				for (i=0;i<N;i++)
					m_validRange[i]= scan[i] < maxRange;
			}

			if (version>=2)
			{
				in >> stdError;
			}
			else
			{
				stdError = 0.01f;
			}

			if (version>=3)
			{
				in >> timestamp;
			}

			// Default values for newer versions:
			beamAperture = DEG2RAD(0.25f);

			deltaPitch  = 0;
			sensorLabel = "";

		} break;

	case 4:
	case 5:
	case 6:
	case 7:
		{
			uint32_t		N;

			CMatrix covSensorPose;
			in >> aperture >> rightToLeft  >> maxRange >> sensorPose;

			if (version<6) // covSensorPose was removed in version 6
				in  >> covSensorPose;

			in >> N;
			this->resizeScan(N);
			if (N)
			{
				in.ReadBufferFixEndianness( &m_scan[0], N);
				in.ReadBuffer( &m_validRange[0], sizeof(m_validRange[0])*N );
			}
			in >> stdError;
			in.ReadBufferFixEndianness( &timestamp, 1);
			in >> beamAperture;

			if (version>=5)
			{
				in >> sensorLabel;
				in >> deltaPitch;
			}
			else
			{
				sensorLabel = "";
				deltaPitch  = 0;
			}
			if (version>=7)
			{
				bool hasIntensity;
				in >> hasIntensity;
				setScanHasIntensity(hasIntensity);
				if (hasIntensity && N) {
					in.ReadBufferFixEndianness( &m_intensity[0], N);
				}
			}
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};

	m_cachedMap.clear();
}

/*---------------------------------------------------------------
  Implements the writing to a mxArray for Matlab
 ---------------------------------------------------------------*/
#if MRPT_HAS_MATLAB
// Add to implement mexplus::from template specialization
IMPLEMENTS_MEXPLUS_FROM( mrpt::obs::CObservation2DRangeScan )

mxArray* CObservation2DRangeScan::writeToMatlab() const
{
	const char* fields[] = {"class",	// Data common to any MRPT class
							"ts","sensorLabel",		// Data common to any observation
							"scan","validRange","intensity"	// Received raw data
							"aperture","rightToLeft","maxRange",	// Scan plane geometry and properties
							"stdError","beamAperture","deltaPitch",	// Ray properties
							"pose", // Sensor pose
							"map"}; // Points map
	mexplus::MxArray obs_struct( mexplus::MxArray::Struct(sizeof(fields)/sizeof(fields[0]),fields) );

	obs_struct.set("class", this->GetRuntimeClass()->className);

	obs_struct.set("ts", this->timestamp);
	obs_struct.set("sensorLabel", this->sensorLabel);

	obs_struct.set("scan", this->m_scan);
	// TODO: "validRange should be a vector<bool> for Matlab instead of vector<char>" ==> JLBC: It cannot be done like that since serializing "bool" is not cross-platform safe, while "char" is...
	obs_struct.set("validRange", this->m_validRange);
	obs_struct.set("intensity", this->m_intensity);
	obs_struct.set("aperture", this->aperture);
	obs_struct.set("rightToLeft", this->rightToLeft);
	obs_struct.set("maxRange", this->maxRange);
	obs_struct.set("stdError", this->stdError);
	obs_struct.set("beamAperture", this->beamAperture);
	obs_struct.set("deltaPitch", this->deltaPitch);
	obs_struct.set("pose", this->sensorPose);
	// TODO: obs_struct.set("map", ...)
	return obs_struct.release();
}
#endif

/*---------------------------------------------------------------
						isPlanarScan
 ---------------------------------------------------------------*/
bool  CObservation2DRangeScan::isPlanarScan( const double tolerance  ) const
{
	return sensorPose.isHorizontal(tolerance);
}

bool  CObservation2DRangeScan::hasIntensity() const
{
	return m_has_intensity;
}
void CObservation2DRangeScan::setScanHasIntensity(bool setHasIntensityFlag)
{
	m_has_intensity = setHasIntensityFlag;
}


/*---------------------------------------------------------------
						filterByExclusionAreas
 ---------------------------------------------------------------*/
void CObservation2DRangeScan::filterByExclusionAreas( const TListExclusionAreasWithRanges  &areas )
{
	if (areas.empty()) return;

	MRPT_START

	double	Ang, dA;
	size_t  sizeRangeScan = scan.size();

	ASSERT_(scan.size()==validRange.size());

	if (!sizeRangeScan) return;

	if (rightToLeft)
	{
		Ang = - 0.5 * aperture;
		dA  = aperture / (sizeRangeScan-1);
	}
	else
	{
		Ang = + 0.5 * aperture;
		dA  = - aperture / (sizeRangeScan-1);
	}

	std::vector<char>::iterator   valid_it;
	std::vector<float>::const_iterator  scan_it;

	for (scan_it=m_scan.begin(), valid_it=m_validRange.begin(); scan_it!=m_scan.end(); scan_it++, valid_it++)
	{
		if (! *valid_it)
		{
			Ang+=dA;
			continue; // Already it's invalid
		}

		// Compute point in 2D, local to the laser center:
		const double Lx = *scan_it * cos( Ang );
		const double Ly = *scan_it * sin( Ang );
		Ang+=dA;

		// To real 3D pose:
		double Gx,Gy,Gz;
		this->sensorPose.composePoint(Lx,Ly,0, Gx,Gy,Gz);

		// Filter by X,Y:
		for (TListExclusionAreasWithRanges::const_iterator i=areas.begin();i!=areas.end();++i)
		{
			if ( i->first.PointIntoPolygon(Gx,Gy) &&
				(Gz>=i->second.first && Gz<=i->second.second) )
			{
				*valid_it = false;
				break; // Go for next point
			}
		} // for each area
	} // for each point

	MRPT_END
}

/*---------------------------------------------------------------
						filterByExclusionAreas
 ---------------------------------------------------------------*/
void CObservation2DRangeScan::filterByExclusionAreas( const std::vector<mrpt::math::CPolygon>  &areas )
{
	if (areas.empty()) return;

	TListExclusionAreasWithRanges lst;
	for (size_t i=0;i<areas.size();i++)
	{
		TListExclusionAreasWithRanges::value_type dat;
		dat.first = areas[i];
		dat.second.first  = -std::numeric_limits<double>::max();
		dat.second.second =  std::numeric_limits<double>::max();

		lst.push_back(dat);
	}
	filterByExclusionAreas(lst);
}

/*---------------------------------------------------------------
						filterByExclusionAngles
 ---------------------------------------------------------------*/
void CObservation2DRangeScan::filterByExclusionAngles( const std::vector<std::pair<double,double> >  &angles )
{
	if (angles.empty()) return;

	MRPT_START

	double	Ang, dA;
	const size_t  sizeRangeScan = scan.size();

	ASSERT_(scan.size()==validRange.size());

	if (!sizeRangeScan) return;

	if (rightToLeft)
	{
		Ang = - 0.5 * aperture;
		dA  = aperture / (sizeRangeScan-1);
	}
	else
	{
		Ang = + 0.5 * aperture;
		dA  = - aperture / (sizeRangeScan-1);
	}

	// For each forbiden angle range:
	for (vector<pair<double,double> >::const_iterator itA=angles.begin();itA!=angles.end();++itA)
	{
		int ap_idx_ini = mrpt::math::wrapTo2Pi(itA->first-Ang) / dA;  // The signs are all right! ;-)
		int ap_idx_end = mrpt::math::wrapTo2Pi(itA->second-Ang) / dA;

		if (ap_idx_ini<0) ap_idx_ini=0;
		if (ap_idx_end<0) ap_idx_end=0;

		if (ap_idx_ini>(int)sizeRangeScan) ap_idx_ini=sizeRangeScan-1;
		if (ap_idx_end>(int)sizeRangeScan) ap_idx_end=sizeRangeScan-1;

		const size_t idx_ini = ap_idx_ini;
		const size_t idx_end = ap_idx_end;

		if (idx_end>=idx_ini)
		{
			for (size_t i=idx_ini;i<=idx_end;i++)
				m_validRange[i]=false;
		}
		else
		{
			for (size_t i=0;i<idx_end;i++)
				m_validRange[i]=false;

			for (size_t i=idx_ini;i<sizeRangeScan;i++)
				m_validRange[i]=false;
		}
	}

	MRPT_END
}

namespace mrpt
{
	namespace obs
	{
		// Tricky way to call to a library that depends on us, a sort of "run-time" linking:
		//  ptr_internal_build_points_map_from_scan2D is a functor in "mrpt-obs", set by "mrpt-maps" at its startup.
		void OBS_IMPEXP (*ptr_internal_build_points_map_from_scan2D)(const mrpt::obs::CObservation2DRangeScan &obs, mrpt::maps::CMetricMapPtr &out_map, const void *insertOps) = NULL;
	}
}

/*---------------------------------------------------------------
						internal_buildAuxPointsMap
  ---------------------------------------------------------------*/
void CObservation2DRangeScan::internal_buildAuxPointsMap( const void *options ) const
{
	if (!ptr_internal_build_points_map_from_scan2D)
		throw std::runtime_error("[CObservation2DRangeScan::buildAuxPointsMap] ERROR: This function needs linking against mrpt-maps.\n");

	(*ptr_internal_build_points_map_from_scan2D)(*this,m_cachedMap, options);
}

/** Fill out a T2DScanProperties structure with the parameters of this scan */
void CObservation2DRangeScan::getScanProperties(T2DScanProperties& p) const
{
	p.nRays       = this->scan.size();
	p.aperture    = this->aperture;
	p.rightToLeft = this->rightToLeft;
}

bool mrpt::obs::operator<(const T2DScanProperties&a, const T2DScanProperties&b)
{
	if (a.nRays != b.nRays) return a.nRays<b.nRays;
	if (a.aperture != b.aperture) return a.aperture < b.aperture;
	if (a.rightToLeft != b.rightToLeft) return a.rightToLeft;
	return false;
}


void CObservation2DRangeScan::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);
	o << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
	o << sensorPose.getHomogeneousMatrixVal() << sensorPose << endl;

	o << format("Samples direction: %s\n", (rightToLeft) ? "Right->Left" : "Left->Right");
	o << format("Points in the scan: %u\n", (unsigned)scan.size());
	o << format("Estimated sensor 'sigma': %f\n", stdError );
	o << format("Increment in pitch during the scan: %f deg\n", RAD2DEG( deltaPitch ));

	size_t i,inval = 0;
	for (i=0;i<scan.size();i++) if (!validRange[i]) inval++;
	o << format("Invalid points in the scan: %u\n", (unsigned)inval);

	o << format("Sensor maximum range: %.02f m\n", maxRange );
	o << format("Sensor field-of-view (\"aperture\"): %.01f deg\n", RAD2DEG(aperture) );

	o << "Raw scan values: [";
	for (i=0;i<scan.size();i++) o << format("%.03f ", scan[i] );
	o << "]\n";

	o << "Raw valid-scan values: [";
	for (i=0;i<validRange.size();i++) o << format("%u ", validRange[i] ? 1:0 );
	o << "]\n\n";

	if(hasIntensity()){
		o << "Raw intensity values: [";
		for (i=0;i<m_intensity.size();i++) o << format("%d ", m_intensity[i]);
		o << "]\n\n";
	}

}

float CObservation2DRangeScan::getScanRange(const size_t i) const
{
	ASSERT_BELOW_(i,m_scan.size());
	return m_scan[i];
}

void CObservation2DRangeScan::setScanRange(const size_t i, const float val)
{
	ASSERT_BELOW_(i,m_scan.size());
	m_scan[i] = val;
}

int CObservation2DRangeScan::getScanIntensity(const size_t i) const
{
	ASSERT_BELOW_(i,m_intensity.size());
	return m_intensity[i];
}
void CObservation2DRangeScan::setScanIntensity(const size_t i, const int val)
{
	ASSERT_BELOW_(i,m_intensity.size());
	m_intensity[i] = val;
}

bool  CObservation2DRangeScan::getScanRangeValidity(const size_t i) const
{
	ASSERT_BELOW_(i,m_validRange.size());
	return m_validRange[i] != 0;
}
void CObservation2DRangeScan::setScanRangeValidity(const size_t i, const bool val)
{
	ASSERT_BELOW_(i,m_validRange.size());
	m_validRange[i] = val ? 1:0;
}

void CObservation2DRangeScan::resizeScan(const size_t len)
{
	const size_t capacity = mrpt::utils::length2length4N(len);
	m_scan.reserve(capacity);
	m_intensity.reserve(capacity);
	m_validRange.reserve(capacity);

	m_scan.resize(len);
	m_intensity.resize(len);
	m_validRange.resize(len);
}

void CObservation2DRangeScan::resizeScanAndAssign(const size_t len, const float rangeVal, const bool rangeValidity, const int32_t rangeIntensity)
{
	const size_t capacity = mrpt::utils::length2length4N(len);
	m_scan.reserve(capacity);
	m_intensity.reserve(capacity);
	m_validRange.reserve(capacity);

	m_scan.assign(len, rangeVal);
	m_validRange.assign(len, rangeValidity);
	m_intensity.assign(len, rangeIntensity);
}

size_t CObservation2DRangeScan::getScanSize() const
{
	return m_scan.size();
}

void CObservation2DRangeScan::loadFromVectors(size_t nRays, const float *scanRanges, const char *scanValidity )
{
	ASSERT_(scanRanges);
	ASSERT_(scanValidity);
	resizeScan(nRays);
	for (size_t i=0;i<nRays;i++) {
		m_scan[i] = scanRanges[i];
		m_validRange[i] = scanValidity[i];
	}
}


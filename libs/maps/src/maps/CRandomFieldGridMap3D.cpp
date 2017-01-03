/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/maps/CRandomFieldGridMap3D.h>
#include <mrpt/utils/CConfigFileBase.h>

//#include <mrpt/math/CMatrix.h>
//#include <mrpt/math/utils.h>
//#include <mrpt/utils/CTimeLogger.h>
//#include <mrpt/utils/color_maps.h>
//#include <mrpt/utils/round.h>
//#include <mrpt/utils/CFileGZInputStream.h>
//#include <mrpt/opengl/CSetOfObjects.h>
//#include <mrpt/opengl/CSetOfTriangles.h>
//#include <numeric>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CRandomFieldGridMap3D, CSerializable,mrpt::maps)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CRandomFieldGridMap3D::CRandomFieldGridMap3D(
	double x_min, double x_max,
	double y_min, double y_max,
	double z_min, double z_max,
	double voxel_size,
	bool call_initialize_now
) :
	CDynamicGrid3D<TRandomFieldVoxel>( x_min,x_max,y_min,y_max, z_min, z_max, voxel_size /*xy*/, voxel_size /*z*/ )
{
	if (call_initialize_now)
		this->initialize();
}

/** Changes the size of the grid, erasing previous contents. \sa resize */
void CRandomFieldGridMap3D::setSize(
	const double x_min, const double x_max,
	const double y_min, const double y_max,
	const double z_min, const double z_max,
	const double resolution_xy, const double resolution_z,
	const  TRandomFieldVoxel* fill_value)
{
	CDynamicGrid3D<TRandomFieldVoxel>::setSize(x_min, x_max, y_min, y_max, z_min, z_max, resolution_xy, resolution_z, fill_value);
	this->initialize();
}

void CRandomFieldGridMap3D::initialize()
{
	// Set the gridmap (m_map) to initial values:
	TRandomFieldVoxel  def(0, 0); // mean, std
	fill(def);

	{
		// Initiating prior (fully connected)\n";
		//---------------------------------------------------------------
		// Load default values for H_prior without Occupancy information:
		//---------------------------------------------------------------
		size_t cx = 0;
		size_t cy = 0;
		for (size_t j = 0; j<nodeCount; j++)
		{
			//Factor with the right node: H_ji = - Lamda_prior
			//-------------------------------------------------
			if (cx<(m_size_x - 1))
			{
				size_t i = j + 1;

				TPriorFactorGMRF new_prior(*this);
				new_prior.node_id_i = i;
				new_prior.node_id_j = j;
				new_prior.Lambda = m_insertOptions_common->GMRF_lambdaPrior;

				m_mrf_factors_priors.push_back(new_prior);
				m_gmrf.addConstraint(*m_mrf_factors_priors.rbegin()); // add to graph
			}

			//Factor with the above node: H_ji = - Lamda_prior
			//-------------------------------------------------
			if (cy<(m_size_y - 1))
			{
				size_t i = j + m_size_x;

				TPriorFactorGMRF new_prior(*this);
				new_prior.node_id_i = i;
				new_prior.node_id_j = j;
				new_prior.Lambda = m_insertOptions_common->GMRF_lambdaPrior;

				m_mrf_factors_priors.push_back(new_prior);
				m_gmrf.addConstraint(*m_mrf_factors_priors.rbegin()); // add to graph
			}

			// Increment j coordinates (row(x), col(y))
			if (++cx >= m_size_x) {
				cx = 0;
				cy++;
			}
		} // end for "j"
	} // end if_use_Occupancy

	if (m_rfgm_verbose) {
		cout << " [CRandomFieldGridMap2D::clear] Prior built in " << tictac.Tac() << " s ----------\n";
	}

	if (m_rfgm_run_update_upon_clear) {
		//Solve system and update map estimation
		updateMapEstimation_GMRF();
	}

}

/*---------------------------------------------------------------
					TInsertionOptions
 ---------------------------------------------------------------*/
CRandomFieldGridMap3D::TInsertionOptions::TInsertionOptions() :
	GMRF_lambdaPrior			( 0.01f ),		// [GMRF model] The information (Lambda) of fixed map constraints
	GMRF_lambdaObs				( 10.0f ),		// [GMRF model] The initial information (Lambda) of each observation (this information will decrease with time)
	GMRF_skip_variance			(false)
{
}

void  CRandomFieldGridMap3D::TInsertionOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("GMRF_lambdaPrior						= %f\n", GMRF_lambdaPrior);
	out.printf("GMRF_lambdaObs	                        = %f\n", GMRF_lambdaObs);
}

void  CRandomFieldGridMap3D::TInsertionOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	GMRF_lambdaPrior		= iniFile.read_float(section.c_str(),"GMRF_lambdaPrior",GMRF_lambdaPrior);
	GMRF_lambdaObs			= iniFile.read_float(section.c_str(),"GMRF_lambdaObs",GMRF_lambdaObs);
}


/*---------------------------------------------------------------
					resize
 ---------------------------------------------------------------*/
void  CRandomFieldGridMap3D::resize(
	double new_x_min, double new_x_max,
	double new_y_min, double new_y_max,
	double new_z_min, double new_z_max,
	const TRandomFieldVoxel& defaultValueNewCells, double additionalMarginMeters)
{
	MRPT_START

	MRPT_TODO("IMPL");

	MRPT_END
}


bool mrpt::maps::CRandomFieldGridMap3D::saveAsCSV(const std::string & filName_mean, const std::string & filName_stddev) const
{
	MRPT_TODO("Impl");
	return false;
}

void CRandomFieldGridMap3D::updateMapEstimation()
{
	MRPT_TODO("Impl");
}

bool CRandomFieldGridMap3D::insertIndividualReading(
	const double sensorReading,              //!< [in] The value observed in the (x,y,z) position
	const double sensorVariance,             //!< [in] The variance of the sensor observation
	const mrpt::math::TPoint3D & point,      //!< [in] The (x,y,z) location
	const TVoxelInterpolationMethod method,  //!< [in] Voxel interpolation method: how many voxels will be affected by the reading
	const bool update_map                    //!< [in] Run a global map update after inserting this observatin (algorithm-dependant)
)
{
	MRPT_TODO("Impl");

	return false;
}

void CRandomFieldGridMap3D::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		dyngridcommon_writeToStream(out);

		// To assure compatibility: The size of each cell:
		uint32_t n = static_cast<uint32_t>(sizeof(TRandomFieldVoxel));
		out << n;

		// Save the map contents:
		n = static_cast<uint32_t>(m_map.size());
		out << n;

		// Save the "m_map": This requires special handling for big endian systems:
#if MRPT_IS_BIG_ENDIAN
		for (uint32_t i = 0; i<n; i++)
		{
			out << m_map[i].mean_value << m_map[i].stddev_value;
		}
#else
		// Little endian: just write all at once:
		out.WriteBuffer(&m_map[0], sizeof(m_map[0])*m_map.size());
#endif

		out << insertionOptions.GMRF_lambdaObs
			<< insertionOptions.GMRF_lambdaPrior
			<< insertionOptions.GMRF_skip_variance;
	}
}

void CRandomFieldGridMap3D::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch (version)
	{
	case 0:
	{
		dyngridcommon_readFromStream(in);

		// To assure compatibility: The size of each cell:
		uint32_t n;
		in >> n;

		ASSERT_EQUAL_(n, static_cast<uint32_t>(sizeof(TRandomFieldVoxel)));
		// Load the map contents:
		in >> n;
		m_map.resize(n);

		// Read the note in writeToStream()
#if MRPT_IS_BIG_ENDIAN
		for (uint32_t i = 0; i<n; i++)
			in >> m_map[i].mean_value >> m_map[i].stddev_value;
#else
		// Little endian: just read all at once:
		in.ReadBuffer(&m_map[0], sizeof(m_map[0])*m_map.size());
#endif
		in >> insertionOptions.GMRF_lambdaObs
		   >> insertionOptions.GMRF_lambdaPrior
		   >> insertionOptions.GMRF_skip_variance;

	} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};

}



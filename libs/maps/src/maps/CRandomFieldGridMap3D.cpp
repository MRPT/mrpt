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

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CRandomFieldGridMap3D, CSerializable,mrpt::maps)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CRandomFieldGridMap3D::CRandomFieldGridMap3D(
	double x_min, double x_max,
	double y_min, double y_max,
	double z_min, double z_max,
	double voxel_size
) :
	CDynamicGrid3D<TRandomFieldVoxel>( x_min,x_max,y_min,y_max, z_min, z_max, voxel_size /*xy*/, voxel_size /*z*/ )
{
}

/** Changes the size of the grid, erasing previous contents. \sa resize */
void CRandomFieldGridMap3D::setSize(
	const double x_min, const double x_max,
	const double y_min, const double y_max,
	const double z_min, const double z_max,
	const double resolution_xy, const double resolution_z,
	const  TRandomFieldVoxel* fill_value)
{
	MRPT_TODO("Impl");
	//CDynamicGrid<TRandomFieldCell>::setSize(x_min,x_max,y_min,y_max,resolution,fill_value);
	//CMetricMap::clear();
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


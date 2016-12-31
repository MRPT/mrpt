/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/utils/CDynamicGrid3D.h>
#include <mrpt/utils/types_math.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/COutputLogger.h>

#include <mrpt/maps/link_pragmas.h>

#if EIGEN_VERSION_AT_LEAST(3,1,0) // eigen 3.1+
	#include <Eigen/SparseCore>
	#include <Eigen/SparseCholesky>
#endif

namespace mrpt
{
namespace maps
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CRandomFieldGridMap3D, CSerializable, MAPS_IMPEXP )

	/** The contents of each voxel in a CRandomFieldGridMap3D map.
	  * \ingroup mrpt_maps_grp
	 **/
#if defined(MRPT_IS_X86_AMD64) // Pragma defined to ensure no structure packing: since we'll serialize TRandomFieldVoxel to streams, we want it not to depend on compiler options, etc.
#pragma pack(push,1)
#endif
	struct MAPS_IMPEXP TRandomFieldVoxel
	{
		double mean_value, stddev_value; //!< Mean and sigma (standard deviation) estimated values for the voxel.

		/** Constructor */
		TRandomFieldVoxel(double _mean_value = .0, double _stddev_value = .0) :
			mean_value(_mean_value),
			stddev_value(_stddev_value)
		{ }
	};
#if defined(MRPT_IS_X86_AMD64)
#pragma pack(pop)
#endif

	/** CRandomFieldGridMap3D represents a 3D regular grid where each voxel is associated one real-valued property which is to be estimated by this class.
	  *
	  *  This class implements a Gaussian Markov Random Field (GMRF) estimator, with each voxel being connected to its 
	  *   6 immediate neighbors (Up, down, left, right, front, back).
	  *  - See papers: 
	  *    - "Time-variant gas distribution mapping with obstacle information", Monroy, J. G., Blanco, J. L., & Gonzalez-Jimenez, J. Autonomous Robots, 40(1), 1-16, 2016.
	  *
	  *  Note that this class does not derive from mrpt::maps::CMetricMap since the estimated values do not have sensor-especific semantics, 
	  *  i.e. the grid can be used to estimate temperature, gas concentration, etc.
	  *
	  * \sa mrpt::maps::CRandomFieldGridMap2D
	  * \ingroup mrpt_maps_grp
	  * \note [New in MRPT 1.5.0]
	  */
	class CRandomFieldGridMap3D :
		public mrpt::utils::CDynamicGrid3D<TRandomFieldVoxel>,
		public mrpt::utils::CSerializable,
		public mrpt::utils::COutputLogger
	{
		typedef utils::CDynamicGrid3D<TRandomFieldVoxel> BASE;

		// This must be added to any CSerializable derived class:
		DEFINE_VIRTUAL_SERIALIZABLE( CRandomFieldGridMap3D )
	public:
		static bool ENABLE_GMRF_PROFILER; //!< [default:false] Enables a profiler to show a performance report at application end.

		/** Constructor */
		CRandomFieldGridMap3D(
			double x_min = -2, double x_max = 2,
			double y_min = -2, double y_max = 2,
			double z_min = -2, double z_max = 2,
			double voxel_size = 0.5
			);

		/** Save the current estimated mean values to a CSV file with fields `x y z mean_value`.
		  * Optionally, std deviations can be also saved to another file with fields `x y z stddev_value`, if `filName_stddev` is provided.
		  * \return false on error writing to file
		  */
		bool saveAsCSV(const std::string  &filName_mean, const std::string  &filName_stddev = std::string() ) const;

		/** Parameters common to any derived class.
		  *  Derived classes should derive a new struct from this one, plus "public utils::CLoadableOptions",
		  *  and call the internal_* methods where appropiate to deal with the variables declared here.
		  *  Derived classes instantions of their "TInsertionOptions" MUST set the pointer "m_insertOptions_common" upon construction.
		  */
		struct MAPS_IMPEXP TInsertionOptions : public mrpt::utils::CLoadableOptions
		{
			TInsertionOptions();  //!< Default values loader

			/** See utils::CLoadableOptions */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase  &source,
				const std::string &section);

			void  dumpToTextStream(mrpt::utils::CStream	&out) const; //!< See utils::CLoadableOptions

			/** @name Gaussian Markov Random Fields method
			    @{ */
			double GMRF_lambdaPrior;		//!< The information (Lambda) of fixed map constraints
			double GMRF_lambdaObs;			//!< The initial information (Lambda) of each observation (this information will decrease with time)
			
			bool   GMRF_skip_variance;     //!< (Default:false) Skip the computation of the variance, just compute the mean
			/** @} */
		};

		TInsertionOptions insertionOptions; //!< \sa updateMapEstimation()

		/** Changes the size of the grid, maintaining previous contents. \sa setSize */
		virtual void resize(
			double new_x_min, double new_x_max,
			double new_y_min, double new_y_max,
			double new_z_min, double new_z_max,
			const TRandomFieldVoxel& defaultValueNewCells, double additionalMarginMeters = 2.0) MRPT_OVERRIDE;

		/** Changes the size of the grid, erasing previous contents. \sa resize */
		virtual void setSize(
			const double x_min, const double x_max,
			const double y_min, const double y_max,
			const double z_min, const double z_max,
			const double resolution_xy, const double resolution_z,
			const  TRandomFieldVoxel* fill_value = NULL) MRPT_OVERRIDE;

		enum TVoxelInterpolationMethod  {
			gimNearest = 0,
			gimBilinear
		};

		/** Direct update of the map with a reading in a given position of the map.
		  * \return false if point is out of the grid extension.
		  */
		bool insertIndividualReading(
			const double sensorReading,              //!< [in] The value observed in the (x,y,z) position
			const double sensorVariance,             //!< [in] The variance of the sensor observation
			const mrpt::math::TPoint3D & point,      //!< [in] The (x,y,z) location
			const TVoxelInterpolationMethod method,  //!< [in] Voxel interpolation method: how many voxels will be affected by the reading
			const bool update_map = true             //!< [in] Run a global map update after inserting this observatin (algorithm-dependant)
			);

		void updateMapEstimation(); //!< Run the method-specific procedure required to ensure that the mean & variances are up-to-date with all inserted observations, using parameters in insertionOptions

	protected:
		/** @name Auxiliary vars for GMRF method
		    @{ */
#if EIGEN_VERSION_AT_LEAST(3,1,0)
		std::vector<Eigen::Triplet<double> >  H_prior;	// the prior part of H
#endif
		Eigen::VectorXd g;								// Gradient vector
		size_t nPriorFactors;							// L
		size_t nObsFactors;								// M
		size_t nFactors;								// L+M
		std::multimap<size_t,size_t> cell_interconnections;		//Store the interconnections (relations) of each voxel with its neighbourds

		struct TObservationGMRF
		{
			double obsValue;
			double Lambda;
			//bool   time_invariant;						//if the observation will lose weight (lambda) as time goes on (default false)
		};

		std::vector<std::vector<TObservationGMRF> > activeObs;		//Vector with the active observations and their respective Information
		/** @} */

		/** The implementation of "insertObservation" for the Gaussian Markov Random Field map model.
		  * \param normReading Is a [0,1] normalized concentration reading.
		  * \param point Is the sensor location on the map
		  */
		void  insertObservation_GMRF(double normReading,const mrpt::math::TPoint2D &point, const bool update_map,const bool time_invariant);

		/** solves the minimum quadratic system to determine the new concentration of each voxel */
		void  updateMapEstimation_GMRF();

		/** Erase all the contents of the map */
		virtual void  internal_clear() MRPT_OVERRIDE;

		/** Check if two cells of the gridmap (m_map) are connected, based on the provided occupancy gridmap*/
		bool exist_relation_between2cells(
			const mrpt::maps::COccupancyGridMap2D *m_Ocgridmap,
			size_t cxo_min, 
			size_t cxo_max, 
			size_t cyo_min, 
			size_t cyo_max, 
			const size_t seed_cxo, 
			const size_t seed_cyo, 
			const size_t objective_cxo, 
			const size_t objective_cyo);
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CRandomFieldGridMap3D , CSerializable, MAPS_IMPEXP )


	} // End of namespace


	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<maps::CRandomFieldGridMap3D::TMapRepresentation>
		{
			typedef maps::CRandomFieldGridMap3D::TMapRepresentation enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(maps::CRandomFieldGridMap3D::mrKernelDM,          "mrKernelDM");
				m_map.insert(maps::CRandomFieldGridMap3D::mrKalmanFilter,      "mrKalmanFilter");
				m_map.insert(maps::CRandomFieldGridMap3D::mrKalmanApproximate, "mrKalmanApproximate");
				m_map.insert(maps::CRandomFieldGridMap3D::mrKernelDMV,         "mrKernelDMV");
				m_map.insert(maps::CRandomFieldGridMap3D::mrGMRF_G,			   "mrGMRF_G");
				m_map.insert(maps::CRandomFieldGridMap3D::mrGMRF_SD,		   "mrGMRF_SD");
			}
		};
	} // End of namespace
} // End of namespace


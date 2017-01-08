/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/utils/CDynamicGrid3D.h>
#include <mrpt/utils/types_math.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/graphs/ScalarFactorGraph.h>
#include <mrpt/math/lightweight_geom_data.h>

#include <mrpt/maps/link_pragmas.h>

// Fwdr decl:
class vtkStructuredGrid;

namespace mrpt
{
namespace maps
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CRandomFieldGridMap3D, mrpt::utils::CSerializable, MAPS_IMPEXP )

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
	  *  Usage:
	  *  - Define grid size with either constructor or via `setSize()`.
	  *  - Initialize the map with `initialize()`. This resets the contents of the map, so previously-added observations will be lost.
	  *  - Add observations of 3D voxels with `insertIndividualReading()`
	  *
	  * \sa mrpt::maps::CRandomFieldGridMap3D
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
		DEFINE_SERIALIZABLE( CRandomFieldGridMap3D )
	public:
		static bool ENABLE_GMRF_PROFILER; //!< [default:false] Enables a profiler to show a performance report at application end.

		/** Constructor. 
		  * If you set call_initialize_now to false, the object will be initialized immediately (without the heavy initialization of the GMRF), 
		  * but you then must call `setSize()` or `clear()` later to properly initialize the object before using it to insert observations.
		  */
		CRandomFieldGridMap3D(
			double x_min = -2, double x_max = 2,
			double y_min = -2, double y_max = 2,
			double z_min = -2, double z_max = 2,
			double voxel_size = 0.5, 
			bool call_initialize_now = true
			);

		/** Erases all added observations and start again with an empty gridmap. */
		void clear() MRPT_OVERRIDE;

		/** Save the current estimated mean values to a CSV file (compatible with Paraview) with fields `x y z mean_value`.
		  * Optionally, std deviations can be also saved to another file with fields `x y z stddev_value`, if `filName_stddev` is provided.
		  * \return false on error writing to file
		  * \sa saveAsVtkStructuredGrid
		  */
		bool saveAsCSV(const std::string  &filName_mean, const std::string  &filName_stddev = std::string() ) const;

		/** Save the current estimated grid to a VTK file (.vts) as a "structured grid". \sa saveAsCSV */
		bool saveAsVtkStructuredGrid(const std::string &fil) const;

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

		/** Changes the size of the grid, erasing previous contents.If `resolution_z`<0, the same resolution will be used for all dimensions x,y,z as given in `resolution_xy` \sa resize.*/
		virtual void setSize(
			const double x_min, const double x_max,
			const double y_min, const double y_max,
			const double z_min, const double z_max,
			const double resolution_xy, const double resolution_z = -1.0,
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
			const bool update_map                    //!< [in] Run a global map update after inserting this observation (algorithm-dependant)
			);

		void updateMapEstimation(); //!< Run the method-specific procedure required to ensure that the mean & variances are up-to-date with all inserted observations, using parameters in insertionOptions

		/** Returns the 3D grid contents as an VTK grid. */
		void getAsVtkStructuredGrid(vtkStructuredGrid* output, const std::string &label_mean = std::string("mean"), const std::string &label_stddev = std::string("stddev") ) const;

	protected:
		/** Internal: called called after each change of resolution, size, etc. to build the prior factor information */
		void internal_initialize(bool erase_prev_contents = true);

		mrpt::graphs::ScalarFactorGraph  m_gmrf;

		struct TObservationGMRF : public mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase
		{
			double obsValue;       //!< Observation value
			double Lambda;         //!< "Information" of the observation (=inverse of the variance)

			double evaluateResidual() const MRPT_OVERRIDE;
			double getInformation() const  MRPT_OVERRIDE;
			void evalJacobian(double &dr_dx) const MRPT_OVERRIDE;

			TObservationGMRF(CRandomFieldGridMap3D &parent) : obsValue(.0), Lambda(.0), m_parent(&parent) {}
		private:
			CRandomFieldGridMap3D *m_parent;
		};

		struct TPriorFactorGMRF : public mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase
		{
			double Lambda;         //!< "Information" of the observation (=inverse of the variance)

			double evaluateResidual() const MRPT_OVERRIDE;
			double getInformation() const MRPT_OVERRIDE;
			void evalJacobian(double &dr_dx_i, double &dr_dx_j) const MRPT_OVERRIDE;

			TPriorFactorGMRF(CRandomFieldGridMap3D &parent) : Lambda(.0), m_parent(&parent) {}
		private:
			CRandomFieldGridMap3D *m_parent;
		};

		std::vector<std::deque<TObservationGMRF> > m_mrf_factors_activeObs; //!< Vector with the active observations and their respective Information, for each map cell.
		std::deque<TPriorFactorGMRF>               m_mrf_factors_priors; //!< Vector with the precomputed priors for each GMRF model


	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CRandomFieldGridMap3D , mrpt::utils::CSerializable, MAPS_IMPEXP )

	} // End of namespace
} // End of namespace


/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/containers/CDynamicGrid3D.h>
#include <mrpt/math/types_math.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/graphs/ScalarFactorGraph.h>
#include <mrpt/math/lightweight_geom_data.h>

// Fwdr decl:
class vtkStructuredGrid;

namespace mrpt::maps
{
/** The contents of each voxel in a CRandomFieldGridMap3D map.
 * \ingroup mrpt_maps_grp
 **/
#if defined(MRPT_IS_X86_AMD64)  // Pragma defined to ensure no structure
// packing: since we'll serialize
// TRandomFieldVoxel to streams, we want it not
// to depend on compiler options, etc.
#pragma pack(push, 1)
#endif
struct TRandomFieldVoxel
{
	/** Mean and sigma (standard deviation) estimated values for the voxel. */
	double mean_value, stddev_value;

	/** Constructor */
	TRandomFieldVoxel(double _mean_value = .0, double _stddev_value = .0)
		: mean_value(_mean_value), stddev_value(_stddev_value)
	{
	}
};
#if defined(MRPT_IS_X86_AMD64)
#pragma pack(pop)
#endif

/** CRandomFieldGridMap3D represents a 3D regular grid where each voxel is
 * associated one real-valued property which is to be estimated by this class.
 *
 *  This class implements a Gaussian Markov Random Field (GMRF) estimator, with
 * each voxel being connected to its
 *   6 immediate neighbors (Up, down, left, right, front, back).
 *  - See papers:
 *    - "Time-variant gas distribution mapping with obstacle information",
 * Monroy, J. G., Blanco, J. L., & Gonzalez-Jimenez, J. Autonomous Robots,
 * 40(1), 1-16, 2016.
 *
 *  Note that this class does not derive from mrpt::maps::CMetricMap since the
 * estimated values do not have sensor-especific semantics,
 *  i.e. the grid can be used to estimate temperature, gas concentration, etc.
 *
 *  Usage:
 *  - Define grid size with either constructor or via `setSize()`.
 *  - Initialize the map with `initialize()`. This resets the contents of the
 * map, so previously-added observations will be lost.
 *  - Add observations of 3D voxels with `insertIndividualReading()`
 *
 * Custom connectivity patterns can be defined with setVoxelsConnectivity().
 *
 * \sa mrpt::maps::CRandomFieldGridMap3D
 * \ingroup mrpt_maps_grp
 * \note [New in MRPT 1.5.0]
 */
class CRandomFieldGridMap3D
	: public mrpt::containers::CDynamicGrid3D<TRandomFieldVoxel>,
	  public mrpt::serialization::CSerializable,
	  public mrpt::system::COutputLogger
{
	using BASE = mrpt::containers::CDynamicGrid3D<TRandomFieldVoxel>;

	DEFINE_SERIALIZABLE(CRandomFieldGridMap3D)
   public:
	/** [default:false] Enables a profiler to show a performance report at
	 * application end. */
	static bool ENABLE_GMRF_PROFILER;

	/** Constructor.
	 * If you set call_initialize_now to false, the object will be initialized
	 * immediately (without the heavy initialization of the GMRF),
	 * but you then must call `setSize()` or `clear()` later to properly
	 * initialize the object before using it to insert observations.
	 */
	CRandomFieldGridMap3D(
		double x_min = -2, double x_max = 2, double y_min = -2,
		double y_max = 2, double z_min = -2, double z_max = 2,
		double voxel_size = 0.5, bool call_initialize_now = true);

	/** Erases all added observations and start again with an empty gridmap. */
	void clear() override;

	/** Save the current estimated mean values to a CSV file (compatible with
	 * Paraview) with fields `x y z mean_value`.
	 * Optionally, std deviations can be also saved to another file with fields
	 * `x y z stddev_value`, if `filName_stddev` is provided.
	 * \return false on error writing to file
	 * \sa saveAsVtkStructuredGrid
	 */
	bool saveAsCSV(
		const std::string& filName_mean,
		const std::string& filName_stddev = std::string()) const;

	/** Save the current estimated grid to a VTK file (.vts) as a "structured
	 * grid". \sa saveAsCSV */
	bool saveAsVtkStructuredGrid(const std::string& fil) const;

	/** Parameters common to any derived class.
	 *  Derived classes should derive a new struct from this one, plus "public
	 * utils::CLoadableOptions",
	 *  and call the internal_* methods where appropiate to deal with the
	 * variables declared here.
	 *  Derived classes instantions of their "TInsertionOptions" MUST set the
	 * pointer "m_insertOptions_common" upon construction.
	 */
	struct TInsertionOptions : public mrpt::config::CLoadableOptions
	{
		/** Default values loader */
		TInsertionOptions();

		/** See utils::CLoadableOptions */
		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& source,
			const std::string& section) override;

		/** See utils::CLoadableOptions */
		void dumpToTextStream(std::ostream& out) const override;

		/** @name Gaussian Markov Random Fields method
			@{ */
		/** The information (Lambda) of fixed map constraints */
		double GMRF_lambdaPrior{0.01f};
		/** (Default:false) Skip the computation of the variance, just compute
		 * the mean */
		bool GMRF_skip_variance{false};
		/** @} */
	};

	/** \sa updateMapEstimation() */
	TInsertionOptions insertionOptions;

	/** Changes the size of the grid, maintaining previous contents. \sa setSize
	 */
	void resize(
		double new_x_min, double new_x_max, double new_y_min, double new_y_max,
		double new_z_min, double new_z_max,
		const TRandomFieldVoxel& defaultValueNewvoxels,
		double additionalMarginMeters = 2.0) override;

	/** Changes the size of the grid, erasing previous contents.If
	 * `resolution_z`<0, the same resolution will be used for all dimensions
	 * x,y,z as given in `resolution_xy` \sa resize.*/
	void setSize(
		const double x_min, const double x_max, const double y_min,
		const double y_max, const double z_min, const double z_max,
		const double resolution_xy, const double resolution_z = -1.0,
		const TRandomFieldVoxel* fill_value = nullptr) override;

	/** Base class for user-supplied objects capable of describing voxels
	 * connectivity, used to build prior factors of the MRF graph. \sa
	 * setvoxelsConnectivity() */
	struct ConnectivityDescriptor
	{
		using Ptr = std::shared_ptr<ConnectivityDescriptor>;
		// Virtual destructor for polymorphic type.
		virtual ~ConnectivityDescriptor() = default;
		/** Implement the check of whether node i=(icx,icy,icz) is connected
		 * with node j=(jcx,jcy,jcy).
		 * This visitor method will be called only for immediate neighbors.
		 * \return true if connected (and the "information" value should be also
		 * updated in out_edge_information), false otherwise.
		 */
		virtual bool getEdgeInformation(
			/** The parent map on which we are running */
			const CRandomFieldGridMap3D* parent,
			/** (cx,cy,cz) for node "i" */
			size_t icx, size_t icy, size_t icz,
			/** (cx,cy,cz) for node "j" */
			size_t jcx, size_t jcy, size_t jcz,
			/** Must output here the inverse of the variance of the constraint
			   edge. */
			double& out_edge_information) = 0;
	};

	/** Sets a custom object to define the connectivity between voxels. Must
	 * call clear() or setSize() afterwards for the changes to take place. */
	void setVoxelsConnectivity(
		const ConnectivityDescriptor::Ptr& new_connectivity_descriptor);

	enum TVoxelInterpolationMethod
	{
		gimNearest = 0,
		gimBilinear
	};

	/** Direct update of the map with a reading in a given position of the map.
	 * \return false if point is out of the grid extension.
	 */
	bool insertIndividualReading(
		/** [in] The value observed in the (x,y,z) position */
		const double sensorReading,
		/** [in] The variance of the sensor observation */
		const double sensorVariance,
		/** [in] The (x,y,z) location */
		const mrpt::math::TPoint3D& point,
		/** [in] Voxel interpolation method: how many voxels will be
		   affected by the reading */
		const TVoxelInterpolationMethod method,
		/** [in] Run a global map update after inserting this observation
		   (algorithm-dependant) */
		const bool update_map);

	/** Run the method-specific procedure required to ensure that the mean &
	 * variances are up-to-date with all inserted observations, using parameters
	 * in insertionOptions */
	void updateMapEstimation();

	/** Returns the 3D grid contents as an VTK grid. */
	void getAsVtkStructuredGrid(
		vtkStructuredGrid* output,
		const std::string& label_mean = std::string("mean"),
		const std::string& label_stddev = std::string("stddev")) const;

   protected:
	/** Internal: called called after each change of resolution, size, etc. to
	 * build the prior factor information */
	void internal_initialize(bool erase_prev_contents = true);

	/** Empty: default */
	ConnectivityDescriptor::Ptr m_gmrf_connectivity;

	mrpt::graphs::ScalarFactorGraph m_gmrf;

	struct TObservationGMRF
		: public mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase
	{
		/** Observation value */
		double obsValue;
		/** "Information" of the observation (=inverse of the variance) */
		double Lambda;

		double evaluateResidual() const override;
		double getInformation() const override;
		void evalJacobian(double& dr_dx) const override;

		TObservationGMRF(CRandomFieldGridMap3D& parent)
			: obsValue(.0), Lambda(.0), m_parent(&parent)
		{
		}

	   private:
		CRandomFieldGridMap3D* m_parent;
	};

	struct TPriorFactorGMRF
		: public mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase
	{
		/** "Information" of the observation (=inverse of the variance) */
		double Lambda;

		double evaluateResidual() const override;
		double getInformation() const override;
		void evalJacobian(double& dr_dx_i, double& dr_dx_j) const override;

		TPriorFactorGMRF(CRandomFieldGridMap3D& parent)
			: Lambda(.0), m_parent(&parent)
		{
		}

	   private:
		CRandomFieldGridMap3D* m_parent;
	};

	/** Vector with the active observations and their respective Information,
	 * for each map cell. */
	std::vector<std::deque<TObservationGMRF>> m_mrf_factors_activeObs;
	/** Vector with the precomputed priors for each GMRF model */
	std::deque<TPriorFactorGMRF> m_mrf_factors_priors;
};

}  // namespace mrpt::maps

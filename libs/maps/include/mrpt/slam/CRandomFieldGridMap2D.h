/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#ifndef CRandomFieldGridMap2D_H
#define CRandomFieldGridMap2D_H

#include <mrpt/utils/CDynamicGrid.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/slam/CMetricMap.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
namespace slam
{
	using namespace mrpt::utils;
	using namespace mrpt::poses;
	using namespace mrpt::math;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CRandomFieldGridMap2D , CMetricMap, MAPS_IMPEXP )

	// Pragma defined to ensure no structure packing: since we'll serialize TRandomFieldCell to streams, we want it not to depend on compiler options, etc.
#pragma pack(push,1)

	/** The contents of each cell in a CRandomFieldGridMap2D map.
	  * \ingroup mrpt_maps_grp
	 **/
	struct MAPS_IMPEXP TRandomFieldCell
	{
		/** Constructor */
		TRandomFieldCell(double kfmean_dm_mean = 1e-20, double kfstd_dmmeanw = 0) :
			kf_mean      (kfmean_dm_mean),
			kf_std       (kfstd_dmmeanw),
			dmv_var_mean (0)
		{ }

		// *Note*: Use unions to share memory between data fields, since only a set
		//          of the variables will be used for each mapping strategy.
		// You can access to a "TRandomFieldCell *cell" like: cell->kf_mean, cell->kf_std, etc..
		//  but accessing cell->kf_mean would also modify (i.e. ARE the same memory slot) cell->dm_mean, for example.

		// Note 2: If the number of type of fields are changed in the future,
		//   *PLEASE* also update the writeToStream() and readFromStream() methods!!

		union
		{
			double kf_mean; //!< [KF-methods only] The mean value of this cell
			double dm_mean; //!< [Kernel-methods only] The cumulative weighted readings of this cell

		};
		union
		{
			double kf_std;    //!< [KF-methods only] The standard deviation value of this cell
			double dm_mean_w; //!< [Kernel-methods only] The cumulative weights (concentration = alpha * dm_mean / dm_mean_w + (1-alpha)*r0 )
		};

		double dmv_var_mean;   //!< [Kernel DM-V only] The cumulative weighted variance of this cell
	};
#pragma pack(pop)

	/** CRandomFieldGridMap2D represents a 2D grid map where each cell is associated one real-valued property which is estimated by this map, either
	  *   as a simple value or as a probility distribution (for each cell).
	  *
	  *  There are a number of methods available to build the gas grid-map, depending on the value of
	  *    "TMapRepresentation maptype" passed in the constructor.
	  *
	  *  The following papers describe the mapping alternatives implemented here:
	  *		- mrKernelDM: A kernel-based method:
	  *		"Building gas concentration gridmaps with a mobile robot", Lilienthal, A. and Duckett, T., Robotics and Autonomous Systems, v.48, 2004.
	  *
	  *		- mrKernelDMV: A kernel-based method:
	  *		"A Statistical Approach to Gas Distribution Modelling with Mobile Robots--The Kernel DM+ V Algorithm"
	  * 	  , Lilienthal, A.J. and Reggente, M. and Trincavelli, M. and Blanco, J.L. and Gonzalez, J., IROS 2009.
	  *
	  *  Note that this class is virtual, since derived classes still have to implement:
	  *		- mrpt::slam::CMetricMap::computeObservationLikelihood()
	  *		- mrpt::slam::CMetricMap::internal_insertObservation()
	  *		- Serialization methods: writeToStream() and readFromStream()
	  *
	  * \sa mrpt::slam::CGasConcentrationGridMap2D, mrpt::slam::CWirelessPowerGridMap2D, mrpt::slam::CMetricMap, mrpt::utils::CDynamicGrid, The application icp-slam, mrpt::slam::CMultiMetricMap
	  * \ingroup mrpt_maps_grp
	  */
	class CRandomFieldGridMap2D : public CMetricMap, public utils::CDynamicGrid<TRandomFieldCell>
	{
		typedef utils::CDynamicGrid<TRandomFieldCell> BASE;

		// This must be added to any CSerializable derived class:
		DEFINE_VIRTUAL_SERIALIZABLE( CRandomFieldGridMap2D )
	public:

		/** Calls the base CMetricMap::clear
		  * Declared here to avoid ambiguity between the two clear() in both base classes.
		  */
		inline void clear() { CMetricMap::clear(); }

		// This method is just used for the ::saveToTextFile() method in base class.
		float cell2float(const TRandomFieldCell& c) const
		{
			return c.kf_mean;
		}

		/** The type of map representation to be used.
		  */
		enum TMapRepresentation
		{
			mrKernelDM = 0,   //
			mrAchim = 0,      // Another alias for "mrKernelDM", for backward compatibility
			mrKalmanFilter,
			mrKalmanApproximate,
			mrKernelDMV
		};

		/** Constructor
		  */
		CRandomFieldGridMap2D(
			TMapRepresentation	mapType = mrAchim,
            float				x_min = -2,
			float				x_max = 2,
			float				y_min = -2,
			float				y_max = 2,
			float				resolution = 0.1
			);

		/** Destructor */
		virtual ~CRandomFieldGridMap2D();

		 /** Returns true if the map is empty/no observation has been inserted (in this class it always return false,
		   * unless redefined otherwise in base classes)
		   */
		 virtual bool  isEmpty() const;


		/** Save the current map as a graphical file (BMP,PNG,...).
		  * The file format will be derived from the file extension (see  CImage::saveToFile )
		  *  It depends on the map representation model:
		  *		mrAchim: Each pixel is the ratio \f$ \sum{\frac{wR}{w}} \f$
		  *		mrKalmanFilter: Each pixel is the mean value of the Gaussian that represents each cell.
		  *
		  * \sa \a getAsBitmapFile()
		  */
		virtual void  saveAsBitmapFile(const std::string  &filName) const;

		/** Returns an image just as described in \a saveAsBitmapFile */
		virtual void  getAsBitmapFile(mrpt::utils::CImage &out_img) const;

		/** Parameters common to any derived class.
		  *  Derived classes should derive a new struct from this one, plus "public utils::CLoadableOptions",
		  *  and call the internal_* methods where appropiate to deal with the variables declared here.
		  *  Derived classes instantions of their "TInsertionOptions" MUST set the pointer "m_insertOptions_common" upon construction.
		  */
		struct MAPS_IMPEXP TInsertionOptionsCommon
		{
			TInsertionOptionsCommon();	//!< Default values loader

			/** See utils::CLoadableOptions */
			void  internal_loadFromConfigFile_common(
				const mrpt::utils::CConfigFileBase  &source,
				const std::string &section);

			void  internal_dumpToTextStream_common(CStream	&out) const; //!< See utils::CLoadableOptions

			/** @name Kernel methods (mrKernelDM, mrKernelDMV)
			    @{ */
			float	sigma;	//!< The sigma of the "Parzen"-kernel Gaussian
			float	cutoffRadius;	//!< The cutoff radius for updating cells.
            float	R_min,R_max;	//!< Limits for normalization of sensor readings.
			double	dm_sigma_omega;	//!< [DM/DM+V methods] The scaling parameter for the confidence "alpha" values (see the IROS 2009 paper; see CRandomFieldGridMap2D) */
			/** @} */

			/** @name Kalman-filter methods (mrKalmanFilter, mrKalmanApproximate)
			    @{ */
			float	KF_covSigma;	//!< The "sigma" for the initial covariance value between cells (in meters).
			float	KF_initialCellStd;	//!< The initial standard deviation of each cell's concentration (will be stored both at each cell's structure and in the covariance matrix as variances in the diagonal) (in normalized concentration units).
			float	KF_observationModelNoise;	//!< The sensor model noise (in normalized concentration units).
			float	KF_defaultCellMeanValue;	//!< The default value for the mean of cells' concentration.
			uint16_t	KF_W_size;	//!< [mrKalmanApproximate] The size of the window of neighbor cells.
			/** @} */
		};

		/** Changes the size of the grid, maintaining previous contents.
		  * \sa setSize
		  */
		virtual void  resize(		float	new_x_min,
									float	new_x_max,
									float	new_y_min,
									float	new_y_max,
									const TRandomFieldCell& defaultValueNewCells,
									float	additionalMarginMeters = 1.0f );

		/** Computes the ratio in [0,1] of correspondences between "this" and the "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
		 *   In the case of a multi-metric map, this returns the average between the maps. This method always return 0 for grid maps.
		 * \param  otherMap					  [IN] The other map to compute the matching with.
		 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen from "this".
		 * \param  minDistForCorr			  [IN] The minimum distance between 2 non-probabilistic map elements for counting them as a correspondence.
		 * \param  minMahaDistForCorr		  [IN] The minimum Mahalanobis distance between 2 probabilistic map elements for counting them as a correspondence.
		 *
		 * \return The matching ratio [0,1]
		 * \sa computeMatchingWith2D
		 */
		float  compute3DMatchingRatio(
				const CMetricMap						*otherMap,
				const CPose3D							&otherMapPose,
				float									minDistForCorr = 0.10f,
				float									minMahaDistForCorr = 2.0f
				) const;


		/** The implementation in this class just calls all the corresponding method of the contained metric maps.
		  */
		virtual void  saveMetricMapRepresentationToFile(
			const std::string	&filNamePrefix
			) const;

		/** Save a matlab ".m" file which represents as 3D surfaces the mean and a given confidence level for the concentration of each cell.
		  *  This method can only be called in a KF map model.
		  *  \sa getAsMatlab3DGraphScript
		  */
		virtual void  saveAsMatlab3DGraph(const std::string  &filName) const;

		/** Return a large text block with a MATLAB script to plot the contents of this map \sa saveAsMatlab3DGraph
		  *  This method can only be called in a KF map model.
		  */
		void  getAsMatlab3DGraphScript(std::string  &out_script) const;

		/** Returns a 3D object representing the map.
		  */
		virtual void  getAs3DObject ( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const;

		/** Return the type of the random-field grid map, according to parameters passed on construction.
		  */
		TMapRepresentation	 getMapType();

		/** Returns the prediction of the measurement at some (x,y) coordinates, and its certainty (in the form of the expected variance).
		  *  This methods is implemented differently for the different gas map types.
		  */
		virtual void predictMeasurement(
			const double	&x,
			const double	&y,
			double			&out_predict_response,
			double			&out_predict_response_variance );

		/** Return the mean and covariance vector of the full Kalman filter estimate (works for all KF-based methods). */
		void getMeanAndCov( vector_double &out_means, CMatrixDouble &out_cov) const;

	protected:
		/** Common options to all random-field grid maps: pointer that is set to the derived-class instance of "insertOptions" upon construction of this class. */
		TInsertionOptionsCommon * m_insertOptions_common;

		/** Get the part of the options common to all CRandomFieldGridMap2D classes */
		virtual CRandomFieldGridMap2D::TInsertionOptionsCommon* getCommonInsertOptions() = 0;

		/** The map representation type of this map, as passed in the constructor */
		TMapRepresentation	m_mapType;

		CMatrixD				m_cov;	//!< The whole covariance matrix, used for the Kalman Filter map representation.

		/** The compressed band diagonal matrix for the KF2 implementation.
		  *   The format is a Nx(W^2+2W+1) matrix, one row per cell in the grid map with the
		  *    cross-covariances between each cell and half of the window around it in the grid.
		  */
		CMatrixD		m_stackedCov;
		mutable bool	m_hasToRecoverMeanAndCov;       //!< Only for the KF2 implementation.

		/** @name Auxiliary vars for DM & DM+V methods
		    @{ */
		float               m_DM_lastCutOff;
		std::vector<float>	m_DM_gaussWindow;
		double				m_average_normreadings_mean, m_average_normreadings_var;
		size_t              m_average_normreadings_count;
		/** @} */

		/** The implementation of "insertObservation" for Achim Lilienthal's map models DM & DM+V.
		  * \param normReading Is a [0,1] normalized concentration reading.
		  * \param sensorPose Is the sensor pose on the robot
		  * \param is_DMV = false -> map type is Kernel DM; true -> map type is DM+V
		  */
		void  insertObservation_KernelDM_DMV(
			float            normReading,
			const CPose3D   &sensorPose,
			bool             is_DMV );

		/** The implementation of "insertObservation" for the (whole) Kalman Filter map model.
		  * \param normReading Is a [0,1] normalized concentration reading.
		  * \param sensorPose Is the sensor pose
		  */
		void  insertObservation_KF(
			float			normReading,
			const CPose3D	&sensorPose );

		/** The implementation of "insertObservation" for the Efficient Kalman Filter map model.
		  * \param normReading Is a [0,1] normalized concentration reading.
		  * \param sensorPose Is the sensor pose
		  */
		void  insertObservation_KF2(
			float			normReading,
			const CPose3D	&sensorPose );

		/** Computes the average cell concentration, or the overall average value if it has never been observed  */
		double computeMeanCellValue_DM_DMV (const TRandomFieldCell *cell ) const;

		/** Computes the estimated variance of the cell concentration, or the overall average variance if it has never been observed  */
		double computeVarCellValue_DM_DMV (const TRandomFieldCell *cell ) const;

		/** In the KF2 implementation, takes the auxiliary matrices and from them update the cells' mean and std values.
		  * \sa m_hasToRecoverMeanAndCov
		  */
		void  recoverMeanAndCov() const;

		/** Erase all the contents of the map */
		virtual void  internal_clear();

	};


	} // End of namespace


	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<slam::CRandomFieldGridMap2D::TMapRepresentation>
		{
			typedef slam::CRandomFieldGridMap2D::TMapRepresentation enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(slam::CRandomFieldGridMap2D::mrKernelDM,          "mrKernelDM");
				m_map.insert(slam::CRandomFieldGridMap2D::mrKalmanFilter,      "mrKalmanFilter");
				m_map.insert(slam::CRandomFieldGridMap2D::mrKalmanApproximate, "mrKalmanApproximate");
				m_map.insert(slam::CRandomFieldGridMap2D::mrKernelDMV,         "mrKernelDMV");
			}
		};
	} // End of namespace
} // End of namespace

#endif

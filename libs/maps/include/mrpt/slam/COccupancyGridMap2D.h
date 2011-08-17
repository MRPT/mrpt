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

#ifndef COccupancyGridMap2D_H
#define COccupancyGridMap2D_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CDynamicGrid.h>
#include <mrpt/slam/CMetricMap.h>
#include <mrpt/utils/TMatchingPair.h>
#include <mrpt/slam/CLogOddsGridMap2D.h>

#include <mrpt/maps/link_pragmas.h>

#include <mrpt/utils/safe_pointers.h>

#include <mrpt/config.h>

#if !defined(OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS) && !defined(OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS)
		#error One of OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS or OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS must be defined.
#endif

#if defined(OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS) && defined(OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS)
		#error Only one of OCCUPANCY_GRIDMAP_CELL_SIZE_16BITS or OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS must be defined at a time.
#endif


namespace mrpt
{
	namespace poses { class CPose2D; }
namespace slam
{
	using namespace mrpt::poses;
	using namespace mrpt::utils;

	class CObservation2DRangeScan;
	class CObservationRange;
	class CObservation;
	class CPointsMap;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( COccupancyGridMap2D, CMetricMap, MAPS_IMPEXP )

	/** A class for storing an occupancy grid map.
	 *  COccupancyGridMap2D is a class for storing a metric map
	 *   representation in the form of a probabilistic occupancy
	 *   grid map: value of 0 means certainly occupied, 1 means
	 *   a certainly empty cell. Initially 0.5 means uncertainty.
	 *
	 * The cells keep the log-odd representation of probabilities instead of the probabilities themselves.
	 *  More details can be found at http://www.mrpt.org/Occupancy_Grids
	 *
	 * The algorithm for updating the grid from a laser scanner can optionally take into account the progressive widening of the beams, as
	 *   described in the <a href="http://www.mrpt.org/Occupancy_Grids" > wiki </a> (this feature was introduced in MRPT 0.6.4).
	 *
	 *   Some implemented methods are:
	 *		- Update of individual cells
	 *		- Insertion of observations
	 *		- Voronoi diagram and critical points (\a buildVoronoiDiagram)
	 *		- Saving and loading from/to a bitmap
	 *		- Laser scans simulation for the map contents
	 *		- Entropy and information methods (See computeEntropy)
	 *
	  * \ingroup mrpt_maps_grp
	 **/
	class MAPS_IMPEXP COccupancyGridMap2D :
		public CMetricMap,
		// Inherit from the corresponding specialization of CLogOddsGridMap2D<>:
#ifdef	OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
		public CLogOddsGridMap2D<int8_t>
#else
		public CLogOddsGridMap2D<int16_t>
#endif
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( COccupancyGridMap2D )

	public:
	/** The type of the map cells: */
#ifdef	OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
		typedef int8_t  cellType;
		typedef uint8_t cellTypeUnsigned;
#else
		typedef int16_t  cellType;
		typedef uint16_t cellTypeUnsigned;
#endif

	/** Discrete to float conversion factors: The min/max values of the integer cell type, eg.[0,255] or [0,65535] */
	static const cellType OCCGRID_CELLTYPE_MIN  = CLogOddsGridMap2D<cellType>::CELLTYPE_MIN;
	static const cellType OCCGRID_CELLTYPE_MAX  = CLogOddsGridMap2D<cellType>::CELLTYPE_MAX;
	static const cellType OCCGRID_P2LTABLE_SIZE = CLogOddsGridMap2D<cellType>::P2LTABLE_SIZE;

	protected:

		friend class CMultiMetricMap;
		friend class CMultiMetricMapPDF;

		static CLogOddsGridMapLUT<cellType>  m_logodd_lut; //!< Lookup tables for log-odds

		/** This is the buffer for storing the cells.In this dynamic
		 *   size buffer are stored the cell values as
		 *   "bytes", stored row by row, from left to right cells.
		 */
		std::vector<cellType>    map;

		/** The size of the grid in cells.
		 */
		uint32_t		size_x,size_y;

		/** The limits of the grid in "units" (meters).
		 */
		float           x_min,x_max,y_min,y_max;

		/** Cell size, i.e. resolution of the grid map.
		 */
		float           resolution;

		/** These are auxiliary variables to speed up the computation of observation likelihood values for LF method among others, at a high cost in memory (see TLikelihoodOptions::enableLikelihoodCache).
		  */
		std::vector<double>		precomputedLikelihood;
		bool					precomputedLikelihoodToBeRecomputed;

		/** Used for Voronoi calculation.Same struct as "map", but contains a "0" if not a basis point. */
		CDynamicGrid<uint8_t>	m_basis_map;

		/** Used to store the Voronoi diagram.
		 *    Contains the distance of each cell to its closer obstacles
		 *    in 1/100th distance units (i.e. in centimeters), or 0 if not into the Voronoi diagram.
		 */
		CDynamicGrid<uint16_t>	m_voronoi_diagram;

		bool m_is_empty; //!< True upon construction; used by isEmpty()

		virtual void OnPostSuccesfulInsertObs(const CObservation *); //!< See base class

		/** The free-cells threshold used to compute the Voronoi diagram.
		 */
		float           voroni_free_threshold;

		/** Frees the dynamic memory buffers of map.
		 */
		void            freeMap();

		/** Entropy computation internal function:
		 */
		static double  H(double p);

		/** Change the contents [0,1] of a cell, given its index.
		 */
		inline void   setCell_nocheck(int x,int y,float value)
		{
				map[x+y*size_x]=p2l(value);
		}

		/** Read the real valued [0,1] contents of a cell, given its index.
		 */
		inline float  getCell_nocheck(int x,int y) const
		{
				return l2p(map[x+y*size_x]);
		}

		/** Changes a cell by its absolute index (Do not use it normally)
		  */
		inline void  setRawCell(unsigned int cellIndex, cellType b)
		{
			if (cellIndex<size_x*size_y)
			{
				map[cellIndex] = b;
			}
		}

		/** Internally used to speed-up entropy calculation
		  */
		static std::vector<float>		entropyTable;


		/** One of the methods that can be selected for implementing "computeObservationLikelihood" (This method is the Range-Scan Likelihood Consensus for gridmaps, see the ICRA2007 paper by Blanco et al.)
		  */
		double	 computeObservationLikelihood_Consensus(
					const CObservation		*obs,
					const CPose2D				&takenFrom );

		/** One of the methods that can be selected for implementing "computeObservationLikelihood"
		  *  TODO: This method is described in....
		  */
		double	 computeObservationLikelihood_ConsensusOWA(
					const CObservation		*obs,
					const CPose2D				&takenFrom );

		/** One of the methods that can be selected for implementing "computeObservationLikelihood".
		  */
		double	 computeObservationLikelihood_CellsDifference(
					const CObservation		*obs,
					const CPose2D				&takenFrom );

		/** One of the methods that can be selected for implementing "computeObservationLikelihood".
		  */
		double	 computeObservationLikelihood_MI(
					const CObservation		*obs,
					const CPose2D				&takenFrom );

		/** One of the methods that can be selected for implementing "computeObservationLikelihood".
		  */
		double	 computeObservationLikelihood_rayTracing(
					const CObservation		*obs,
					const CPose2D				&takenFrom );

		/** One of the methods that can be selected for implementing "computeObservationLikelihood".
		  */
		double	 computeObservationLikelihood_likelihoodField_Thrun(
					const CObservation		*obs,
					const CPose2D				&takenFrom );

		/** One of the methods that can be selected for implementing "computeObservationLikelihood".
		  */
		double	 computeObservationLikelihood_likelihoodField_II(
					const CObservation		*obs,
					const CPose2D				&takenFrom );

		/** Clear the map: It set all cells to their default occupancy value (0.5), without changing the resolution (the grid extension is reset to the default values).
		  */
		virtual void  internal_clear( );

		 /** Insert the observation information into this map.
		  *
		  * \param obs The observation
		  * \param robotPose The 3D pose of the robot mobile base in the map reference system, or NULL (default) if you want to use CPose2D(0,0,deg)
		  *
		  *  After successfull execution, "lastObservationInsertionInfo" is updated.
		  *
		  * \sa insertionOptions, CObservation::insertObservationInto
		  */
		 virtual bool  internal_insertObservation( const CObservation *obs, const CPose3D *robotPose = NULL );

	public:

		/** Performs the Bayesian fusion of a new observation of a cell.
		  * \sa updateInfoChangeOnly, updateCell_fast_occupied, updateCell_fast_free
		 */
		void  updateCell(int x,int y, float v);

		/** An internal structure for storing data related to counting the new information apported by some observation.
		  */
		struct MAPS_IMPEXP TUpdateCellsInfoChangeOnly
		{
			TUpdateCellsInfoChangeOnly( bool	enabled = false,
										double	I_change = 0,
										int		cellsUpdated=0) :	enabled(enabled),
																	I_change(I_change),
																	cellsUpdated(cellsUpdated),
																	laserRaysSkip(1)
			{
			}

			/** If set to false (default), this struct is not used. Set to true only when measuring the info of an observation.
			  */
			bool		enabled;

			/** The cummulative change in Information: This is updated only from the "updateCell" method.
			  */
			double		I_change;

			/** The cummulative updated cells count: This is updated only from the "updateCell" method.
			  */
			int			cellsUpdated;

			/** In this mode, some laser rays can be skips to speep-up
			  */
			int			laserRaysSkip;
		} updateInfoChangeOnly;

		/** Constructor.
		 */
		COccupancyGridMap2D( float min_x = -20.0f,
							 float max_x = 20.0f,
							 float min_y = -20.0f,
							 float max_y = 20.0f,
							 float resolution = 0.05f
							 );

		/** Fills all the cells with a default value.
		  */
		void  fill(float default_value = 0.5f );

		/** Destructor.
		 */
		virtual ~COccupancyGridMap2D();

		/** Change the size of gridmap, erasing all its previous contents.
		 * \param x_min The "x" coordinates of left most side of grid.
		 * \param x_max The "x" coordinates of right most side of grid.
		 * \param y_min The "y" coordinates of top most side of grid.
		 * \param y_max The "y" coordinates of bottom most side of grid.
		 * \param resolution The new size of cells.
		 * \param default_value The value of cells, tipically 0.5.
		 * \sa ResizeGrid
		 */
		void  setSize(float x_min,float x_max,float y_min,float y_max,float resolution,float default_value = 0.5f);

		/** Change the size of gridmap, maintaining previous contents.
		 * \param new_x_min The "x" coordinates of new left most side of grid.
		 * \param new_x_max The "x" coordinates of new right most side of grid.
		 * \param new_y_min The "y" coordinates of new top most side of grid.
		 * \param new_y_max The "y" coordinates of new bottom most side of grid.
		 * \param new_cells_default_value The value of the new cells, tipically 0.5.
		 * \param additionalMargin If set to true (default), an additional margin of a few meters will be added to the grid, ONLY if the new coordinates are larger than current ones.
		 * \sa setSize
		 */
		void  resizeGrid(float new_x_min,float new_x_max,float new_y_min,float new_y_max,float new_cells_default_value = 0.5f, bool additionalMargin = true) MRPT_NO_THROWS;

		/** Returns the area of the gridmap, in square meters */
		inline double getArea() const { return size_x*size_y*square(resolution); }

		/** Returns the horizontal size of grid map in cells count.
		 */
		inline unsigned int   getSizeX() const { return size_x; }

		/** Returns the vertical size of grid map in cells count.
		 */
		inline unsigned int   getSizeY() const { return size_y; }

		/** Returns the "x" coordinate of left side of grid map.
		 */
		inline float  getXMin() const { return x_min; }

		/** Returns the "x" coordinate of right side of grid map.
		 */
		inline float  getXMax() const { return x_max; }

		/** Returns the "y" coordinate of top side of grid map.
		 */
		inline float  getYMin() const { return y_min; }

		/** Returns the "y" coordinate of bottom side of grid map.
		 */
		inline float  getYMax() const { return y_max; }

		/** Returns the resolution of the grid map.
		 */
		inline float  getResolution() const { return resolution; }

		/** Transform a coordinate value into a cell index.
		 */
		inline int   x2idx(float x) const { return static_cast<int>((x-x_min)/resolution ); }
		inline int   y2idx(float y) const { return static_cast<int>((y-y_min)/resolution ); }

		inline int   x2idx(double x) const { return static_cast<int>((x-x_min)/resolution ); }
		inline int   y2idx(double y) const { return static_cast<int>((y-y_min)/resolution ); }

		/** Transform a cell index into a coordinate value.
		 */
		inline float   idx2x(const size_t cx) const { return x_min+(cx+0.5f)*resolution; }
		inline float   idx2y(const size_t cy) const { return y_min+(cy+0.5f)*resolution; }

		/** Transform a coordinate value into a cell index, using a diferent "x_min" value
		 */
		inline int   x2idx(float x,float x_min) const  { return static_cast<int>((x-x_min)/resolution ); }
		inline int   y2idx(float y, float y_min) const { return static_cast<int>((y-y_min)/resolution ); }

		/** Scales an integer representation of the log-odd into a real valued probability in [0,1], using p=exp(l)/(1+exp(l))
		  */
		static inline float l2p(const cellType  l)
		{
			return m_logodd_lut.l2p(l);
		}

		/** Scales an integer representation of the log-odd into a linear scale [0,255], using p=exp(l)/(1+exp(l))
		  */
		static inline uint8_t l2p_255(const cellType l)
		{
			return m_logodd_lut.l2p_255(l);
		}

		/** Scales a real valued probability in [0,1] to an integer representation of: log(p)-log(1-p)  in the valid range of cellType.
		  */
		static inline cellType p2l(const float p)
		{
			return m_logodd_lut.p2l(p);
		}

		/** Change the contents [0,1] of a cell, given its index.
		 */
		inline void   setCell(int x,int y,float value)
		{
			// The x> comparison implicitly holds if x<0
			if (static_cast<unsigned int>(x)>=size_x ||	static_cast<unsigned int>(y)>=size_y)
					return;
			else	map[x+y*size_x]=p2l(value);
		}

		/** Read the real valued [0,1] contents of a cell, given its index.
		 */
		inline float  getCell(int x,int y) const
		{
			// The x> comparison implicitly holds if x<0
			if (static_cast<unsigned int>(x)>=size_x ||	static_cast<unsigned int>(y)>=size_y)
					return 0.5f;
			else	return l2p(map[x+y*size_x]);
		}

		/** Access to a "row": mainly used for drawing grid as a bitmap efficiently, do not use it normally.
		  */
		inline  cellType *getRow( int cy ) { if (cy<0 || static_cast<unsigned int>(cy)>=size_y) return NULL; else return &map[0+cy*size_x]; }

		/** Access to a "row": mainly used for drawing grid as a bitmap efficiently, do not use it normally.
		  */
		inline  const cellType *getRow( int cy ) const { if (cy<0 || static_cast<unsigned int>(cy)>=size_y) return NULL; else return &map[0+cy*size_x]; }

		/** Change the contents [0,1] of a cell, given its coordinates.
		 */
		inline void   setPos(float x,float y,float value) { setCell(x2idx(x),y2idx(y),value); }

		/** Read the real valued [0,1] contents of a cell, given its coordinates.
		 */
		inline float  getPos(float x,float y) const { return getCell(x2idx(x),y2idx(y)); }

		/** Returns "true" if cell is "static", i.e.if its occupancy is below a given threshold.
		 */
		inline bool   isStaticPos(float x,float y,float threshold = 0.7f) const { return isStaticCell(x2idx(x),y2idx(y),threshold); }
		inline bool   isStaticCell(int cx,int cy,float threshold = 0.7f) const { return (getCell(cx,cy)<=threshold); }

		/** Change a cell in the "basis" maps.Used for Voronoi calculation.
		 */
		inline void   setBasisCell(int x,int y,uint8_t value)
		{
			uint8_t *cell=m_basis_map.cellByIndex(x,y);
#ifdef _DEBUG
			ASSERT_ABOVEEQ_(x,0)
			ASSERT_ABOVEEQ_(y,0)
			ASSERT_BELOWEQ_(x,int(m_basis_map.getSizeX()))
			ASSERT_BELOWEQ_(y,int(m_basis_map.getSizeY()))
#endif
			*cell = value;
		}

		/** Reads a cell in the "basis" maps.Used for Voronoi calculation.
		 */
		inline unsigned char  getBasisCell(int x,int y) const
		{
			const uint8_t *cell=m_basis_map.cellByIndex(x,y);
#ifdef _DEBUG
			ASSERT_ABOVEEQ_(x,0)
			ASSERT_ABOVEEQ_(y,0)
			ASSERT_BELOWEQ_(x,int(m_basis_map.getSizeX()))
			ASSERT_BELOWEQ_(y,int(m_basis_map.getSizeY()))
#endif
			return *cell;
		}

		/** Used for returning entropy related information
		 * \sa computeEntropy
		 */
		struct MAPS_IMPEXP TEntropyInfo
		{
			TEntropyInfo() : H(0),I(0),mean_H(0),mean_I(0),effectiveMappedArea(0),effectiveMappedCells(0)
			{
			}

			/** The target variable for absolute entropy, computed as:<br><center>H(map)=Sum<sub>x,y</sub>{ -p(x,y)·ln(p(x,y)) -(1-p(x,y))·ln(1-p(x,y)) }</center><br><br>
			  */
			double		H;

			/** The target variable for absolute "information", defining I(x) = 1 - H(x)
			  */
			double		I;

			/** The target variable for mean entropy, defined as entropy per cell: mean_H(map) = H(map) / (cells)
			  */
			double		mean_H;

			/** The target variable for mean information, defined as information per cell: mean_I(map) = I(map) / (cells)
			  */
			double		mean_I;

			/** The target variable for the area of cells with information, i.e. p(x)!=0.5
			  */
			double		effectiveMappedArea;

			/** The mapped area in cells.
			  */
			unsigned long effectiveMappedCells;
		};

		/** With this struct options are provided to the observation insertion process.
		* \sa CObservation::insertIntoGridMap
		*/
		class MAPS_IMPEXP TInsertionOptions : public mrpt::utils::CLoadableOptions
		{
		public:
			/** Initilization of default parameters
			*/
			TInsertionOptions( );

			/** This method load the options from a ".ini" file.
			 *   Only those parameters found in the given "section" and having
			 *   the same name that the variable are loaded. Those not found in
			 *   the file will stay with their previous values (usually the default
			 *   values loaded at initialization). An example of an ".ini" file:
			 *  \code
			 *  [section]
			 *	resolution=0.10		; blah blah...
			 *	modeSelection=1		; 0=blah, 1=blah,...
			 *  \endcode
			 */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase  &source,
				const std::string &section);

			/** This method must display clearly all the contents of the structure in textual form, sending it to a CStream.
			  */
			void  dumpToTextStream(CStream	&out) const;


			/** The altitude (z-axis) of 2D scans (within a 0.01m tolerance) for they to be inserted in this map!
			  */
			float	mapAltitude;

			/** The parameter "mapAltitude" has effect while inserting observations in the grid only if this is true.
			  */
			bool	useMapAltitude;

			/** The largest distance at which cells will be updated (Default 15 meters)
			*/
			float	maxDistanceInsertion;

			/** A value in the range [0.5,1] used for updating cell with a bayesian approach (default 0.8)
			*/
			float	maxOccupancyUpdateCertainty;

			/** If set to true (default), invalid range values (no echo rays) as consider as free space until "maxOccupancyUpdateCertainty", but ONLY when the previous and next rays are also an invalid ray.
			*/
			bool	considerInvalidRangesAsFreeSpace;

			/** Specify the decimation of the range scan (default=1 : take all the range values!)
			  */
			uint16_t	decimation;

			/** The tolerance in rads in pitch & roll for a laser scan to be considered horizontal, then processed by calls to this class (default=0). */
			float horizontalTolerance;

			/** Gaussian sigma of the filter used in getAsImageFiltered (for features detection) (Default=1) (0:Disabled) */
			float	CFD_features_gaussian_size;

			/** Size of the Median filter used in getAsImageFiltered (for features detection) (Default=3) (0:Disabled) */
			float	CFD_features_median_size;

			bool	wideningBeamsWithDistance;	//!< Enabled: Rays widen with distance to approximate the real behavior of lasers, disabled: insert rays as simple lines (Default=true)

		};

		/** With this struct options are provided to the observation insertion process.
		* \sa CObservation::insertIntoGridMap
		*/
		TInsertionOptions	insertionOptions;

		/** The type for selecting a likelihood computation method
		*/
		enum TLikelihoodMethod
		{
			lmMeanInformation = 0,
			lmRayTracing,
			lmConsensus,
			lmCellsDifference,
			lmLikelihoodField_Thrun,
			lmLikelihoodField_II,
			lmConsensusOWA
		};

		/** With this struct options are provided to the observation likelihood computation process.
		*/
		class MAPS_IMPEXP TLikelihoodOptions  : public mrpt::utils::CLoadableOptions
		{
		public:
			/** Initilization of default parameters
			*/
			TLikelihoodOptions();

			/** This method load the options from a ".ini" file.
			 *   Only those parameters found in the given "section" and having
			 *   the same name that the variable are loaded. Those not found in
			 *   the file will stay with their previous values (usually the default
			 *   values loaded at initialization). An example of an ".ini" file:
			 *  \code
			 *  [section]
			 *	resolution=0.10		; blah blah...
			 *	modeSelection=1		; 0=blah, 1=blah,...
			 *  \endcode
			 */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase  &source,
				const std::string &section);

			/** This method must display clearly all the contents of the structure in textual form, sending it to a CStream.
			  */
			void  dumpToTextStream(CStream	&out) const;

			/** The selected method to compute an observation likelihood.
			*/
			TLikelihoodMethod	likelihoodMethod;

			/** [LikelihoodField] The laser range "sigma" used in computations; Default value = 0.35
			*/
			float	LF_stdHit;

			/** [LikelihoodField] Ratios of the hit/random components of the likelihood; Default values=0.95/0.5
			*/
			float	LF_zHit, LF_zRandom;

			/** [LikelihoodField] The max. range of the sensor (def=81meters)
			*/
			float	LF_maxRange;

			/** [LikelihoodField] The decimation of the points in a scan, default=1 == no decimation.
			*/
			uint32_t LF_decimation;

			/** [LikelihoodField] The max. distance for searching correspondences around each sensed point
			*/
			float	LF_maxCorrsDistance;

			/** [LikelihoodField] Set this to "true" ot use an alternative method, where the likelihood of the whole range scan is computed by "averaging" of individual ranges, instead of by the "product".
			  */
			bool	LF_alternateAverageMethod;

			/** [MI] The exponent in the MI likelihood computation. Default value = 5
			*/
			float	MI_exponent;

			/** [MI] The scan rays decimation: at every N rays, one will be used to compute the MI:
			 */
			uint32_t MI_skip_rays;

			/** [MI] The ratio for the max. distance used in the MI computation and in the insertion of scans, e.g. if set to 2.0 the MI will use twice the distance that the update distance.
			  */
			float	MI_ratio_max_distance;

			/** [rayTracing] If true (default), the rayTracing method will ignore measured ranges shorter than the simulated ones.
			  */
			bool	rayTracing_useDistanceFilter;

			/** [rayTracing] One out of "rayTracing_decimation" rays will be simulated and compared only: set to 1 to use all the sensed ranges.
			  */
			int32_t  rayTracing_decimation;

			/** [rayTracing] The laser range sigma.
			  */
			float	rayTracing_stdHit;

			/** [Consensus] The down-sample ratio of ranges (default=1, consider all the ranges)
			  */
			int32_t	consensus_takeEachRange;

			/** [Consensus] The power factor for the likelihood (default=5)
			  */
			float	consensus_pow;

			/** [OWA] The sequence of weights to be multiplied to of the ordered list of likelihood values (first one is the largest); the size of this vector determines the number of highest likelihood values to fuse.
			  */
			std::vector<float>	OWA_weights;

			/** Enables the usage of a cache of likelihood values (for LF methods), if set to true (default=false).
			  */
			bool    enableLikelihoodCache;

		} likelihoodOptions;

		/** Auxiliary private class.
		  */
		typedef std::pair<double,CPoint2D> TPairLikelihoodIndex;

		/** Some members of this struct will contain intermediate or output data after calling "computeObservationLikelihood" for some likelihood functions.
		  */
		class TLikelihoodOutput
		{
		public:
			TLikelihoodOutput() : OWA_pairList(), OWA_individualLikValues()
			{}

			/** [OWA method] This will contain the ascending-ordered list of pairs:(likelihood values, 2D point in map coordinates).
			  */
			std::vector<TPairLikelihoodIndex>	OWA_pairList;

			/** [OWA method] This will contain the ascending-ordered list of likelihood values for individual range measurements in the scan.
			  */
			std::vector<double>	OWA_individualLikValues;

		} likelihoodOutputs;

		 /** Performs a downsampling of the gridmap, by a given factor: resolution/=ratio
		  */
		 void  subSample( int downRatio );

		/** Computes the entropy and related values of this grid map.
		 *  The entropy is computed as the summed entropy of each cell, taking them as discrete random variables following a Bernoulli distribution:
		 * \param info The output information is returned here.
		 */
		void  computeEntropy( TEntropyInfo &info ) const;

		/** @name Voronoi methods
		    @{ */

		/** Build the Voronoi diagram of the grid map.
		 * \param threshold The threshold for binarizing the map.
		 * \param robot_size Size in "units" (meters) of robot, approx.
		 * \param x1 Left coordinate of area to be computed. Default, entire map.
		 * \param x2 Right coordinate of area to be computed. Default, entire map.
		 * \param y1 Top coordinate of area to be computed. Default, entire map.
		 * \param y2 Bottom coordinate of area to be computed. Default, entire map.
		 * \sa findCriticalPoints
		 */
		void  buildVoronoiDiagram(float threshold, float robot_size,int x1=0,int x2=0, int y1=0,int y2=0);

		/** Reads a the clearance of a cell (in centimeters), after building the Voronoi diagram with \a buildVoronoiDiagram */
		inline uint16_t getVoroniClearance(int cx,int cy) const
		{
#ifdef _DEBUG
			ASSERT_ABOVEEQ_(cx,0)
			ASSERT_ABOVEEQ_(cy,0)
			ASSERT_BELOWEQ_(cx,int(m_voronoi_diagram.getSizeX()))
			ASSERT_BELOWEQ_(cy,int(m_voronoi_diagram.getSizeY()))
#endif
			const uint16_t *cell=m_voronoi_diagram.cellByIndex(cx,cy);
			return *cell;
		}

	protected:
		/** Used to set the clearance of a cell, while building the Voronoi diagram. */
		inline void setVoroniClearance(int cx,int cy,uint16_t dist)
		{
			uint16_t *cell=m_voronoi_diagram.cellByIndex(cx,cy);
#ifdef _DEBUG
			ASSERT_ABOVEEQ_(cx,0)
			ASSERT_ABOVEEQ_(cy,0)
			ASSERT_BELOWEQ_(cx,int(m_voronoi_diagram.getSizeX()))
			ASSERT_BELOWEQ_(cy,int(m_voronoi_diagram.getSizeY()))
#endif
			*cell = dist;
		}

	public:

		/** Return the auxiliary "basis" map built while building the Voronoi diagram \sa buildVoronoiDiagram */
		inline const CDynamicGrid<uint8_t>	& getBasisMap() const { return m_basis_map; }

		/** Return the Voronoi diagram; each cell contains the distance to its closer obstacle, or 0 if not part of the Voronoi diagram \sa buildVoronoiDiagram */
		inline const CDynamicGrid<uint16_t>	& getVoronoiDiagram() const { return m_voronoi_diagram; }

		/** Builds a list with the critical points from Voronoi diagram, which must
		 *    must be built before calling this method.
		 * \param filter_distance The minimum distance between two critical points.
		 * \sa buildVoronoiDiagram
		 */
		void  findCriticalPoints( float filter_distance );

		/** @} */ // End of Voronoi methods


		/** Compute the clearance of a given cell, and returns its two first
		 *   basis (closest obstacle) points.Used to build Voronoi and critical points.
		 * \return The clearance of the cell, in 1/100 of "cell".
		 * \param cx The cell index
		 * \param cy The cell index
		 * \param basis_x Target buffer for coordinates of basis, having a size of two "ints".
		 * \param basis_y Target buffer for coordinates of basis, having a size of two "ints".
		 * \param nBasis The number of found basis: Can be 0,1 or 2.
		 * \param GetContourPoint If "true" the basis are not returned, but the closest free cells.Default at false.
		 * \sa Build_VoronoiDiagram
		 */
		int  computeClearance( int cx, int cy, int *basis_x, int *basis_y, int *nBasis, bool GetContourPoint = false ) const;

		/** An alternative method for computing the clearance of a given location (in meters).
		  *  \return The clearance (distance to closest OCCUPIED cell), in meters.
		  */
		float  computeClearance( float x, float y, float maxSearchDistance ) const;

		/** Compute the 'cost' of traversing a segment of the map according to the occupancy of traversed cells.
		  *  \return This returns '1-mean(traversed cells occupancy)', i.e. 0.5 for unknown cells, 1 for a free path.
		  */
		float  computePathCost( float x1, float y1, float x2, float y2 ) const;


		/** \name Sensor simulators
		    @{
		   */

		/** Simulates a laser range scan into the current grid map.
		 *   The simulated scan is stored in a CObservation2DRangeScan object, which is also used
		 *    to pass some parameters: all previously stored characteristics (as aperture,...) are
		 *	  taken into account for simulation. Only a few more parameters are needed. Additive gaussian noise can be optionally added to the simulated scan.
		 * \param inout_Scan [IN/OUT] This must be filled with desired parameters before calling, and will contain the scan samples on return.
		 * \param robotPose [IN] The robot pose in this map coordinates. Recall that sensor pose relative to this robot pose must be specified in the observation object.
		 * \param threshold [IN] The minimum occupancy threshold to consider a cell to be occupied, for example 0.5.
		 * \param N [IN] The count of range scan "rays", by default to 361.
		 * \param noiseStd [IN] The standard deviation of measurement noise. If not desired, set to 0.
		 * \param decimation [IN] The rays that will be simulated are at indexes: 0, D, 2D, 3D, ... Default is D=1
		 * \param angleNoiseStd [IN] The sigma of an optional Gaussian noise added to the angles at which ranges are measured (in radians).
		 *
		 * \sa sonarSimulator
		 */
		void  laserScanSimulator(
				CObservation2DRangeScan	        &inout_Scan,
				const CPose2D					&robotPose,
				float						    threshold = 0.5f,
				size_t						    N = 361,
				float						    noiseStd = 0,
				unsigned int				    decimation = 1,
				float							angleNoiseStd = DEG2RAD(0) ) const;

		/** Simulates the observations of a sonar rig into the current grid map.
		 *   The simulated ranges are stored in a CObservationRange object, which is also used
		 *    to pass in some needed parameters, as the poses of the sonar sensors onto the mobile robot.
		 * \param inout_observation [IN/OUT] This must be filled with desired parameters before calling, and will contain the simulated ranges on return.
		 * \param robotPose [IN] The robot pose in this map coordinates. Recall that sensor pose relative to this robot pose must be specified in the observation object.
		 * \param threshold [IN] The minimum occupancy threshold to consider a cell to be occupied, for example 0.5.
		 * \param rangeNoiseStd [IN] The standard deviation of measurement noise. If not desired, set to 0.
		 * \param angleNoiseStd [IN] The sigma of an optional Gaussian noise added to the angles at which ranges are measured (in radians).
		 *
		 * \sa laserScanSimulator
		 */
		void  sonarSimulator(
				CObservationRange	        &inout_observation,
				const CPose2D				&robotPose,
				float						threshold = 0.5f,
				float						rangeNoiseStd = 0,
				float						angleNoiseStd = DEG2RAD(0) ) const;

		/** Simulate just one "ray" in the grid map. This method is used internally to sonarSimulator and laserScanSimulator.
		  */
		inline void simulateScanRay(
			const double x,const double y,const double angle_direction,
			float &out_range,bool &out_valid,
			const unsigned int max_ray_len,
			const float threshold_free=0.5f,
			const double noiseStd=0, const double angleNoiseStd=0 ) const;


		/** @} */

		/** Computes the likelihood that a given observation was taken from a given pose in the world being modeled with this map.
		 *  See "likelihoodOptions" for configuration parameters.
		 *
		 * \param takenFrom The robot's pose the observation is supposed to be taken from.
		 * \param obs The observation.
		 * \return This method returns a likelihood in the range [0,1].
		 *
		 * Used in particle filter algorithms, see: CMultiMetricMapPDF::prediction_and_update
		 *
		 * \sa likelihoodOptions, likelihoodOutputs
		 */
		double	 computeObservationLikelihood( const CObservation *obs, const CPose3D &takenFrom );

		/** Returns true if this map is able to compute a sensible likelihood function for this observation (i.e. an occupancy grid map cannot with an image).
		 * \param obs The observation.
		 * \sa computeObservationLikelihood
		 */
		bool canComputeObservationLikelihood( const CObservation *obs );

		/** Computes the likelihood [0,1] of a set of points, given the current grid map as reference.
		  * \param pm The points map
		  * \param relativePose The relative pose of the points map in this map's coordinates, or NULL for (0,0,0).
		  *  See "likelihoodOptions" for configuration parameters.
		  */
		double	 computeLikelihoodField_Thrun( const CPointsMap	*pm, const CPose2D *relativePose = NULL);

		/** Computes the likelihood [0,1] of a set of points, given the current grid map as reference.
		  * \param pm The points map
		  * \param relativePose The relative pose of the points map in this map's coordinates, or NULL for (0,0,0).
		  *  See "likelihoodOptions" for configuration parameters.
		  */
		double	 computeLikelihoodField_II( const CPointsMap	*pm, const CPose2D *relativePose = NULL);

		/** Saves the gridmap as a graphical file (BMP,PNG,...).
		 * The format will be derived from the file extension (see  CImage::saveToFile )
		 * \return False on any error.
		 */
		bool  saveAsBitmapFile(const std::string &file) const;

		/** Saves a composite image with two gridmaps and lines representing a set of correspondences between them.
		 * \sa saveAsEMFTwoMapsWithCorrespondences
		 * \return False on any error.
		 */
		static bool  saveAsBitmapTwoMapsWithCorrespondences(
			const std::string						&fileName,
			const COccupancyGridMap2D				*m1,
			const COccupancyGridMap2D				*m2,
			const TMatchingPairList		&corrs);

		/** Saves a composite image with two gridmaps and numbers for the correspondences between them.
		 * \sa saveAsBitmapTwoMapsWithCorrespondences
		 * \return False on any error.
		 */
		static bool  saveAsEMFTwoMapsWithCorrespondences(
			const std::string						&fileName,
			const COccupancyGridMap2D				*m1,
			const COccupancyGridMap2D				*m2,
			const TMatchingPairList		&corrs);

		/** Saves the gridmap as a graphical bitmap file, 8 bit gray scale, 1 pixel is 1 cell, and with an overlay of landmarks.
		 * \note The template parameter CLANDMARKSMAP is assumed to be mrpt::slam::CLandmarksMap normally.
		 * \return False on any error.
		 */
		template <class CLANDMARKSMAP>
		bool  saveAsBitmapFileWithLandmarks(
			const std::string	&file,
			const CLANDMARKSMAP *landmarks,
			bool  addTextLabels = false,
			const mrpt::utils::TColor &marks_color = mrpt::utils::TColor(0,0,255) ) const
		{
			MRPT_START
			CImage		img(1,1,3);
			getAsImageFiltered( img, false,  true ); // in RGB
			const bool topleft = img.isOriginTopLeft();
			for (unsigned int i=0;i<landmarks->landmarks.size();i++)
			{
				const typename CLANDMARKSMAP::landmark_type *lm = landmarks->landmarks.get( i );
				int		px = x2idx( lm->pose_mean.x );
				int		py = topleft ?  size_y-1- y2idx( lm->pose_mean.y ) : y2idx( lm->pose_mean.y );
				img.rectangle(  px - 7, (py + 7), px +7, (py -7), marks_color );
				img.rectangle(  px - 6, (py + 6), px +6, (py -6), marks_color );
				if (addTextLabels)
					img.textOut(px,py-8,format("%u",i), TColor::black);
			}
			return img.saveToFile(file.c_str() );
			MRPT_END
		}


		/** Returns the grid as a 8-bit graylevel image, where each pixel is a cell (output image is RGB only if forceRGB is true)
		  *  If "tricolor" is true, only three gray levels will appear in the image: gray for unobserved cells, and black/white for occupied/empty cells respectively.
		  * \sa getAsImageFiltered
		  */
		void  getAsImage( utils::CImage	&img, bool verticalFlip = false, bool forceRGB=false, bool tricolor = false) const;

		/** Returns the grid as a 8-bit graylevel image, where each pixel is a cell (output image is RGB only if forceRGB is true) - This method filters the image for easy feature detection
		  *  If "tricolor" is true, only three gray levels will appear in the image: gray for unobserved cells, and black/white for occupied/empty cells respectively.
		  * \sa getAsImage
		  */
		void  getAsImageFiltered( utils::CImage	&img, bool verticalFlip = false, bool forceRGB=false) const;

		/** Returns a 3D plane with its texture being the occupancy grid and transparency proportional to "uncertainty" (i.e. a value of 0.5 is fully transparent)
		  */
		void  getAs3DObject ( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const;

		/** Returns true upon map construction or after calling clear(), the return
		  *  changes to false upon successful insertObservation() or any other method to load data in the map.
		  */
		bool  isEmpty() const;

		/** Load the gridmap from a image in a file (the format can be any supported by CImage::loadFromFile).
		 * \param file The file to be loaded.
		 * \param resolution The size of a pixel (cell), in meters. Recall cells are always squared, so just a dimension is needed.
		 * \param xCentralPixel The "x" coordinate (0=first) for the pixel which will be taken at coordinates origin (0,0). If not supplied, it will be used the middle of the map.
		 * \param yCentralPixel The "y" coordinate (0=first) for the pixel which will be taken at coordinates origin (0,0). If not supplied, it will be used the middle of the map.
		 * \return False on any error.
		 * \sa loadFromBitmap
		 */
		bool  loadFromBitmapFile(const std::string	&file, float resolution, float xCentralPixel = -1, float yCentralPixel =-1 );

		/** Load the gridmap from a image in a file (the format can be any supported by CImage::loadFromFile).
		 * \param img The image. Only a grayscale image will be used, so RGB components will be mixed if a color image is passed.
		 * \param resolution The size of a pixel (cell), in meters. Recall cells are always squared, so just a dimension is needed.
		 * \param xCentralPixel The "x" coordinate (0=first) for the pixel which will be taken at coordinates origin (0,0). If not supplied, it will be used the middle of the map.
		 * \param yCentralPixel The "y" coordinate (0=first) for the pixel which will be taken at coordinates origin (0,0). If not supplied, it will be used the middle of the map.
		 * \return False on any error.
		 * \sa loadFromBitmapFile
		 */
		bool  loadFromBitmap(const mrpt::utils::CImage &img, float resolution, float xCentralPixel = -1, float yCentralPixel =-1 );

		/** See the base class for more details: In this class it is implemented as correspondences of the passed points map to occupied cells.
		 * NOTICE: That the "z" dimension is ignored in the points. Clip the points as appropiated if needed before calling this method.
		 *
		 * \sa computeMatching3DWith
		 */
		void  computeMatchingWith2D(
				const CMetricMap     *otherMap,
				const CPose2D        &otherMapPose,
				float                maxDistForCorrespondence,
				float                maxAngularDistForCorrespondence,
				const CPose2D        &angularDistPivotPoint,
				TMatchingPairList    &correspondences,
				float                &correspondencesRatio,
				float                *sumSqrDist	= NULL,
				bool                  onlyKeepTheClosest = false,
				bool                  onlyUniqueRobust = false,
				const size_t          decimation_other_map_points = 1,
				const size_t          offset_other_map_points = 0 ) const;


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

		/** This virtual method saves the map to a file "filNamePrefix"+< some_file_extension >, as an image or in any other applicable way (Notice that other methods to save the map may be implemented in classes implementing this virtual interface).
		  */
		void  saveMetricMapRepresentationToFile(
			const std::string	&filNamePrefix
			) const;

		/** The structure used to store the set of Voronoi diagram
		 *    critical points.
		 * \sa findCriticalPoints
		 */
		struct MAPS_IMPEXP TCriticalPointsList
		{
			TCriticalPointsList() : x(),y(),clearance(),x_basis1(),y_basis1(),x_basis2(),y_basis2()
			{}

			/** The coordinates of critical point.
			 */
			std::vector<int>       x,y;
			/** The clearance of critical points, in 1/100 of cells.
			 */
			std::vector<int>       clearance;
			/** Their two first basis points coordinates.
			 */
			std::vector<int>       x_basis1,y_basis1, x_basis2,y_basis2;
		} CriticalPointsList;


	private:
		/** Returns a byte with the occupancy of the 8 sorrounding cells.
		 * \param cx The cell index
		 * \param cy The cell index
		 * \sa direction2idx
		 */
		inline unsigned char  GetNeighborhood( int cx, int cy ) const;

		/** Used to store the 8 possible movements from a cell to the
		 *   sorrounding ones.Filled in the constructor.
		 * \sa direction2idx
		 */
		int     direccion_vecino_x[8],direccion_vecino_y[8];

		/** Returns the index [0,7] of the given movement, or
		 *  -1 if invalid one.
		 * \sa direccion_vecino_x,direccion_vecino_y,GetNeighborhood
		 */
		int  direction2idx(int dx, int dy);
	};


	bool operator <(const COccupancyGridMap2D::TPairLikelihoodIndex &e1, const COccupancyGridMap2D::TPairLikelihoodIndex &e2);

	} // End of namespace
} // End of namespace

#endif

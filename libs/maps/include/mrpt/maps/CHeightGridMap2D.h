/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef CHeightGridMap2D_H
#define CHeightGridMap2D_H

#include <mrpt/maps/CHeightGridMap2D_Base.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/utils/CDynamicGrid.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/color_maps.h>
#include <mrpt/utils/TEnumType.h>
#include <mrpt/maps/link_pragmas.h>
#include <mrpt/poses/poses_frwds.h>
#include <mrpt/maps/link_pragmas.h>
#include <mrpt/obs/obs_frwds.h>

namespace mrpt
{
	namespace maps
	{

		/** The contents of each cell in a CHeightGridMap2D map */
		struct MAPS_IMPEXP THeightGridmapCell
		{
			/** The current average height (in meters) */
			float     h;    
			/** The current standard deviation of the height (in meters) */
			float     var;  
			/** Auxiliary variable for storing the incremental mean value (in meters). */
			float     u;    
			/** Auxiliary (in meters) */
			float     v;    
			/** [For mrSimpleAverage model] The accumulated weight: initially zero if un-observed, increased by one for each observation */
			uint32_t  w;    

			THeightGridmapCell() : h(),var(),u(),v(),w() {}
		};

		/** Digital Elevation Model (DEM), a mesh or grid representation of a surface which keeps the estimated height for each (x,y) location.
		  *  Important implemented features are the insertion of 2D laser scans (from arbitrary 6D poses) and the exportation as 3D scenes.
		  *
		  * Each cell contains the up-to-date average height from measured falling in that cell. Algorithms that can be used:
		  *   - mrSimpleAverage: Each cell only stores the current average value.
		  *
		  *  This class implements generic version of mrpt::maps::CMetric::insertObservation() accepting these types of sensory data:
		  *   - mrpt::obs::CObservation2DRangeScan: 2D range scans
		  *   - mrpt::obs::CObservationVelodyneScan
		  *
		  * \ingroup mrpt_maps_grp
		  */
		class MAPS_IMPEXP CHeightGridMap2D :
			public mrpt::maps::CMetricMap,
			public utils::CDynamicGrid<THeightGridmapCell>,
			public CHeightGridMap2D_Base
		{
			DEFINE_SERIALIZABLE( CHeightGridMap2D )
		public:

			/** Calls the base CMetricMap::clear
			  * Declared here to avoid ambiguity between the two clear() in both base classes.
			  */
			inline void clear() { CMetricMap::clear(); }

			float cell2float(const THeightGridmapCell& c) const override {
				return float(c.h);
			}

			/** The type of map representation to be used.
			  *  See mrpt::maps::CHeightGridMap2D for discussion.
			  */
			enum TMapRepresentation
			{
				mrSimpleAverage = 0
			};

			/** Constructor */
			CHeightGridMap2D(
				TMapRepresentation	mapType = mrSimpleAverage,
				double x_min = -2, double x_max = 2,
				double y_min = -2, double y_max = 2,
				double resolution = 0.1
				);

			 /** Returns true if the map is empty/no observation has been inserted. */
			 bool isEmpty() const override; 

			/** Parameters related with inserting observations into the map */
			struct MAPS_IMPEXP TInsertionOptions : public utils::CLoadableOptions
			{
				/** Default values loader */
				TInsertionOptions(); 

				void   loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) override; // See base docs
				void   dumpToTextStream(mrpt::utils::CStream &out) const override; // See base docs

				/** Whether to perform filtering by z-coordinate (default=false): coordinates are always RELATIVE to the robot for this filter.vvv */
				bool   filterByHeight; 
				/** Only when filterByHeight is true: coordinates are always RELATIVE to the robot for this filter. */
				float  z_min,z_max; 

				mrpt::utils::TColormap colorMap;
			} insertionOptions;

			/** See docs in base class: in this class it always returns 0 */
			float compute3DMatchingRatio(const mrpt::maps::CMetricMap *otherMap, const mrpt::poses::CPose3D &otherMapPose, const TMatchingRatioParams &params) const override;

			void  saveMetricMapRepresentationToFile(const std::string &filNamePrefix) const override; // See base class docs

			/** Returns a 3D object representing the map: by default, it will be a mrpt::opengl::CMesh object, unless
			  *   it is specified otherwise in mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH */
			void getAs3DObject(mrpt::opengl::CSetOfObjects::Ptr &outObj) const override;

			/** Return the type of the gas distribution map, according to parameters passed on construction */
			TMapRepresentation	 getMapType();

			/** Return the number of cells with at least one height data inserted. */
			size_t countObservedCells() const;

			virtual bool insertIndividualPoint(const double x,const double y,const double z, const CHeightGridMap2D_Base::TPointInsertParams & params = CHeightGridMap2D_Base::TPointInsertParams() ) override;
			virtual double dem_get_resolution() const  override;
			virtual size_t dem_get_size_x() const  override;
			virtual size_t dem_get_size_y() const  override;
			virtual bool   dem_get_z_by_cell(const size_t cx, const size_t cy, double &z_out) const  override;
			virtual bool   dem_get_z(const double x, const double y, double &z_out) const  override;
			virtual void   dem_update_map() override;

			/** The map representation type of this map */
			TMapRepresentation  m_mapType;  

			// See docs in base class
			void  internal_clear() override;
			bool  internal_insertObservation( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D *robotPose = nullptr ) override;
			double internal_computeObservationLikelihood( const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D &takenFrom ) override;

			MAP_DEFINITION_START(CHeightGridMap2D,MAPS_IMPEXP)
				/** See CHeightGridMap2D::CHeightGridMap2D */
				double min_x,max_x,min_y,max_y,resolution;	
				/** The kind of map representation (see CHeightGridMap2D::CHeightGridMap2D) */
				mrpt::maps::CHeightGridMap2D::TMapRepresentation	mapType;	
				mrpt::maps::CHeightGridMap2D::TInsertionOptions	insertionOpts;
			MAP_DEFINITION_END(CHeightGridMap2D,MAPS_IMPEXP)
		};
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CHeightGridMap2D, CMetricMap, MAPS_IMPEXP  )

	} // End of namespace

	namespace global_settings
	{
		/** If set to true (default), mrpt::maps::CHeightGridMap2D will be exported as a opengl::CMesh, otherwise, as a opengl::CPointCloudColoured
		  * Affects to:
		  *		- CHeightGridMap2D::getAs3DObject
		  */
		extern MAPS_IMPEXP bool HEIGHTGRIDMAP_EXPORT3D_AS_MESH;
	}

	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<maps::CHeightGridMap2D::TMapRepresentation>
		{
			typedef maps::CHeightGridMap2D::TMapRepresentation enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(maps::CHeightGridMap2D::mrSimpleAverage,     "mrSimpleAverage");
			}
		};
	} // End of namespace

} // End of namespace

#endif

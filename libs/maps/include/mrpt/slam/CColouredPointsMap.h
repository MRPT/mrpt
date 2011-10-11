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
#ifndef CColouredPointsMap_H
#define CColouredPointsMap_H

#include <mrpt/slam/CPointsMap.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CObservationImage.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/utils/stl_extensions.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		class CObservation3DRangeScan;


		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CColouredPointsMap, CPointsMap,MAPS_IMPEXP )

		/** A map of 2D/3D points with individual colours (RGB).
		 *  For different color schemes, see CColouredPointsMap::colorScheme
		 *  Colors are defined in the range [0,1].
		 * \sa mrpt::slam::CPointsMap, mrpt::slam::CMetricMap, mrpt::utils::CSerializable
	  	 * \ingroup mrpt_maps_grp
		 */
		class MAPS_IMPEXP CColouredPointsMap : public CPointsMap
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CColouredPointsMap )

		public:
			 /** Destructor
			   */
			 virtual ~CColouredPointsMap();

			 /** Default constructor
			  */
			 CColouredPointsMap();

			// --------------------------------------------
			/** @name Pure virtual interfaces to be implemented by any class derived from CPointsMap
				@{ */

			/** Reserves memory for a given number of points: the size of the map does not change, it only reserves the memory.
			  *  This is useful for situations where it is approximately known the final size of the map. This method is more
			  *  efficient than constantly increasing the size of the buffers. Refer to the STL C++ library's "reserve" methods.
			  */
			virtual void reserve(size_t newLength);

			/** Resizes all point buffers so they can hold the given number of points: newly created points are set to default values,
			  *  and old contents are not changed.
			  * \sa reserve, setPoint, setPointFast, setSize
			  */
			virtual void resize(size_t newLength);

			/** Resizes all point buffers so they can hold the given number of points, *erasing* all previous contents
			  *  and leaving all points to default values.
			  * \sa reserve, setPoint, setPointFast, setSize
			  */
			virtual void setSize(size_t newLength);

			/** Changes the coordinates of the given point (0-based index), *without* checking for out-of-bounds and *without* calling mark_as_modified()  \sa setPoint */
			virtual void  setPointFast(size_t index,float x, float y, float z)
			{
				this->x[index] = x;
				this->y[index] = y;
				this->z[index] = z;
			}

			/** The virtual method for \a insertPoint() *without* calling mark_as_modified()   */
			virtual void  insertPointFast( float x, float y, float z = 0 );

			 /** Virtual assignment operator, to be implemented in derived classes.
			   */
			 virtual void  copyFrom(const CPointsMap &obj);

			/** Get all the data fields for one point as a vector: [X Y Z R G B]
			  *  Unlike getPointAllFields(), this method does not check for index out of bounds
			  * \sa getPointAllFields, setPointAllFields, setPointAllFieldsFast
			  */
			virtual void  getPointAllFieldsFast( const size_t index, std::vector<float> & point_data ) const {
				point_data.resize(6);
				point_data[0] = x[index];
				point_data[1] = y[index];
				point_data[2] = z[index];
				point_data[3] = m_color_R[index];
				point_data[4] = m_color_G[index];
				point_data[5] = m_color_B[index];
			}

			/** Set all the data fields for one point as a vector: [X Y Z R G B]
			  *  Unlike setPointAllFields(), this method does not check for index out of bounds
			  * \sa setPointAllFields, getPointAllFields, getPointAllFieldsFast
			  */
			virtual void  setPointAllFieldsFast( const size_t index, const std::vector<float> & point_data ) {
				ASSERTDEB_(point_data.size()==6)
				x[index] = point_data[0];
				y[index] = point_data[1];
				z[index] = point_data[2];
				m_color_R[index] = point_data[3];
				m_color_G[index] = point_data[4];
				m_color_B[index] = point_data[5];
			}

			/** See CPointsMap::loadFromRangeScan() */
			virtual void  loadFromRangeScan(
					const CObservation2DRangeScan &rangeScan,
					const CPose3D				  *robotPose = NULL );

			/** See CPointsMap::loadFromRangeScan() */
			virtual void  loadFromRangeScan(
					const CObservation3DRangeScan &rangeScan,
					const CPose3D				  *robotPose = NULL );

		protected:

			/** Auxiliary method called from within \a addFrom() automatically, to finish the copying of class-specific data  */
			virtual void  addFrom_classSpecific(const CPointsMap &anotherMap, const size_t nPreviousPoints);


			// Friend methods:
			template <class Derived> friend struct detail::loadFromRangeImpl;
			template <class Derived> friend struct detail::pointmap_traits;

		public:


			/** @} */
			// --------------------------------------------

			/** Save to a text file. In each line contains X Y Z (meters) R G B (range [0,1]) for each point in the map.
			 *     Returns false if any error occured, true elsewere.
			 */
			bool  save3D_and_colour_to_text_file(const std::string &file) const;

			/** Changes a given point from map. First index is 0.
			 * \exception Throws std::exception on index out of bound.
			 */
			virtual void  setPoint(size_t index,float x, float y, float z, float R, float G, float B);

			// The following overloads must be repeated here (from CPointsMap) due to the shadowing of the above "setPoint()"
			/// \overload
			inline void  setPoint(size_t index,float x, float y, float z) {
				ASSERT_BELOW_(index,this->size())
				setPointFast(index,x,y,z);
				mark_as_modified();
			}
			/// \overload
			inline void  setPoint(size_t index,CPoint2D &p) {  setPoint(index,p.x(),p.y(),0); }
			/// \overload
			inline void  setPoint(size_t index,CPoint3D &p)  { setPoint(index,p.x(),p.y(),p.z()); }
			/// \overload
			inline void  setPoint(size_t index,float x, float y) { setPoint(index,x,y,0); }


			/** Adds a new point given its coordinates and color (colors range is [0,1]) */
			virtual void  insertPoint( float x, float y, float z, float R, float G, float B );
			// The following overloads must be repeated here (from CPointsMap) due to the shadowing of the above "insertPoint()"
			/// \overload of \a insertPoint()
			inline void  insertPoint( const CPoint3D &p ) { insertPoint(p.x(),p.y(),p.z()); }
			/// \overload
			inline void  insertPoint( const mrpt::math::TPoint3D &p ) { insertPoint(p.x,p.y,p.z); }
			/// \overload
			inline void  insertPoint( float x, float y, float z) { insertPointFast(x,y,z); mark_as_modified(); }

			/** Changes just the color of a given point from the map. First index is 0.
			 * \exception Throws std::exception on index out of bound.
			 */
			void  setPointColor(size_t index,float R, float G, float B);

			/** Like \c setPointColor but without checking for out-of-index erors */
			inline void  setPointColor_fast(size_t index,float R, float G, float B)
			{
				this->m_color_R[index]=R;
				this->m_color_G[index]=G;
				this->m_color_B[index]=B;
			}

			/** Retrieves a point and its color (colors range is [0,1])
			  */
			virtual void  getPoint( size_t index, float &x, float &y, float &z, float &R, float &G, float &B ) const;

			/** Retrieves a point  */
			unsigned long  getPoint( size_t index, float &x, float &y, float &z) const;

			/** Retrieves a point color (colors range is [0,1]) */
			void  getPointColor( size_t index, float &R, float &G, float &B ) const;

			/** Like \c getPointColor but without checking for out-of-index erors */
			inline void  getPointColor_fast( size_t index, float &R, float &G, float &B ) const
			{
				R = m_color_R[index];
				G = m_color_G[index];
				B = m_color_B[index];
			}

			/** Returns true if the point map has a color field for each point */
			virtual bool hasColorPoints() const { return true; }

			/** Override of the default 3D scene builder to account for the individual points' color.
			  * \sa mrpt::global_settings::POINTSMAPS_3DOBJECT_POINTSIZE
			  */
			virtual void  getAs3DObject ( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const;

			/** Colour a set of points from a CObservationImage and the global pose of the robot
			  */
			bool colourFromObservation( const CObservationImage &obs, const CPose3D &robotPose );

			/** The choices for coloring schemes:
			  *		- cmFromHeightRelativeToSensor: The Z coordinate wrt the sensor will be used to obtain the color using the limits z_min,z_max.
			  * 	- cmFromIntensityImage: When inserting 3D range scans, take the color from the intensity image channel, if available.
			  * \sa TColourOptions
			  */
			enum TColouringMethod
			{
				cmFromHeightRelativeToSensor = 0,
				cmFromHeightRelativeToSensorJet = 0,
				cmFromHeightRelativeToSensorGray = 1,
				cmFromIntensityImage = 2
			};

			/** The definition of parameters for generating colors from laser scans */
			 struct MAPS_IMPEXP TColourOptions : public utils::CLoadableOptions
			 {
				/** Initilization of default parameters */
				TColourOptions( );
				virtual ~TColourOptions() {}
				/** See utils::CLoadableOptions
				  */
				void  loadFromConfigFile(
					const mrpt::utils::CConfigFileBase  &source,
					const std::string &section);

				/** See utils::CLoadableOptions
				  */
				void  dumpToTextStream(CStream	&out) const;

				TColouringMethod	scheme;
				float				z_min,z_max;
				float				d_max;
			 };

			 TColourOptions	colorScheme;	//!< The options employed when inserting laser scans in the map.

			 void resetPointsMinDist( float defValue = 2000.0f ); //!< Reset the minimum-observed-distance buffer for all the points to a predefined value

            /** @name PCL library support
                @{ */

            /** Save the point cloud as a PCL PCD file, in either ASCII or binary format \return false on any error */
            virtual bool savePCDFile(const std::string &filename, bool save_as_binary) const;

            /** @} */


		protected:
			/** The color data */
			std::vector<float>	m_color_R,m_color_G,m_color_B;

			/** Minimum distance from where the points have been seen */
			//std::vector<float>	m_min_dist;

			/** Clear the map, erasing all the points.
			 */
			virtual void  internal_clear();

			/** @name Redefinition of PLY Import virtual methods from CPointsMap
			    @{ */
			/** In a base class, will be called after PLY_import_set_vertex_count() once for each loaded point.
			  *  \param pt_color Will be NULL if the loaded file does not provide color info.
			  */
			virtual void PLY_import_set_vertex(const size_t idx, const mrpt::math::TPoint3Df &pt, const mrpt::utils::TColorf *pt_color = NULL);

			/** In a base class, reserve memory to prepare subsequent calls to PLY_import_set_vertex */
			virtual void PLY_import_set_vertex_count(const size_t N);
			/** @} */

			/** @name Redefinition of PLY Export virtual methods from CPointsMap
			    @{ */
			/** In a base class, will be called after PLY_export_get_vertex_count() once for each exported point.
			  *  \param pt_color Will be NULL if the loaded file does not provide color info.
			  */
			virtual void PLY_export_get_vertex(
				const size_t idx,
				mrpt::math::TPoint3Df &pt,
				bool &pt_has_color,
				mrpt::utils::TColorf &pt_color) const;
			/** @} */

		}; // End of class def.

	} // End of namespace

#include <mrpt/utils/adapters.h>
	namespace utils
	{
		/** Specialization mrpt::utils::PointCloudAdapter<mrpt::slam::CColouredPointsMap> \ingroup mrpt_adapters_grp */
		template <>
		class PointCloudAdapter<mrpt::slam::CColouredPointsMap>
		{
		private:
			mrpt::slam::CColouredPointsMap &m_obj;
		public:
			typedef float  coords_t;         //!< The type of each point XYZ coordinates
			static const int HAS_RGB   = 1;  //!< Has any color RGB info?
			static const int HAS_RGBf  = 1;  //!< Has native RGB info (as floats)?
			static const int HAS_RGBu8 = 0;  //!< Has native RGB info (as uint8_t)?

			/** Constructor (accept a const ref for convenience) */
			inline PointCloudAdapter(const mrpt::slam::CColouredPointsMap &obj) : m_obj(*const_cast<mrpt::slam::CColouredPointsMap*>(&obj)) { }
			/** Get number of points */
			inline size_t size() const { return m_obj.size(); }
			/** Set number of points (to uninitialized values) */
			inline void resize(const size_t N) { m_obj.resize(N); }

			/** Get XYZ coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ(const size_t idx, T &x,T &y, T &z) const {
				m_obj.getPointFast(idx,x,y,z);
			}
			/** Set XYZ coordinates of i'th point */
			inline void setPointXYZ(const size_t idx, const coords_t x,const coords_t y, const coords_t z) {
				m_obj.setPointFast(idx,x,y,z);
			}

			/** Get XYZ_RGBf coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ_RGBf(const size_t idx, T &x,T &y, T &z, float &r,float &g,float &b) const {
				m_obj.getPoint(idx,x,y,z,r,g,b);
			}
			/** Set XYZ_RGBf coordinates of i'th point */
			inline void setPointXYZ_RGBf(const size_t idx, const coords_t x,const coords_t y, const coords_t z, const float r,const float g,const float b) {
				m_obj.setPoint(idx,x,y,z,r,g,b);
			}

			/** Get XYZ_RGBu8 coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ_RGBu8(const size_t idx, T &x,T &y, T &z, uint8_t &r,uint8_t &g,uint8_t &b) const {
				float Rf,Gf,Bf;
				m_obj.getPoint(idx,x,y,z,Rf,Gf,Bf);
				r=Rf*255; g=Gf*255; b=Bf*255;
			}
			/** Set XYZ_RGBu8 coordinates of i'th point */
			inline void setPointXYZ_RGBu8(const size_t idx, const coords_t x,const coords_t y, const coords_t z, const uint8_t r,const uint8_t g,const uint8_t b) {
				m_obj.setPoint(idx,x,y,z,r/255.f,g/255.f,b/255.f);
			}

			/** Get RGBf color of i'th point */
			inline void getPointRGBf(const size_t idx, float &r,float &g,float &b) const { m_obj.getPointColor_fast(idx,r,g,b); }
			/** Set XYZ_RGBf coordinates of i'th point */
			inline void setPointRGBf(const size_t idx, const float r,const float g,const float b) { m_obj.setPointColor_fast(idx,r,g,b); }

			/** Get RGBu8 color of i'th point */
			inline void getPointRGBu8(const size_t idx, uint8_t &r,uint8_t &g,uint8_t &b) const {
				float R,G,B;
				m_obj.getPointColor_fast(idx,R,G,B);
				r=R*255; g=G*255; b=B*255;
			}
			/** Set RGBu8 coordinates of i'th point */
			inline void setPointRGBu8(const size_t idx,const uint8_t r,const uint8_t g,const uint8_t b) {
				m_obj.setPointColor_fast(idx,r/255.f,g/255.f,b/255.f);
			}

		}; // end of PointCloudAdapter<mrpt::slam::CColouredPointsMap>

	}

} // End of namespace

#endif

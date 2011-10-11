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
#ifndef mrpt_maps_PCL_adapters_H
#define mrpt_maps_PCL_adapters_H

#include <mrpt/config.h>
#include <mrpt/utils/adapters.h>

// NOTE: Only include this file if you have PCL installed in your system
//        and do it only after including MRPT headers...

// Make sure the essential PCL headers are included:
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace mrpt
{
	namespace utils
	{
		/** Specialization mrpt::utils::PointCloudAdapter<pcl::PointCloud<pcl::PointXYZ> > for an XYZ point cloud (without RGB) \ingroup mrpt_adapters_grp */
		template <>
		class PointCloudAdapter<pcl::PointCloud<pcl::PointXYZ> > :  public detail::PointCloudAdapterHelperNoRGB<pcl::PointCloud<pcl::PointXYZ>,float>
		{
		private:
			pcl::PointCloud<pcl::PointXYZ> &m_obj;
		public:
			typedef float  coords_t;         //!< The type of each point XYZ coordinates
			static const int HAS_RGB   = 0;  //!< Has any color RGB info?
			static const int HAS_RGBf  = 0;  //!< Has native RGB info (as floats)?
			static const int HAS_RGBu8 = 0;  //!< Has native RGB info (as uint8_t)?

			/** Constructor (accept a const ref for convenience) */
			inline PointCloudAdapter(const pcl::PointCloud<pcl::PointXYZ> &obj) : m_obj(*const_cast<pcl::PointCloud<pcl::PointXYZ>*>(&obj)) { }
			/** Get number of points */
			inline size_t size() const { return m_obj.points.size(); }
			/** Set number of points (to uninitialized values) */
			inline void resize(const size_t N) { m_obj.points.resize(N); }

			/** Get XYZ coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ(const size_t idx, T &x,T &y, T &z) const {
				const pcl::PointXYZ &p=m_obj.points[idx];
				x=p.x; y=p.y; z=p.z;
			}
			/** Set XYZ coordinates of i'th point */
			inline void setPointXYZ(const size_t idx, const coords_t x,const coords_t y, const coords_t z) {
				pcl::PointXYZ &p=m_obj.points[idx];
				p.x=x; p.y=y; p.z=z;
			}
		}; // end of mrpt::utils::PointCloudAdapter<pcl::PointCloud<pcl::PointXYZ> >


		/** Specialization mrpt::utils::PointCloudAdapter<pcl::PointCloud<pcl::PointXYZRGB> > for an XYZ point cloud with RGB  \ingroup mrpt_adapters_grp */
		template <>
		class PointCloudAdapter<pcl::PointCloud<pcl::PointXYZRGB> >
		{
		private:
			pcl::PointCloud<pcl::PointXYZRGB> &m_obj;
		public:
			typedef float  coords_t;         //!< The type of each point XYZ coordinates
			static const int HAS_RGB   = 1;  //!< Has any color RGB info?
			static const int HAS_RGBf  = 0;  //!< Has native RGB info (as floats)?
			static const int HAS_RGBu8 = 1;  //!< Has native RGB info (as uint8_t)?

			/** Constructor (accept a const ref for convenience) */
			inline PointCloudAdapter(const pcl::PointCloud<pcl::PointXYZRGB> &obj) : m_obj(*const_cast<pcl::PointCloud<pcl::PointXYZRGB>*>(&obj)) { }
			/** Get number of points */
			inline size_t size() const { return m_obj.points.size(); }
			/** Set number of points (to uninitialized values) */
			inline void resize(const size_t N) { m_obj.points.resize(N); }

			/** Get XYZ coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ(const size_t idx, T &x,T &y, T &z) const {
				const pcl::PointXYZRGB &p=m_obj.points[idx];
				x=p.x; y=p.y; z=p.z;
			}
			/** Set XYZ coordinates of i'th point */
			inline void setPointXYZ(const size_t idx, const coords_t x,const coords_t y, const coords_t z) {
				pcl::PointXYZRGB &p=m_obj.points[idx];
				p.x=x; p.y=y; p.z=z;
				p.r=p.g=p.b=255;
			}

			/** Get XYZ_RGBf coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ_RGBf(const size_t idx, T &x,T &y, T &z, float &r,float &g,float &b) const {
				const pcl::PointXYZRGB &p=m_obj.points[idx];
				x=p.x; y=p.y; z=p.z;
				r=p.r/255.f; g=p.g/255.f; b=p.b/255.f;
			}
			/** Set XYZ_RGBf coordinates of i'th point */
			inline void setPointXYZ_RGBf(const size_t idx, const coords_t x,const coords_t y, const coords_t z, const float r,const float g,const float b) {
				pcl::PointXYZRGB &p=m_obj.points[idx];
				p.x=x; p.y=y; p.z=z;
				p.r=r*255; p.g=g*255; p.b=b*255;
			}

			/** Get XYZ_RGBu8 coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ_RGBu8(const size_t idx, T &x,T &y, T &z, uint8_t &r,uint8_t &g,uint8_t &b) const {
				const pcl::PointXYZRGB &p=m_obj.points[idx];
				x=p.x; y=p.y; z=p.z;
				r=p.r; g=p.g; b=p.b;
			}
			/** Set XYZ_RGBu8 coordinates of i'th point */
			inline void setPointXYZ_RGBu8(const size_t idx, const coords_t x,const coords_t y, const coords_t z, const uint8_t r,const uint8_t g,const uint8_t b) {
				pcl::PointXYZRGB &p=m_obj.points[idx];
				p.x=x; p.y=y; p.z=z;
				p.r=r; p.g=g; p.b=b;
			}

			/** Get RGBf color of i'th point */
			inline void getPointRGBf(const size_t idx, float &r,float &g,float &b) const {
				const pcl::PointXYZRGB &p=m_obj.points[idx];
				r=p.r/255.f; g=p.g/255.f; b=p.b/255.f;
			}
			/** Set XYZ_RGBf coordinates of i'th point */
			inline void setPointRGBf(const size_t idx, const float r,const float g,const float b) {
				pcl::PointXYZRGB &p=m_obj.points[idx];
				p.r=r*255; p.g=g*255; p.b=b*255;
			}

			/** Get RGBu8 color of i'th point */
			inline void getPointRGBu8(const size_t idx, uint8_t &r,uint8_t &g,uint8_t &b) const {
				const pcl::PointXYZRGB &p=m_obj.points[idx];
				r=p.r; g=p.g; b=p.b;
			}
			/** Set RGBu8 coordinates of i'th point */
			inline void setPointRGBu8(const size_t idx,const uint8_t r,const uint8_t g,const uint8_t b) {
				pcl::PointXYZRGB &p=m_obj.points[idx];
				p.r=r; p.g=g; p.b=b;
			}

		}; // end of mrpt::utils::PointCloudAdapter<pcl::PointCloud<pcl::PointXYZRGB> >

	}
} // End of namespace

#endif

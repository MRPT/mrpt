/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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


		/** Specialization mrpt::utils::PointCloudAdapter<pcl::PointCloud<pcl::PointXYZRGBA> > for an XYZ point cloud with RGB  \ingroup mrpt_adapters_grp */
		template <>
		class PointCloudAdapter<pcl::PointCloud<pcl::PointXYZRGBA> >
		{
		private:
			pcl::PointCloud<pcl::PointXYZRGBA> &m_obj;
		public:
			typedef float  coords_t;         //!< The type of each point XYZ coordinates
			static const int HAS_RGB   = 1;  //!< Has any color RGB info?
			static const int HAS_RGBf  = 0;  //!< Has native RGB info (as floats)?
			static const int HAS_RGBu8 = 1;  //!< Has native RGB info (as uint8_t)?

			/** Constructor (accept a const ref for convenience) */
			inline PointCloudAdapter(const pcl::PointCloud<pcl::PointXYZRGBA> &obj) : m_obj(*const_cast<pcl::PointCloud<pcl::PointXYZRGBA>*>(&obj)) { }
			/** Get number of points */
			inline size_t size() const { return m_obj.points.size(); }
			/** Set number of points (to uninitialized values) */
			inline void resize(const size_t N) { m_obj.points.resize(N); }

			/** Get XYZ coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ(const size_t idx, T &x,T &y, T &z) const {
				const pcl::PointXYZRGBA &p=m_obj.points[idx];
				x=p.x; y=p.y; z=p.z;
			}
			/** Set XYZ coordinates of i'th point */
			inline void setPointXYZ(const size_t idx, const coords_t x,const coords_t y, const coords_t z) {
				pcl::PointXYZRGBA &p=m_obj.points[idx];
				p.x=x; p.y=y; p.z=z;
				p.r=p.g=p.b=255;
			}

			/** Get XYZ_RGBf coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ_RGBf(const size_t idx, T &x,T &y, T &z, float &r,float &g,float &b) const {
				const pcl::PointXYZRGBA &p=m_obj.points[idx];
				x=p.x; y=p.y; z=p.z;
				r=p.r/255.f; g=p.g/255.f; b=p.b/255.f;
			}
			/** Set XYZ_RGBf coordinates of i'th point */
			inline void setPointXYZ_RGBf(const size_t idx, const coords_t x,const coords_t y, const coords_t z, const float r,const float g,const float b) {
				pcl::PointXYZRGBA &p=m_obj.points[idx];
				p.x=x; p.y=y; p.z=z;
				p.r=r*255; p.g=g*255; p.b=b*255;
			}

			/** Get XYZ_RGBu8 coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ_RGBu8(const size_t idx, T &x,T &y, T &z, uint8_t &r,uint8_t &g,uint8_t &b) const {
				const pcl::PointXYZRGBA &p=m_obj.points[idx];
				x=p.x; y=p.y; z=p.z;
				r=p.r; g=p.g; b=p.b;
			}
			/** Set XYZ_RGBu8 coordinates of i'th point */
			inline void setPointXYZ_RGBu8(const size_t idx, const coords_t x,const coords_t y, const coords_t z, const uint8_t r,const uint8_t g,const uint8_t b) {
				pcl::PointXYZRGBA &p=m_obj.points[idx];
				p.x=x; p.y=y; p.z=z;
				p.r=r; p.g=g; p.b=b;
			}

			/** Get RGBf color of i'th point */
			inline void getPointRGBf(const size_t idx, float &r,float &g,float &b) const {
				const pcl::PointXYZRGBA &p=m_obj.points[idx];
				r=p.r/255.f; g=p.g/255.f; b=p.b/255.f;
			}
			/** Set XYZ_RGBf coordinates of i'th point */
			inline void setPointRGBf(const size_t idx, const float r,const float g,const float b) {
				pcl::PointXYZRGBA &p=m_obj.points[idx];
				p.r=r*255; p.g=g*255; p.b=b*255;
			}

			/** Get RGBu8 color of i'th point */
			inline void getPointRGBu8(const size_t idx, uint8_t &r,uint8_t &g,uint8_t &b) const {
				const pcl::PointXYZRGBA &p=m_obj.points[idx];
				r=p.r; g=p.g; b=p.b;
			}
			/** Set RGBu8 coordinates of i'th point */
			inline void setPointRGBu8(const size_t idx,const uint8_t r,const uint8_t g,const uint8_t b) {
				pcl::PointXYZRGBA &p=m_obj.points[idx];
				p.r=r; p.g=g; p.b=b;
			}

		}; // end of mrpt::utils::PointCloudAdapter<pcl::PointCloud<pcl::PointXYZRGBA> >

	}
} // End of namespace

#endif

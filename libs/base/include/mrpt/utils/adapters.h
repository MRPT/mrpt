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
#ifndef mrpt_utils_adapters_H
#define mrpt_utils_adapters_H

#include <mrpt/utils/utils_defs.h>

namespace mrpt
{
	namespace utils
	{
		/** \defgroup mrpt_adapters_grp Adapter (wrapper) template classes
			\addtogroup mrpt_base_grp
		*/

		/** \addtogroup mrpt_adapters_grp
		  * @{ */

		/** An adapter to different kinds of point cloud object.
		  *  Implemented as a pure C++ template with specializations for the highest flexibility and efficiency in compiler-generated implementations.
		  *  Usage:
		  *   \code
		  *     PC  my_obj;
		  *     my_obj.specific_methods();
		  *     // ...
		  *     PointCloudAdapter<PC> pca(my_obj);
		  *     pca.unified_interface_methods();
		  *     // ...
		  *   \endcode
		  *  See specializations for details on the exposed API.
		  */
		template <class POINTCLOUD> class PointCloudAdapter;

		/** @} */  // end of grouping



	namespace detail
	{
		/** A helper base class for those PointCloudAdapter<> which do not handle RGB data; it declares needed interface methods which fall back to XYZ-only methods */
		template <class POINTMAPTYPE, typename coords_t>
		class PointCloudAdapterHelperNoRGB
		{
		public:
			typedef PointCloudAdapter<POINTMAPTYPE> Derived;

			inline       Derived & derived()       { return *static_cast<Derived*>(this); }
			inline const Derived & derived() const { return *static_cast<const Derived*>(this); }

			/** Get XYZ_RGBf coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ_RGBf(const size_t idx, T &x,T &y, T &z, float &r,float &g,float &b) const {
				derived().getPointXYZ(idx,x,y,z);
				r=g=b=1.0f;
			}
			/** Set XYZ_RGBf coordinates of i'th point */
			inline void setPointXYZ_RGBf(const size_t idx, const coords_t x,const coords_t y, const coords_t z, const float r,const float g,const float b) {
				derived().setPointXYZ(idx,x,y,z);
			}

			/** Get XYZ_RGBu8 coordinates of i'th point */
			template <typename T>
			inline void getPointXYZ_RGBu8(const size_t idx, T &x,T &y, T &z, uint8_t &r,uint8_t &g,uint8_t &b) const {
				derived().getPointXYZ(idx,x,y,z);
				r=g=b=255;
			}
			/** Set XYZ_RGBu8 coordinates of i'th point */
			inline void setPointXYZ_RGBu8(const size_t idx, const coords_t x,const coords_t y, const coords_t z, const uint8_t r,const uint8_t g,const uint8_t b) {
				derived().setPointXYZ(idx,x,y,z);
			}

			/** Get RGBf color of i'th point */
			inline void getPointRGBf(const size_t idx, float &r,float &g,float &b) const { r=g=b=1.0f; }
			/** Set XYZ_RGBf coordinates of i'th point */
			inline void setPointRGBf(const size_t idx, const float r,const float g,const float b) { }

			/** Get RGBu8 color of i'th point */
			inline void getPointRGBu8(const size_t idx, uint8_t &r,uint8_t &g,uint8_t &b) const { r=g=b=255; }
			/** Set RGBu8 coordinates of i'th point */
			inline void setPointRGBu8(const size_t idx,const uint8_t r,const uint8_t g,const uint8_t b) { }
		}; // end of PointCloudAdapterHelperNoRGB
	} // End of namespace detail

	} // End of namespace
} // end of namespace
#endif

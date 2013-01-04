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

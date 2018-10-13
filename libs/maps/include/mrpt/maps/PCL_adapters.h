/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config.h>
#include <mrpt/opengl/pointcloud_adapters.h>

// NOTE: Only include this file if you have PCL installed in your system
//        and do it only after including MRPT headers...

// Make sure the essential PCL headers are included:
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace mrpt::opengl
{
/** Specialization
 * mrpt::opengl::PointCloudAdapter<pcl::PointCloud<pcl::PointXYZ> > for an XYZ
 * point cloud (without RGB) \ingroup mrpt_adapters_grp */
template <>
class PointCloudAdapter<pcl::PointCloud<pcl::PointXYZ>>
{
   private:
	pcl::PointCloud<pcl::PointXYZ>& m_obj;

   public:
	/** The type of each point XYZ coordinates */
	using coords_t = float;
	/** Has any color RGB info? */
	static constexpr bool HAS_RGB = false;
	/** Has native RGB info (as floats)? */
	static constexpr bool HAS_RGBf = false;
	/** Has native RGB info (as uint8_t)? */
	static constexpr bool HAS_RGBu8 = false;

	/** Constructor (accept a const ref for convenience) */
	inline PointCloudAdapter(const pcl::PointCloud<pcl::PointXYZ>& obj)
		: m_obj(*const_cast<pcl::PointCloud<pcl::PointXYZ>*>(&obj))
	{
	}
	/** Get number of points */
	inline size_t size() const { return m_obj.points.size(); }
	/** Set number of points (to uninitialized values) */
	inline void resize(const size_t N) { m_obj.points.resize(N); }
	/** Set height and width (for organized) */
	inline void setDimensions(const size_t& height, const size_t& width)
	{
		m_obj.height = height;
		m_obj.width = width;
	}
	/** Get XYZ coordinates of i'th point */
	template <typename T>
	inline void getPointXYZ(const size_t idx, T& x, T& y, T& z) const
	{
		const pcl::PointXYZ& p = m_obj.points[idx];
		x = p.x;
		y = p.y;
		z = p.z;
	}
	/** Set XYZ coordinates of i'th point */
	inline void setPointXYZ(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z)
	{
		pcl::PointXYZ& p = m_obj.points[idx];
		p.x = x;
		p.y = y;
		p.z = z;
	}

	/** Set Invalid Point */
	inline void setInvalidPoint(const size_t idx)
	{
		pcl::PointXYZ& p = m_obj.points[idx];
		p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
	}
};  // end of mrpt::opengl::PointCloudAdapter<pcl::PointCloud<pcl::PointXYZ> >

/** Specialization
 * mrpt::opengl::PointCloudAdapter<pcl::PointCloud<pcl::PointXYZRGB> > for an
 * XYZ point cloud with RGB  \ingroup mrpt_adapters_grp */
template <>
class PointCloudAdapter<pcl::PointCloud<pcl::PointXYZRGB>>
{
   private:
	pcl::PointCloud<pcl::PointXYZRGB>& m_obj;

   public:
	/** The type of each point XYZ coordinates */
	using coords_t = float;
	/** Has any color RGB info? */
	static constexpr bool HAS_RGB = true;
	/** Has native RGB info (as floats)? */
	static constexpr bool HAS_RGBf = false;
	/** Has native RGB info (as uint8_t)? */
	static constexpr bool HAS_RGBu8 = true;

	/** Constructor (accept a const ref for convenience) */
	inline PointCloudAdapter(const pcl::PointCloud<pcl::PointXYZRGB>& obj)
		: m_obj(*const_cast<pcl::PointCloud<pcl::PointXYZRGB>*>(&obj))
	{
	}
	/** Get number of points */
	inline size_t size() const { return m_obj.points.size(); }
	/** Set number of points (to uninitialized values) */
	inline void resize(const size_t N) { m_obj.points.resize(N); }
	/** Set height and width (for organized) */
	inline void setDimensions(const size_t& height, const size_t& width)
	{
		m_obj.height = height;
		m_obj.width = width;
	}

	/** Get XYZ coordinates of i'th point */
	template <typename T>
	inline void getPointXYZ(const size_t idx, T& x, T& y, T& z) const
	{
		const pcl::PointXYZRGB& p = m_obj.points[idx];
		x = p.x;
		y = p.y;
		z = p.z;
	}
	/** Set XYZ coordinates of i'th point */
	inline void setPointXYZ(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z)
	{
		pcl::PointXYZRGB& p = m_obj.points[idx];
		p.x = x;
		p.y = y;
		p.z = z;
		p.r = p.g = p.b = 255;
	}

	/** Get XYZ_RGBf coordinates of i'th point */
	template <typename T>
	inline void getPointXYZ_RGBf(
		const size_t idx, T& x, T& y, T& z, float& r, float& g, float& b) const
	{
		const pcl::PointXYZRGB& p = m_obj.points[idx];
		x = p.x;
		y = p.y;
		z = p.z;
		r = p.r / 255.f;
		g = p.g / 255.f;
		b = p.b / 255.f;
	}
	/** Set XYZ_RGBf coordinates of i'th point */
	inline void setPointXYZ_RGBf(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z,
		const float r, const float g, const float b)
	{
		pcl::PointXYZRGB& p = m_obj.points[idx];
		p.x = x;
		p.y = y;
		p.z = z;
		p.r = r * 255;
		p.g = g * 255;
		p.b = b * 255;
	}

	/** Get XYZ_RGBu8 coordinates of i'th point */
	template <typename T>
	inline void getPointXYZ_RGBu8(
		const size_t idx, T& x, T& y, T& z, uint8_t& r, uint8_t& g,
		uint8_t& b) const
	{
		const pcl::PointXYZRGB& p = m_obj.points[idx];
		x = p.x;
		y = p.y;
		z = p.z;
		r = p.r;
		g = p.g;
		b = p.b;
	}
	/** Set XYZ_RGBu8 coordinates of i'th point */
	inline void setPointXYZ_RGBu8(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z,
		const uint8_t r, const uint8_t g, const uint8_t b)
	{
		pcl::PointXYZRGB& p = m_obj.points[idx];
		p.x = x;
		p.y = y;
		p.z = z;
		p.r = r;
		p.g = g;
		p.b = b;
	}

	/** Get RGBf color of i'th point */
	inline void getPointRGBf(
		const size_t idx, float& r, float& g, float& b) const
	{
		const pcl::PointXYZRGB& p = m_obj.points[idx];
		r = p.r / 255.f;
		g = p.g / 255.f;
		b = p.b / 255.f;
	}
	/** Set XYZ_RGBf coordinates of i'th point */
	inline void setPointRGBf(
		const size_t idx, const float r, const float g, const float b)
	{
		pcl::PointXYZRGB& p = m_obj.points[idx];
		p.r = r * 255;
		p.g = g * 255;
		p.b = b * 255;
	}

	/** Get RGBu8 color of i'th point */
	inline void getPointRGBu8(
		const size_t idx, uint8_t& r, uint8_t& g, uint8_t& b) const
	{
		const pcl::PointXYZRGB& p = m_obj.points[idx];
		r = p.r;
		g = p.g;
		b = p.b;
	}
	/** Set RGBu8 coordinates of i'th point */
	inline void setPointRGBu8(
		const size_t idx, const uint8_t r, const uint8_t g, const uint8_t b)
	{
		pcl::PointXYZRGB& p = m_obj.points[idx];
		p.r = r;
		p.g = g;
		p.b = b;
	}

};  // end of mrpt::opengl::PointCloudAdapter<pcl::PointCloud<pcl::PointXYZRGB>
// >

/** Specialization
 * mrpt::opengl::PointCloudAdapter<pcl::PointCloud<pcl::PointXYZRGBA> > for an
 * XYZ point cloud with RGB  \ingroup mrpt_adapters_grp */
template <>
class PointCloudAdapter<pcl::PointCloud<pcl::PointXYZRGBA>>
{
   private:
	pcl::PointCloud<pcl::PointXYZRGBA>& m_obj;

   public:
	/** The type of each point XYZ coordinates */
	using coords_t = float;
	/** Has any color RGB info? */
	static constexpr bool HAS_RGB = true;
	/** Has native RGB info (as floats)? */
	static constexpr bool HAS_RGBf = false;
	/** Has native RGB info (as uint8_t)? */
	static constexpr bool HAS_RGBu8 = true;

	/** Constructor (accept a const ref for convenience) */
	inline PointCloudAdapter(const pcl::PointCloud<pcl::PointXYZRGBA>& obj)
		: m_obj(*const_cast<pcl::PointCloud<pcl::PointXYZRGBA>*>(&obj))
	{
	}
	/** Get number of points */
	inline size_t size() const { return m_obj.points.size(); }
	/** Set number of points (to uninitialized values) */
	inline void resize(const size_t N) { m_obj.points.resize(N); }
	/** Set height and width (for organized) */
	inline void setDimensions(const size_t& height, const size_t& width)
	{
		m_obj.height = height;
		m_obj.width = width;
	}

	/** Get XYZ coordinates of i'th point */
	template <typename T>
	inline void getPointXYZ(const size_t idx, T& x, T& y, T& z) const
	{
		const pcl::PointXYZRGBA& p = m_obj.points[idx];
		x = p.x;
		y = p.y;
		z = p.z;
	}
	/** Set XYZ coordinates of i'th point */
	inline void setPointXYZ(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z)
	{
		pcl::PointXYZRGBA& p = m_obj.points[idx];
		p.x = x;
		p.y = y;
		p.z = z;
		p.r = p.g = p.b = 255;
	}

	/** Set Invalid Point */
	inline void setInvalidPoint(const size_t idx)
	{
		pcl::PointXYZRGBA& p = m_obj.points[idx];
		p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
	}

	/** Get XYZ_RGBf coordinates of i'th point */
	template <typename T>
	inline void getPointXYZ_RGBf(
		const size_t idx, T& x, T& y, T& z, float& r, float& g, float& b) const
	{
		const pcl::PointXYZRGBA& p = m_obj.points[idx];
		x = p.x;
		y = p.y;
		z = p.z;
		r = p.r / 255.f;
		g = p.g / 255.f;
		b = p.b / 255.f;
	}
	/** Set XYZ_RGBf coordinates of i'th point */
	inline void setPointXYZ_RGBf(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z,
		const float r, const float g, const float b)
	{
		pcl::PointXYZRGBA& p = m_obj.points[idx];
		p.x = x;
		p.y = y;
		p.z = z;
		p.r = r * 255;
		p.g = g * 255;
		p.b = b * 255;
	}

	/** Get XYZ_RGBu8 coordinates of i'th point */
	template <typename T>
	inline void getPointXYZ_RGBu8(
		const size_t idx, T& x, T& y, T& z, uint8_t& r, uint8_t& g,
		uint8_t& b) const
	{
		const pcl::PointXYZRGBA& p = m_obj.points[idx];
		x = p.x;
		y = p.y;
		z = p.z;
		r = p.r;
		g = p.g;
		b = p.b;
	}
	/** Set XYZ_RGBu8 coordinates of i'th point */
	inline void setPointXYZ_RGBu8(
		const size_t idx, const coords_t x, const coords_t y, const coords_t z,
		const uint8_t r, const uint8_t g, const uint8_t b)
	{
		pcl::PointXYZRGBA& p = m_obj.points[idx];
		p.x = x;
		p.y = y;
		p.z = z;
		p.r = r;
		p.g = g;
		p.b = b;
	}

	/** Get RGBf color of i'th point */
	inline void getPointRGBf(
		const size_t idx, float& r, float& g, float& b) const
	{
		const pcl::PointXYZRGBA& p = m_obj.points[idx];
		r = p.r / 255.f;
		g = p.g / 255.f;
		b = p.b / 255.f;
	}
	/** Set XYZ_RGBf coordinates of i'th point */
	inline void setPointRGBf(
		const size_t idx, const float r, const float g, const float b)
	{
		pcl::PointXYZRGBA& p = m_obj.points[idx];
		p.r = r * 255;
		p.g = g * 255;
		p.b = b * 255;
	}

	/** Get RGBu8 color of i'th point */
	inline void getPointRGBu8(
		const size_t idx, uint8_t& r, uint8_t& g, uint8_t& b) const
	{
		const pcl::PointXYZRGBA& p = m_obj.points[idx];
		r = p.r;
		g = p.g;
		b = p.b;
	}
	/** Set RGBu8 coordinates of i'th point */
	inline void setPointRGBu8(
		const size_t idx, const uint8_t r, const uint8_t g, const uint8_t b)
	{
		pcl::PointXYZRGBA& p = m_obj.points[idx];
		p.r = r;
		p.g = g;
		p.b = b;
	}

};  // end of mrpt::opengl::PointCloudAdapter<pcl::PointCloud<pcl::PointXYZRGBA>
// >
}  // namespace mrpt::opengl

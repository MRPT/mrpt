/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/bits_math.h>  // for .0_deg
#include <mrpt/core/round.h>  // for round()
#include <mrpt/math/TPose3D.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::poses
{
/** This is a template class for storing a 6-dimensional grid, with components
 * corresponding to Euler angle parameterization of SE(3) poses. \ingroup
 * poses_pdf_grp
 */
template <class T>
class CPose3DGridTemplate
{
   protected:
	using self_t = CPose3DGridTemplate<T>;

	mrpt::math::TPose3D m_bb_min{-1., -1., -1., -M_PI, -.5 * M_PI, -.5 * M_PI};
	mrpt::math::TPose3D m_bb_max{+1., +1., +1., +M_PI, +.5 * M_PI, +.5 * M_PI};

	/** Resolution of the grid */
	double m_resolutionXYZ{0.20}, m_resolutionYPR{mrpt::DEG2RAD(10.0)};

	uint32_t m_sizeX{0}, m_sizeY{0}, m_sizeZ{0};
	uint32_t m_sizeYaw{0}, m_sizePitch{0}, m_sizeRoll{0};

	// Cached products of the ones above:
	uint32_t m_size_xy, m_size_xyz, m_size_xyzY, m_size_xyzYP, m_size_xyzYPR;

	void update_cached_size_products()
	{
		m_size_xy = m_sizeX * m_sizeY;
		m_size_xyz = m_size_xy * m_sizeZ;
		m_size_xyzY = m_size_xyz * m_sizeYaw;
		m_size_xyzYP = m_size_xyzY * m_sizePitch;
		m_size_xyzYPR = m_size_xyzYP * m_sizeRoll;
	}

	/** Minimum "cell indexes" for each coordinate */
	int m_min_cidX, m_min_cidY, m_min_cidZ;
	int m_min_cidYaw, m_min_cidPitch, m_min_cidRoll;

	/** The data. Stored in this order:
	 *
	 *  m_data[
	 *     ci_x
	 *   + size_x     * ci_y
	 *   + size_xy    * ci_z
	 *   + size_xyz   * ci_yaw
	 *   + size_xyzy  * ci_pitch
	 *   + size_xyzyp * ci_roll
	 *  ]
	 */
	std::vector<T> m_data;

   public:
	/** Default constructor:
	 */
	CPose3DGridTemplate(
		const mrpt::math::TPose3D& bb_min =
			mrpt::math::TPose3D(-1., -1., -1., -M_PI, -.5 * M_PI, -.5 * M_PI),
		const mrpt::math::TPose3D& bb_max =
			mrpt::math::TPose3D(1., 1., 1., M_PI, .5 * M_PI, .5 * M_PI),
		double resolution_XYZ = 0.10,
		double resolution_YPR = mrpt::DEG2RAD(10.0))
	{
		setSize(bb_min, bb_max, resolution_XYZ, resolution_YPR);
	}

	virtual ~CPose3DGridTemplate() = default;

	/** @name Return "indexes" from coordinates
	 *   @{
	 */
	int x2idx(double x) const
	{
		const int idx = mrpt::round((x - m_bb_min.x) / m_resolutionXYZ);
		ASSERT_(idx >= 0 && idx < static_cast<int>(m_sizeX));
		return idx;
	}
	int y2idx(double y) const
	{
		const int idx = mrpt::round((y - m_bb_min.y) / m_resolutionXYZ);
		ASSERT_(idx >= 0 && idx < static_cast<int>(m_sizeY));
		return idx;
	}
	int z2idx(double z) const
	{
		const int idx = mrpt::round((z - m_bb_min.z) / m_resolutionXYZ);
		ASSERT_(idx >= 0 && idx < static_cast<int>(m_sizeZ));
		return idx;
	}
	int yaw2idx(double yaw) const
	{
		const int idx = mrpt::round((yaw - m_bb_min.yaw) / m_resolutionYPR);
		ASSERT_(idx >= 0 && idx < static_cast<int>(m_sizeYaw));
		return idx;
	}
	int pitch2idx(double pitch) const
	{
		const int idx = mrpt::round((pitch - m_bb_min.pitch) / m_resolutionYPR);
		ASSERT_(idx >= 0 && idx < static_cast<int>(m_sizePitch));
		return idx;
	}
	int roll2idx(double roll) const
	{
		const int idx = mrpt::round((roll - m_bb_min.roll) / m_resolutionYPR);
		ASSERT_(idx >= 0 && idx < static_cast<int>(m_sizeRoll));
		return idx;
	}
	/** @} */

	/** @name Return coordinates from "indexes"
	 *   @{
	 */
	double idx2x(uint32_t cx) const
	{
		ASSERT_(cx < m_sizeX);
		return m_bb_min.x + cx * m_resolutionXYZ;
	}
	double idx2y(uint32_t cy) const
	{
		ASSERT_(cy < m_sizeY);
		return m_bb_min.y + cy * m_resolutionXYZ;
	}
	double idx2z(uint32_t cz) const
	{
		ASSERT_(cz < m_sizeZ);
		return m_bb_min.z + cz * m_resolutionXYZ;
	}

	double idx2yaw(uint32_t cY) const
	{
		ASSERT_(cY < m_sizeYaw);
		return m_bb_min.yaw + cY * m_resolutionYPR;
	}
	double idx2pitch(uint32_t cP) const
	{
		ASSERT_(cP < m_sizePitch);
		return m_bb_min.pitch + cP * m_resolutionYPR;
	}
	double idx2roll(uint32_t cR) const
	{
		ASSERT_(cR < m_sizeRoll);
		return m_bb_min.roll + cR * m_resolutionYPR;
	}

	/** @} */

	/** Changes the limits and size of the grid, erasing previous contents:
	 */
	void setSize(
		const mrpt::math::TPose3D& bb_min, const mrpt::math::TPose3D& bb_max,
		double resolution_XYZ, double resolution_YPR)
	{
		// Checks
		for (int i = 0; i < 6; i++)
			ASSERT_GT_(bb_max[i], bb_min[i]);
		ASSERT_GT_(resolution_XYZ, .0);
		ASSERT_GT_(resolution_YPR, .0);

		// Copy data:
		m_bb_min = bb_min;
		m_bb_max = bb_max;
		m_resolutionXYZ = resolution_XYZ;
		m_resolutionYPR = resolution_YPR;

		// Compute the indexes of the starting borders:
		m_min_cidX = mrpt::round(bb_min.x / resolution_XYZ);
		m_min_cidY = mrpt::round(bb_min.y / resolution_XYZ);
		m_min_cidZ = mrpt::round(bb_min.z / resolution_XYZ);

		m_min_cidYaw = mrpt::round(bb_min.yaw / resolution_YPR);
		m_min_cidPitch = mrpt::round(bb_min.pitch / resolution_YPR);
		m_min_cidRoll = mrpt::round(bb_min.roll / resolution_YPR);

		// Compute new required space:
		m_sizeX = mrpt::round(bb_max.x / resolution_XYZ) - m_min_cidX + 1;
		m_sizeY = mrpt::round(bb_max.y / resolution_XYZ) - m_min_cidY + 1;
		m_sizeZ = mrpt::round(bb_max.z / resolution_XYZ) - m_min_cidZ + 1;

		m_sizeYaw = mrpt::round(bb_max.yaw / resolution_YPR) - m_min_cidYaw + 1;
		m_sizePitch =
			mrpt::round(bb_max.pitch / resolution_YPR) - m_min_cidPitch + 1;
		m_sizeRoll =
			mrpt::round(bb_max.roll / resolution_YPR) - m_min_cidRoll + 1;

		// Products:
		update_cached_size_products();

		// Resize "m_data":
		m_data.clear();
		m_data.resize(m_size_xyzYPR);
	}

	/** Reads the contents of a cell */
	const T* getByPos(
		double x, double y, double z, double yaw, double pitch,
		double roll) const
	{
		return getByIndex(
			x2idx(x), y2idx(y), z2idx(z), yaw2idx(yaw), pitch2idx(pitch),
			roll2idx(roll));
	}

	T* getByPos(
		double x, double y, double z, double yaw, double pitch, double roll)
	{
		return const_cast<T*>(const_cast<const self_t&>(*this).getByPos(
			x, y, z, yaw, pitch, roll));
	}

	const T* getByPos(const mrpt::math::TPose3D& p) const
	{
		return getByPos(p.x, p.y, p.z, p.yaw, p.pitch, p.roll);
	}

	T* getByPos(const mrpt::math::TPose3D& p)
	{
		return getByPos(p.x, p.y, p.z, p.yaw, p.pitch, p.roll);
	}

	/** Reads the contents of a cell */
	const T* getByIndex(int cx, int cy, int cz, int cY, int cP, int cR) const
	{
		ASSERT_(
			cx < static_cast<int>(m_sizeX) && cy < static_cast<int>(m_sizeY) &&
			cz < static_cast<int>(m_sizeZ) &&
			cY < static_cast<int>(m_sizeYaw) &&
			cP < static_cast<int>(m_sizePitch) &&
			cR < static_cast<int>(m_sizeRoll));
		return &m_data
			[cx + m_sizeX * cy + m_size_xy * cz + m_size_xyz * cY +
			 m_size_xyzY * cP + m_size_xyzYP * cR];
	}

	T* getByIndex(int cx, int cy, int cz, int cY, int cP, int cR)
	{
		return const_cast<T*>(const_cast<const self_t&>(*this).getByIndex(
			cx, cy, cz, cY, cP, cR));
	}

	/** Returns a XY slice of the grid, for given constant z,yaw, pitch and
	 * roll.
	 */
	template <class MATRIXLIKE>
	void getAsMatrix(
		MATRIXLIKE& outMat, const double z, const double yaw,
		const double pitch, const double roll) const
	{
		MRPT_START
		outMat.setSize(m_sizeY, m_sizeX);
		const auto cz = z2idx(z), cY = yaw2idx(yaw), cP = pitch2idx(pitch),
				   cR = roll2idx(roll);
		ASSERT_LT_(cz, m_sizeZ);
		ASSERT_LT_(cY, m_sizeYaw);
		ASSERT_LT_(cP, m_sizePitch);
		ASSERT_LT_(cR, m_sizeRoll);
		for (uint32_t cy = 0; cy < m_sizeY; cy++)
			for (uint32_t cx = 0; cx < m_sizeX; cx++)
				outMat(cy, cx) = *getByIndex(cx, cy, cz, cY, cP, cR);
		MRPT_END
	}

	/** Get info about the 6D grid */
	mrpt::math::TPose3D getMinBoundingBox() const { return m_bb_min; }
	mrpt::math::TPose3D getMaxBoundingBox() const { return m_bb_max; }

	double getResolutionXYZ() const { return m_resolutionXYZ; }
	double getResolutionAngles() const { return m_resolutionYPR; }

	inline void fill(const T& val)
	{
		for (auto& v : m_data)
			v = val;
	}

	auto getSizeX() const { return m_sizeX; }
	auto getSizeY() const { return m_sizeY; }
	auto getSizeZ() const { return m_sizeZ; }
	auto getSizeYaw() const { return m_sizeYaw; }
	auto getSizePitch() const { return m_sizePitch; }
	auto getSizeRoll() const { return m_sizeRoll; }

	auto getTotalVoxelCount() const { return m_size_xyzYPR; }

	const std::vector<T>& getData() const { return m_data; }
	std::vector<T>& getData() { return m_data; }

};	// End of class def.

}  // namespace mrpt::poses

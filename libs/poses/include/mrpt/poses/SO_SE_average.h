/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/math/types_math.h>

namespace mrpt::poses
{
/** \addtogroup poses_grp
 *  @{ */

/** Computes weighted and un-weighted averages of SO(2) or SO(3) orientations
 * \sa SE_average, SE_traits<2>, SE_traits<3>, CPose3D, CPose2D */
template <size_t DOF>
class SO_average;

/** Computes weighted and un-weighted averages of SE(2) or SE(3) poses
 * \sa SO_average, SE_traits<2>, SE_traits<3>, CPose3D, CPose2D */
template <size_t DOF>
class SE_average;

/** Computes weighted and un-weighted averages of SO(2) orientations.
 * Add values to average with \a append(), when done call \a get_average().
 * Use \a clear() to reset the accumulator and start a new average computation.
 * Theoretical base: Average on SO(2) manifolds is computed by averaging the
 * corresponding 2D points, then projecting the result back to the closest-point
 * in the manifold.
 * Shortly explained in [these
 * slides](http://ingmec.ual.es/~jlblanco/papers/blanco2013tutorial-manifolds-introduction-robotics.pdf)
 * \note Class introduced in MRPT 1.3.1
 * \sa SE_traits */
template <>
class SO_average<2>
{
   public:
	/** Constructor */
	SO_average();
	/** Resets the accumulator */
	void clear();
	/** Adds a new orientation (radians) to the computation \sa get_average */
	void append(const double orientation_rad);
	/** Adds a new orientation (radians) to the weighted-average computation \sa
	 * get_average */
	void append(const double orientation_rad, const double weight);
	/** Returns the average orientation (radians).
	 * \exception std::logic_error If no data point were inserted.
	 * \exception std::runtime_error Upon undeterminate average value (ie the
	 * average lays exactly on the origin point) and \a
	 * enable_exception_on_undeterminate is set to true (otherwise, the 0
	 * orientation would be returned)
	 * \sa append */
	double get_average() const;
	/** (Default=false) Set to true if you want to raise an exception on
	 * undetermined average values. */
	bool enable_exception_on_undeterminate{false};

   private:
	double m_count{0};
	double m_accum_x{0}, m_accum_y{0};
};  // end SO_average<2>

/** Computes weighted and un-weighted averages of SO(3) orientations.
 * Add values to average with \a append(), when done call \a get_average().
 * Use \a clear() to reset the accumulator and start a new average computation.
 * Theoretical base: Average on SO(3) manifolds is computed by averaging the
 * corresponding matrices, then projecting the result back to the closest matrix
 * in the manifold.
 * Shortly explained in [these
 * slides](http://ingmec.ual.es/~jlblanco/papers/blanco2013tutorial-manifolds-introduction-robotics.pdf)
 * See also: eq. (3.7) in "MEANS AND AVERAGING IN THE GROUP OF ROTATIONS",
 * MAHER MOAKHER, 2002.
 * \note Class introduced in MRPT 1.3.1
 * \sa SE_traits */
template <>
class SO_average<3>
{
   public:
	/** Constructor */
	SO_average();
	/** Resets the accumulator */
	void clear();
	/** Adds a new orientation to the computation \sa get_average */
	void append(const Eigen::Matrix3d& M);
	/** Adds a new orientation to the weighted-average computation \sa
	 * get_average */
	void append(const Eigen::Matrix3d& M, const double weight);
	/** Returns the average orientation.
	 * \exception std::logic_error If no data point were inserted.
	 * \exception std::runtime_error Upon undeterminate average value (ie there
	 * was a problem with the SVD) and \a enable_exception_on_undeterminate is
	 * set to true (otherwise, the 0 orientation would be returned)
	 * \sa append */
	Eigen::Matrix3d get_average() const;
	/** (Default=false) Set to true if you want to raise an exception on
	 * undetermined average values. */
	bool enable_exception_on_undeterminate{false};

   private:
	double m_count{0};
	Eigen::Matrix3d m_accum_rot;
};  // end SO_average<3>

/** Computes weighted and un-weighted averages of SE(2) poses.
 * Add values to average with \a append(), when done call \a get_average().
 * Use \a clear() to reset the accumulator and start a new average computation.
 * Theoretical base: See SO_average<2> for the rotation part. The translation
 * is a simple arithmetic mean in Euclidean space.
 * \note Class introduced in MRPT 1.3.1
 * \sa SE_traits */
template <>
class SE_average<2>
{
   public:
	/** Constructor */
	SE_average();
	/** Resets the accumulator */
	void clear();
	/** Adds a new pose to the computation \sa get_average */
	void append(const mrpt::poses::CPose2D& p);
	/** Adds a new pose to the weighted-average computation \sa get_average */
	void append(const mrpt::poses::CPose2D& p, const double weight);
	void append(const mrpt::math::TPose2D& p, const double weight);
	/** Returns the average pose.
	 * \exception std::logic_error If no data point were inserted.
	 * \exception std::runtime_error Upon undeterminate average value (ie the
	 * average lays exactly on the origin point) and \a
	 * enable_exception_on_undeterminate is set to true (otherwise, the 0
	 * orientation would be returned)
	 * \sa append */
	void get_average(mrpt::poses::CPose2D& out_mean) const;
	/** (Default=false) Set to true if you want to raise an exception on
	 * undetermined average values. */
	bool enable_exception_on_undeterminate{false};

   private:
	double m_count{0};
	double m_accum_x{0}, m_accum_y{0};
	SO_average<2> m_rot_part;
};  // end SE_average<2>

/** Computes weighted and un-weighted averages of SE(3) poses.
 * Add values to average with \a append(), when done call \a get_average().
 * Use \a clear() to reset the accumulator and start a new average computation.
 * Theoretical base: See SO_average<3> for the rotation part. The translation
 * is a simple arithmetic mean in Euclidean space.
 * \note Class introduced in MRPT 1.3.1
 * \sa SE_traits */
template <>
class SE_average<3>
{
   public:
	/** Constructor */
	SE_average();
	/** Resets the accumulator */
	void clear();
	/** Adds a new pose to the computation \sa get_average */
	void append(const mrpt::poses::CPose3D& p);
	/** Adds a new pose to the weighted-average computation \sa get_average */
	void append(const mrpt::poses::CPose3D& p, const double weight);
	void append(const mrpt::math::TPose3D& p, const double weight);
	/** Returns the average pose.
	 * \exception std::logic_error If no data point were inserted.
	 * \exception std::runtime_error Upon undeterminate average value (ie the
	 * average lays exactly on the origin point) and \a
	 * enable_exception_on_undeterminate is set to true (otherwise, the 0
	 * orientation would be returned)
	 * \sa append */
	void get_average(mrpt::poses::CPose3D& out_mean) const;
	/** (Default=false) Set to true if you want to raise an exception on
	 * undetermined average values. */
	bool enable_exception_on_undeterminate{false};

   private:
	double m_count{0};
	double m_accum_x{0}, m_accum_y{0}, m_accum_z{0};
	SO_average<3> m_rot_part;
};  // end SE_average<3>

/** @} */  // end of grouping

}  // namespace mrpt::poses

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPOSEORPOINT_H
#define CPOSEORPOINT_H

#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/homog_matrices.h>
#include <mrpt/math/CArrayNumeric.h>

#include <mrpt/poses/CPoseOrPoint_detail.h>

namespace mrpt
{
	/** \defgroup poses_grp 2D/3D points and poses
	  *  \ingroup mrpt_base_grp */

	/** \defgroup poses_pdf_grp 2D/3D point and pose PDFs
	  *  \ingroup mrpt_base_grp */

	/** Classes for 2D/3D geometry representation, both of single values and probability density distributions (PDFs) in many forms.
	 * \ingroup poses_grp poses_pdf_grp
	  */
	namespace poses
	{
		// For use in some constructors (eg. CPose3D)
		enum TConstructorFlags_Poses
		{
			UNINITIALIZED_POSE = 0
		};

		/** The base template class for 2D & 3D points and poses.
		 *   This class use the Curiously Recurring Template Pattern (CRTP) to define
		 *    a set of common methods to all the children classes without the cost
		 *    of virtual methods. Since most important methods are inline, they will be expanded
		 *    at compile time and optimized for every specific derived case.
		 *
		 *  For more information and examples, refer
		 *    to the <a href="http://www.mrpt.org/2D_3D_Geometry">2D/3D Geometry tutorial</a> online.
		 *
		 *
		 *  <center><h2>Introduction to 2D and 3D representation classes</h2></center>
		 *  <hr>
		 *  <p>
		 *  There are two class of spatial representation classes:
		 *  - Point: A point in the common mathematical sense, with no directional information.
		 *  	- 2D: A 2D point is represented just by its coordinates (x,y).
		 *  	- 3D: A 3D point is represented by its coordinates (x,y,z).
		 *  - Pose: It is a point, plus a direction.
		 *  	- 2D: A 2D pose is a 2D point plus a single angle, the yaw or &#966; angle: the angle from the positive X angle.
		 *  	- 3D: A 3D point is a 3D point plus three orientation angles (More details above).
		 *  </p>
		 *  In the case for a 3D orientation many representation angles can be used (Euler angles,yaw/pitch/roll,...)
		 *  but all of them can be handled by a 4x4 matrix called "Homogeneous Matrix". This matrix includes both, the
		 *  translation and the orientation for a point or a pose, and it can be obtained using
		 *  the method getHomogeneousMatrix() which is defined for any pose or point. Note that when the YPR angles are
		 *   used to define a 3D orientation, these three values can not be extracted from the matrix again.<br><br>
		 *
		 *  <b>Homogeneous matrices:</b> These are 4x4 matrices which can represent any translation or rotation in 2D & 3D.
		 *     See the tutorial online for more details. 			 *
		 *
		 *  <b>Operators:</b> There are operators defined for the pose compounding \f$ \oplus \f$ and inverse pose
		 *   compounding \f$ \ominus \f$ of poses and points. For example, let "a" and "b" be 2D or 3D poses. Then "a+b"
		 *   returns the resulting pose of "moving b" from "a"; and "b-a" returns the pose of "b" as it is seen
		 *   "from a". They can be mixed points and poses, being 2D or 3D, in these operators, with the following
		 *   results: <br>
		 *
		 * <div align="center" >
		 *  <pre>
		 *  Does "a+b" return a Pose or a Point?
		 * +---------------------------------+
		 * |  a \ b   |  Pose     |  Point   |
		 * +----------+-----------+----------+
		 * | Pose     |  Pose     |  Point   |
		 * | Point    |  Pose     |  Point   |
		 * +---------------------------------+
		 *
		 *  Does "a-b" return a Pose or a Point?
		 * +---------------------------------+
		 * |  a \ b   |  Pose     |  Point   |
		 * +----------+-----------+----------+
		 * | Pose     |  Pose     |  Pose    |
		 * | Point    |  Point    |  Point   |
		 * +---------------------------------+
		 *
		 *  Does "a+b" and "a-b" return a 2D or 3D object?
		 * +-------------------------+
		 * |  a \ b   |  2D   |  3D  |
		 * +----------+--------------+
		 * |  2D      |  2D   |  3D  |
		 * |  3D      |  3D   |  3D  |
		 * +-------------------------+
		 *
		 *  </pre>
		 * </div>
		 *
		 * \sa CPose,CPoint
		 * \ingroup poses_grp
		 */
		template <class DERIVEDCLASS>
		class CPoseOrPoint : public mrpt::poses::detail::pose_point_impl<DERIVEDCLASS, mrpt::poses::detail::T3DTypeHelper<DERIVEDCLASS>::is_3D_val>
		{
		public:
			/** Common members of all points & poses classes.
			    @{ */
			// Note: the access to "z" is implemented (only for 3D data types), in detail::pose_point_impl<>
			inline double x() const /*!< Get X coord. */ { return static_cast<const DERIVEDCLASS*>(this)->m_coords[0]; }
			inline double y() const /*!< Get Y coord. */ { return static_cast<const DERIVEDCLASS*>(this)->m_coords[1]; }

			inline double &x() /*!< Get ref to X coord. */ { return static_cast<DERIVEDCLASS*>(this)->m_coords[0]; }
			inline double &y() /*!< Get ref to Y coord. */ { return static_cast<DERIVEDCLASS*>(this)->m_coords[1]; }

			inline void x(const double v) /*!< Set X coord. */ { static_cast<DERIVEDCLASS*>(this)->m_coords[0]=v; }
			inline void y(const double v) /*!< Set Y coord. */ { static_cast<DERIVEDCLASS*>(this)->m_coords[1]=v; }

			inline void x_incr(const double v) /*!< X+=v */ { static_cast<DERIVEDCLASS*>(this)->m_coords[0]+=v; }
			inline void y_incr(const double v) /*!< Y+=v */ { static_cast<DERIVEDCLASS*>(this)->m_coords[1]+=v; }


			/** Return true for poses or points with a Z component, false otherwise. */
			static inline bool is3DPoseOrPoint() { return DERIVEDCLASS::is_3D_val!=0; }

			/** Returns the squared euclidean distance to another pose/point: */
			template <class OTHERCLASS>	inline double sqrDistanceTo(const CPoseOrPoint<OTHERCLASS> &b) const
			{
				using mrpt::utils::square;

				if (b.is3DPoseOrPoint())
				{
					if (is3DPoseOrPoint())
						  return  square(x()-b.x()) + square(y()-b.y()) + square(static_cast<const DERIVEDCLASS*>(this)->m_coords[2]-static_cast<const OTHERCLASS*>(&b)->m_coords[2]);
					else  return  square(x()-b.x()) + square(y()-b.y()) + square(static_cast<const OTHERCLASS*>(&b)->m_coords[2]);
				}
				else
				{
					if (is3DPoseOrPoint())
						  return  square(x()-b.x()) + square(y()-b.y()) + square(static_cast<const OTHERCLASS*>(&b)->m_coords[2]);
					else  return  square(x()-b.x()) + square(y()-b.y());
				}
			}

			/** Returns the Euclidean distance to another pose/point: */
			template <class OTHERCLASS>
			inline double distanceTo(const CPoseOrPoint<OTHERCLASS> &b) const
			{
				return std::sqrt( sqrDistanceTo(b));
			}

			/** Returns the squared 2D distance from this pose/point to a 2D point (ignores Z, if it exists). */
			inline double distance2DToSquare( double ax, double ay ) const { using mrpt::utils::square; return square(ax-x())+square(ay-y()); }

			/** Returns the squared 3D distance from this pose/point to a 3D point */
			inline double distance3DToSquare( double ax, double ay, double az ) const {
				using mrpt::utils::square;
				return square(ax-x())+square(ay-y())+square(az-(is3DPoseOrPoint() ? static_cast<const DERIVEDCLASS*>(this)->m_coords[2] : 0) );
			}

			/** Returns the 2D distance from this pose/point to a 2D point (ignores Z, if it exists). */
			inline double distance2DTo( double ax, double ay ) const { return std::sqrt(distance2DToSquare(ax,ay)); }

			/** Returns the 3D distance from this pose/point to a 3D point */
			inline double distance3DTo( double ax, double ay, double az ) const { return std::sqrt(distance3DToSquare(ax,ay,az));  }

			/** Returns the euclidean distance to a 3D point: */
			inline double distanceTo(const mrpt::math::TPoint3D &b) const { return distance3DTo(b.x,b.y,b.z); }

			/** Returns the euclidean norm of vector: \f$ ||\mathbf{x}|| = \sqrt{x^2+y^2+z^2} \f$ */
			inline double  norm() const
			{
				using mrpt::utils::square;
				return std::sqrt( square(x())+square(y())+ (!is3DPoseOrPoint() ? 0 : square(static_cast<const DERIVEDCLASS*>(this)->m_coords[2]) ) );
			}

			/** Return the pose or point as a 1xN vector with all the components (see derived classes for each implementation) */
			inline mrpt::math::CVectorDouble getAsVectorVal() const
			{
				mrpt::math::CVectorDouble v;
				static_cast<const DERIVEDCLASS*>(this)->getAsVector(v);
				return v;
			}

			/** Returns the corresponding 4x4 homogeneous transformation matrix for the point(translation) or pose (translation+orientation).
			* \sa getInverseHomogeneousMatrix
			*/
			inline mrpt::math::CMatrixDouble44 getHomogeneousMatrixVal() const
			{
				mrpt::math::CMatrixDouble44 m(mrpt::math::UNINITIALIZED_MATRIX);
				static_cast<const DERIVEDCLASS*>(this)->getHomogeneousMatrix(m);
				return m;
			}

			/** Returns the corresponding 4x4 inverse homogeneous transformation matrix for this point or pose.
			* \sa getHomogeneousMatrix
			*/
			inline void getInverseHomogeneousMatrix( mrpt::math::CMatrixDouble44 &out_HM ) const
			{	// Get current HM & inverse in-place:
				static_cast<const DERIVEDCLASS*>(this)->getHomogeneousMatrix(out_HM);
				mrpt::math::homogeneousMatrixInverse(out_HM);
			}

			//! \overload
			inline mrpt::math::CMatrixDouble44 getInverseHomogeneousMatrix() const
			{
				mrpt::math::CMatrixDouble44 M(mrpt::math::UNINITIALIZED_MATRIX);
				getInverseHomogeneousMatrix(M);
				return M;
			}

			/** Set all data fields to quiet NaN */
			virtual void setToNaN() = 0;

			/** @} */
		}; // End of class def.


	} // End of namespace
} // End of namespace

#endif

/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#ifndef CPOSE3D_H
#define CPOSE3D_H

#include <mrpt/poses/CPose.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/CQuaternion.h>

namespace mrpt
{
namespace poses
{
	class BASE_IMPEXP CPose3DQuat;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPose3D, CPose )

	/** A class used to store a 3D pose.
	 *    A class used to store a 3D (6D) pose, including the 3D coordinate
	 *      point and orientation angles. It is used in many situations,
	 *      from defining a robot pose, maps relative poses, sensors,...
	 *		See introduction in documentation for the CPoseOrPoint class.
			<br>For a complete description of Points/Poses, see mrpt::poses::CPoseOrPoint<br>
	 *
	 *  For a complete description of Points/Poses, see mrpt::poses::CPoseOrPoint, or refer
	 *    to the <a href="http://www.mrpt.org/2D_3D_Geometry">2D/3D Geometry tutorial</a> in the wiki.
	 *
	 *  To change the individual components of the pose, use CPose3D::setFromValues. This class assures that the internal
	 *   4x4 homogeneous coordinate matrix is always up-to-date with the "x y z yaw pitch roll" members.
	 *
	 *  Rotations in 3D can be also represented by quaternions. See mrpt::math::CQuaternion, and method CPose3D::getAsQuaternion.
	 *
	 *  This class and CPose3DQuat are very similar, and they can be converted to the each other automatically via transformation constructors.
	 *
	 *  There are Lie algebra methods: \a exp and \a ln (see the methods for documentation).
	 *

<div align=center>

<table class=MsoTableGrid border=1 cellspacing=0 cellpadding=0
 style='border-collapse:collapse;border:none'>
 <tr style='height:15.8pt'>
  <td width=676 colspan=2 style='width:507.25pt;border:solid windowtext 1.0pt;
  background:#E6E6E6;padding:0cm 5.4pt 0cm 5.4pt;height:15.8pt'>
  <p   align=center style='text-align:center'>poses::CPose3D</p>
  </td>
 </tr>
 <tr style='height:15.8pt'>
  <td width=350 style='width:262.65pt;border:solid windowtext 1.0pt;border-top:
  none;padding:0cm 5.4pt 0cm 5.4pt;height:15.8pt'>
  <p   align=center style='text-align:center'>Homogeneous
  transfomation matrix</p>
  </td>
  <td width=326 style='width:244.6pt;border-top:none;border-left:none;
  border-bottom:solid windowtext 1.0pt;border-right:solid windowtext 1.0pt;
  padding:0cm 5.4pt 0cm 5.4pt;height:15.8pt'>
  <p   align=center style='text-align:center'>Spatial
  representation</p>
  </td>
 </tr>
 <tr style='height:202.65pt'>
  <td width=350 style='width:262.65pt;border:solid windowtext 1.0pt;border-top:
  none;padding:0cm 5.4pt 0cm 5.4pt;height:202.65pt'>
  <div align=center>
  <table  Table border=0 cellspacing=0 cellpadding=0 width=334
   style='width:250.65pt;border-collapse:collapse'>
   <tr style='height:16.65pt'>
    <td width=66 style='width:49.65pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.65pt'>
    <p   align=center style='text-align:center'>cycp</p>
    </td>
    <td width=99 style='width:74.15pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.65pt'>
    <p   align=center style='text-align:center'>cyspsr-sycr</p>
    </td>
    <td width=87 style='width:65.55pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.65pt'>
    <p   align=center style='text-align:center'>cyspcr+sysr</p>
    </td>
    <td width=82 style='width:61.3pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.65pt'>
    <p   align=center style='text-align:center'>x</p>
    </td>
   </tr>
   <tr style='height:17.25pt'>
    <td width=66 style='width:49.65pt;padding:0cm 5.4pt 0cm 5.4pt;height:17.25pt'>
    <p   align=center style='text-align:center'>sycp</p>
    </td>
    <td width=99 style='width:74.15pt;padding:0cm 5.4pt 0cm 5.4pt;height:17.25pt'>
    <p   align=center style='text-align:center'>syspsr+cycr</p>
    </td>
    <td width=87 style='width:65.55pt;padding:0cm 5.4pt 0cm 5.4pt;height:17.25pt'>
    <p   align=center style='text-align:center'>syspcr-cysr</p>
    </td>
    <td width=82 style='width:61.3pt;padding:0cm 5.4pt 0cm 5.4pt;height:17.25pt'>
    <p   align=center style='text-align:center'>y</p>
    </td>
   </tr>
   <tr style='height:19.65pt'>
    <td width=66 style='width:49.65pt;padding:0cm 5.4pt 0cm 5.4pt;height:19.65pt'>
    <p   align=center style='text-align:center'>-sp</p>
    </td>
    <td width=99 style='width:74.15pt;padding:0cm 5.4pt 0cm 5.4pt;height:19.65pt'>
    <p   align=center style='text-align:center'>cpsr</p>
    </td>
    <td width=87 style='width:65.55pt;padding:0cm 5.4pt 0cm 5.4pt;height:19.65pt'>
    <p   align=center style='text-align:center'>cpcr</p>
    </td>
    <td width=82 style='width:61.3pt;padding:0cm 5.4pt 0cm 5.4pt;height:19.65pt'>
    <p   align=center style='text-align:center'>z</p>
    </td>
   </tr>
   <tr style='height:11.0pt'>
    <td width=66 style='width:49.65pt;padding:0cm 5.4pt 0cm 5.4pt;height:11.0pt'>
    <p   align=center style='text-align:center'>0</p>
    </td>
    <td width=99 style='width:74.15pt;padding:0cm 5.4pt 0cm 5.4pt;height:11.0pt'>
    <p   align=center style='text-align:center'>0</p>
    </td>
    <td width=87 style='width:65.55pt;padding:0cm 5.4pt 0cm 5.4pt;height:11.0pt'>
    <p   align=center style='text-align:center'>0</p>
    </td>
    <td width=82 style='width:61.3pt;padding:0cm 5.4pt 0cm 5.4pt;height:11.0pt'>
    <p   align=center style='text-align:center'>1</p>
    </td>
   </tr>
  </table>
  </div>
  <p   align=center style='text-align:center'><span lang=EN-GB>where:</span></p>
  <p   align=center style='text-align:center'><span lang=EN-GB>cy
  = cos Yaw ;  sy = sin Yaw</span></p>
  <p   align=center style='text-align:center'><span lang=EN-GB>cp
  = cos Pitch ; sp = sin Pitch</span></p>
  <p   align=center style='text-align:center'><span lang=EN-GB>cr
  = cos Roll ; sr = sin Roll</span></p>
  </td>
  <td width=326 style='width:244.6pt;border-top:none;border-left:none;
  border-bottom:solid windowtext 1.0pt;border-right:solid windowtext 1.0pt;
  padding:0cm 5.4pt 0cm 5.4pt;height:202.65pt'>
  <p   align=center style='text-align:center'><span lang=EN-GB><img  src="CPose3D.gif"></span></p>
  </td>
 </tr>
</table>

</div>

	  *
	 * \sa CPoseOrPoint,CPoint3D, mrpt::math::CQuaternion
	 */
	class BASE_IMPEXP CPose3D : public CPose
	{
		friend class CPose;
		friend class CPose2D;
		friend class CPoint;
		friend std::ostream BASE_IMPEXP & operator << (std::ostream& o, const CPose3D& p);

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPose3D )

	protected:
		mutable bool 	m_ypr_uptodate;			//!< Whether yaw/pitch/roll members are up-to-date since the last homogeneous matrix update.
		mutable double	m_yaw, m_pitch, m_roll;	//!< These variables are updated every time that the object homogeneous matrix is modified (construction, loading from values, pose composition, etc )
		mutable CMatrixDouble44		m_HM;  //!< The homogeneous matrix

		/** Rebuild the homog matrix from x,y,z and the angles. */
		void  rebuildHomogeneousMatrix();

		/** Updates Yaw/pitch/roll members from the HM  */
		inline void updateYawPitchRoll() const { if (!m_ypr_uptodate) { m_ypr_uptodate=true; getYawPitchRoll( m_yaw, m_pitch, m_roll ); } }

	 public:
		 /** Default constructor, with all the coordinates to zero. */
		 CPose3D();

		 /** Constructor with initilization of the pose; (remember that angles are always given in radians!)  */
		 CPose3D(const double x,const double  y,const double  z,const double  yaw=0, const double  pitch=0, const double roll=0);

		 /** Copy constructor.
		  */
		 CPose3D( const CPose3D &o);

		 /** Copy operator.
		  */
		 CPose3D & operator=( const CPose3D &o);

		 /** Constructor from a 4x4 homogeneous matrix: */
		 explicit CPose3D(const math::CMatrixDouble &m);

		 /** Constructor from a 4x4 homogeneous matrix: */
		 explicit CPose3D(const math::CMatrixDouble44 &m);

		 /** Constructor from a CPose2D object.
		  */
		 CPose3D(const CPose2D &);

		 /** Constructor from a CPoint3D object.
		  */
		 CPose3D(const CPoint3D &);

		 /** Constructor from lightweight object.
		  */
		 CPose3D(const mrpt::math::TPose3D &);

		 /** Constructor from a quaternion (which only represents the 3D rotation part) and a 3D displacement. */
		 CPose3D(const mrpt::math::CQuaternionDouble &q, const double x, const double y, const double z );

		 /** Constructor from a CPose3DQuat. */
		 CPose3D(const CPose3DQuat &);

		 /** Fast constructor that leaves all the data uninitialized - call with UNINITIALIZED_POSE as argument */
		 inline CPose3D(TConstructorFlags_Poses constructor_dummy_param) : m_ypr_uptodate(false), m_HM(UNINITIALIZED_MATRIX)
		 {
		 	m_is3D = true;  // At least initialize the 4th row of HM, since many methods assume it will be correct from the begining.
			m_HM(3,0)=m_HM(3,1)=m_HM(3,2)=0; m_HM(3,3)=1;
		 }

		 /** Returns the corresponding 4x4 homogeneous transformation matrix for the point(translation) or pose (translation+orientation).
		   * \sa getInverseHomogeneousMatrix, getRotationMatrix
		   */
		 void  getHomogeneousMatrix(CMatrixDouble44 & out_HM ) const {  out_HM = m_HM; }

		 inline const CMatrixDouble44 &getHomogeneousMatrixVal() const { return m_HM;}


		 /** Get the 3x3 rotation matrix, the upper-left part of the 4x4 homogeneous matrix.
		   * \sa getHomogeneousMatrix
		   */
		 inline void getRotationMatrix( mrpt::math::CMatrixDouble33 & ROT ) const
		 {	// Very fast extract of submatrix:
			for (int c=0;c<3;c++) {
				ROT.m_Val[c]=m_HM.m_Val[c];
				ROT.m_Val[3+c]=m_HM.m_Val[4+c];
				ROT.m_Val[6+c]=m_HM.m_Val[8+c];
			}
		 }
		 //! \overload
		 inline mrpt::math::CMatrixDouble33 getRotationMatrix() const
		 {
		 	mrpt::math::CMatrixDouble33  ROT(UNINITIALIZED_MATRIX);
		 	getRotationMatrix(ROT);
		 	return ROT;
		 }

		/** The operator \f$ a \oplus b \f$ is the pose compounding operator.
		   */
		 inline CPose3D  operator + (const CPose3D& b) const
		 {
		   CPose3D   ret(UNINITIALIZED_POSE);
		   ret.composeFrom(*this,b);
		   return ret;
		 }

		/** The operator \f$ a \oplus b \f$ is the pose compounding operator.
		   */
		 CPoint3D  operator + (const CPoint3D& b) const;

		/** The operator \f$ a \oplus b \f$ is the pose compounding operator.
		   */
		 CPoint3D  operator + (const CPoint2D& b) const;

		 /** Scalar sum of components: This is diferent from poses
		  *    composition, which is implemented as "+" operators.
		  * \sa normalizeAngles
		  */
		 void addComponents(const CPose3D &p);

		 /** Rebuild the internal matrix & update the yaw/pitch/roll angles within the ]-PI,PI] range (Must be called after using addComponents)
		  * \sa addComponents
		   */
		 void  normalizeAngles();

		 /** Scalar multiplication of x,y,z,yaw,pitch & roll (angles will be wrapped to the ]-pi,pi] interval).
		   */
		 void operator *=(const double s);

        /** Computes the spherical coordinates of a 3D point as seen from the 6D pose specified by this object.
          *  For the coordinate system see the top of this page.
          */
        void sphericalCoordinates(
            const TPoint3D &point,
            double &out_range,
            double &out_yaw,
            double &out_pitch ) const;

		/** An alternative, slightly more efficient way of doing \f$ G = P \oplus L \f$ with G and L being 3D points and P this 6D pose.
		  *  If pointers are provided, the corresponding Jacobians are returned.
		  *  "out_jacobian_df_dse3" stands for the Jacobian with respect to the 6D locally Euclidean vector in the tangent space of SE(3).
		  *  See <a href="http://www.mrpt.org/6D_poses:equivalences_compositions_and_uncertainty" >this report</a> for mathematical details.
		  *  \param  If set to true, the Jacobian "out_jacobian_df_dpose" uses a fastest linearized appoximation (valid only for small rotations!).
		  */
		void composePoint(double lx,double ly,double lz, double &gx, double &gy, double &gz,
			mrpt::math::CMatrixFixedNumeric<double,3,3>  *out_jacobian_df_dpoint=NULL,
			mrpt::math::CMatrixFixedNumeric<double,3,6>  *out_jacobian_df_dpose=NULL,
			mrpt::math::CMatrixFixedNumeric<double,3,6>  *out_jacobian_df_dse3=NULL,
			bool use_small_rot_approx = false) const;

		/** An alternative, slightly more efficient way of doing \f$ G = P \oplus L \f$ with G and L being 3D points and P this 6D pose.
		  * \note local_point is passed by value to allow global and local point to be the same variable
		  */
		inline void composePoint(const TPoint3D local_point, TPoint3D &global_point) const {
			composePoint(local_point.x,local_point.y,local_point.z,  global_point.x,global_point.y,global_point.z );
		}

		/** An alternative, slightly more efficient way of doing \f$ G = P \oplus L \f$ with G and L being 3D points and P this 6D pose.  */
		inline void composePoint(double lx,double ly,double lz, float &gx, float &gy, float &gz ) const {
			double ggx, ggy,ggz;
			composePoint(lx,ly,lz,ggx,ggy,ggz);
			gx = ggx; gy = ggy; gz = ggz;
		}

		/**  Computes the 3D point L such as \f$ L = G \ominus this \f$.
		  *  If pointers are provided, the corresponding Jacobians are returned.
		  *  "out_jacobian_df_dse3" stands for the Jacobian with respect to the 6D locally Euclidean vector in the tangent space of SE(3).
		  *  See <a href="http://www.mrpt.org/6D_poses:equivalences_compositions_and_uncertainty" >this report</a> for mathematical details.
		  * \sa composePoint, composeFrom
		  */
		void inverseComposePoint(const double gx,const double gy,const double gz,double &lx,double &ly,double &lz,
			mrpt::math::CMatrixFixedNumeric<double,3,3>  *out_jacobian_df_dpoint=NULL,
			mrpt::math::CMatrixFixedNumeric<double,3,6>  *out_jacobian_df_dpose=NULL,
			mrpt::math::CMatrixFixedNumeric<double,3,6>  *out_jacobian_df_dse3=NULL ) const;

		/**  Makes "this = A (+) B"; this method is slightly more efficient than "this= A + B;" since it avoids the temporary object.
		  *  \note A or B can be "this" without problems.
		  */
		void composeFrom(const CPose3D& A, const CPose3D& B );

		/** Make \f$ this = this \oplus b \f$  (\a b can be "this" without problems) */
		inline CPose3D&  operator += (const CPose3D& b)
		{
			composeFrom(*this,b);
		 	return *this;
		}

		/**  Makes \f$ this = A \ominus B \f$ this method is slightly more efficient than "this= A - B;" since it avoids the temporary object.
		  *  \note A or B can be "this" without problems.
		  * \sa composeFrom, composePoint
		  */
		void inverseComposeFrom(const CPose3D& A, const CPose3D& B );

		/** Compute \f$ RET = this \oplus b \f$  */
		inline CPose3D  operator - (const CPose3D& b) const
		{
			CPose3D ret(UNINITIALIZED_POSE);
			ret.inverseComposeFrom(*this,b);
			return ret;
		}

		/** Convert this pose into its inverse, saving the result in itself. */
		void inverse();

		// These three must be declared here because the next three virtual ones hide the base methods.
		inline double x() const { return m_x; }  //!< Get the X coordinate
		inline double y() const { return m_y; }  //!< Get the Y coordinate
		inline double z() const { return m_z; }  //!< Get the Z coordinate

		void x(const double x_) { m_HM.get_unsafe(0,3)= m_x=x_; }  //!< Set the X coordinate
		void y(const double y_) { m_HM.get_unsafe(1,3)= m_y=y_; }  //!< Set the Y coordinate
		void z(const double z_) { m_HM.get_unsafe(2,3)= m_z=z_; }  //!< Set the Z coordinate

		inline void x_incr(const double Ax) { m_HM.get_unsafe(0,3)= m_x+=Ax; }  //!< Increment the X coordinate
		inline void y_incr(const double Ay) { m_HM.get_unsafe(1,3)= m_y+=Ay; }  //!< Increment the Y coordinate
		inline void z_incr(const double Az) { m_HM.get_unsafe(2,3)= m_z+=Az; }  //!< Increment the Z coordinate

		/** Set the pose from a 3D position (meters) and yaw/pitch/roll angles (radians) - This method recomputes the internal homogeneous coordinates matrix.
		  * \sa getYawPitchRoll, setYawPitchRoll
		  */
		void  setFromValues(
			const double		x0,
			const double		y0,
			const double		z0,
			const double		yaw=0,
			const double		pitch=0,
			const double		roll=0);

		/** Set the pose from a 3D position (meters) and a quaternion, stored as [x y z qr qx qy qz] in a 7-element vector.
		  * \sa setFromValues, getYawPitchRoll, setYawPitchRoll, CQuaternion, getAsQuaternion
		  */
		template <typename VECTORLIKE>
		inline void  setFromXYZQ(
			const VECTORLIKE    &v,
			const size_t        index_offset = 0)
		{
			ASSERT_(v.size()>=7)
			// The 3x3 rotation part:
			mrpt::math::CQuaternion<typename VECTORLIKE::value_type> q( v[index_offset+3],v[index_offset+4],v[index_offset+5],v[index_offset+6] );
			q.rotationMatrixNoResize(m_HM);
			updateYawPitchRoll();
			m_HM.get_unsafe(0,3)=m_x = v[index_offset+0];
			m_HM.get_unsafe(1,3)=m_y = v[index_offset+1];
			m_HM.get_unsafe(2,3)=m_z = v[index_offset+2];
			m_HM.get_unsafe(3,0)=m_HM.get_unsafe(3,1)=m_HM.get_unsafe(3,2)=0;
			m_HM.get_unsafe(3,3)=1.0;
		}

		/** Set the 3 angles of the 3D pose (in radians) - This method recomputes the internal homogeneous coordinates matrix.
		  * \sa getYawPitchRoll, setFromValues
		  */
		inline void  setYawPitchRoll(
			const double		yaw_,
			const double		pitch_,
			const double		roll_)
		{
			setFromValues(x(),y(),z(),yaw_,pitch_,roll_);
		}

		/** Returns the three angles (yaw, pitch, roll), in radians, from the homogeneous matrix.
		  * \sa setFromValues, yaw, pitch, roll
		  */
		void  getYawPitchRoll( double &yaw, double &pitch, double &roll ) const;

		inline double yaw() const { updateYawPitchRoll(); return m_yaw; }  //!< Get the YAW angle (in radians)  \sa setFromValues
		inline double pitch() const { updateYawPitchRoll(); return m_pitch; }  //!< Get the PITCH angle (in radians) \sa setFromValues
		inline double roll() const { updateYawPitchRoll(); return m_roll; }  //!< Get the ROLL angle (in radians) \sa setFromValues

		/** The euclidean distance between two poses taken as two 6-length vectors (angles in radians).
		  */
		double distanceEuclidean6D( const CPose3D &o ) const;

		/** Returns a 1x6 vector with [x y z yaw pitch roll] */
		void getAsVector(vector_double &v) const;

		/** Returns the quaternion associated to the rotation of this object (NOTE: XYZ translation is ignored)
		  * \f[ \mathbf{q} = \left( \begin{array}{c} \cos (\phi /2) \cos (\theta /2) \cos (\psi /2) +  \sin (\phi /2) \sin (\theta /2) \sin (\psi /2) \\ \sin (\phi /2) \cos (\theta /2) \cos (\psi /2) -  \cos (\phi /2) \sin (\theta /2) \sin (\psi /2) \\ \cos (\phi /2) \sin (\theta /2) \cos (\psi /2) +  \sin (\phi /2) \cos (\theta /2) \sin (\psi /2) \\ \cos (\phi /2) \cos (\theta /2) \sin (\psi /2) -  \sin (\phi /2) \sin (\theta /2) \cos (\psi /2) \\ \end{array}\right) \f]
		  * With : \f$ \phi = roll \f$,  \f$ \theta = pitch \f$ and \f$ \psi = yaw \f$.
		  * \param out_dq_dr  If provided, the 4x3 Jacobian of the transformation will be computed and stored here. It's the Jacobian of the transformation from (yaw pitch roll) to (qr qx qy qz).
		  */
		void getAsQuaternion(
			mrpt::math::CQuaternionDouble &q,
			mrpt::math::CMatrixFixedNumeric<double,4,3>   *out_dq_dr = NULL
			) const;


		 /** Returns a human-readable textual representation of the object (eg: "[x y z yaw pitch roll]", angles in degrees.)
		   * \sa fromString
		   */
		void asString(std::string &s) const { updateYawPitchRoll(); s = mrpt::format("[%f %f %f %f %f %f]",m_x,m_y,m_z,RAD2DEG(m_yaw),RAD2DEG(m_pitch),RAD2DEG(m_roll)); }
		inline std::string asString() const { std::string s; asString(s); return s; }

		 /** Set the current object value from a string generated by 'asString' (eg: "[0.02 1.04 -0.8]" )
		   * \sa asString
		   * \exception std::exception On invalid format
		   */
		 void fromString(const std::string &s) {
		 	CMatrixDouble  m;
		 	if (!m.fromMatlabStringFormat(s)) THROW_EXCEPTION("Malformed expression in ::fromString");
			ASSERTMSG_(mrpt::math::size(m,1)==1 && mrpt::math::size(m,2)==6, "Wrong size of vector in ::fromString");
		 	this->setFromValues(m.get_unsafe(0,0),m.get_unsafe(0,1),m.get_unsafe(0,2),DEG2RAD(m.get_unsafe(0,3)),DEG2RAD(m.get_unsafe(0,4)),DEG2RAD(m.get_unsafe(0,5)));
		 }

		 /** Return true if the 6D pose represents a Z axis almost exactly vertical (upwards or downwards), with a given tolerance (if set to 0 exact horizontality is tested).
		   */
		bool isHorizontal( const double tolerance=0) const;

		inline const double &operator[](unsigned int i) const
		{
			updateYawPitchRoll();
			switch(i)
			{
				case 0:return m_x;
				case 1:return m_y;
				case 2:return m_z;
				case 3:return m_yaw;
				case 4:return m_pitch;
				case 5:return m_roll;
		 		default:
				throw std::runtime_error("CPose3D::operator[]: Index of bounds.");
			}
		}
		// CPose3D CANNOT have a write [] operator, since it'd leave the object in an inconsistent state.
		// Use setFromValues().
		// inline double &operator[](unsigned int i)

		/** makes: this = p (+) this */
		inline void  changeCoordinatesReference( const CPose3D & p ) { composeFrom(p,CPose3D(*this)); }


		/** @name Lie Algebra methods
		    @{ */

		/** Exponentiate a Vector in the SE3 Lie Algebra to generate a new CPose3D (static method).
		  * \note Method from TooN (C) Tom Drummond (GNU GPL) */
		static CPose3D exp(const mrpt::math::CArrayNumeric<double,6> & vect);

		/** Exponentiate a vector in the Lie algebra to generate a new SO3 (a 3x3 rotation matrix).
		  * \note Method from TooN (C) Tom Drummond (GNU GPL) */
		static CMatrixDouble33 exp_rotation(const mrpt::math::CArrayNumeric<double,3> & vect);


		/** Take the logarithm of the 3x4 matrix defined by this pose, generating the corresponding vector in the SE3 Lie Algebra.
		  * \note Method from TooN (C) Tom Drummond (GNU GPL)  */
		mrpt::math::CArrayDouble<6> ln() const;

		/** Take the logarithm of the 3x3 rotation matrix, generating the corresponding vector in the Lie Algebra.
		  * \note Method from TooN (C) Tom Drummond (GNU GPL) */
		CArrayDouble<3> ln_rotation() const;

		/** @} */



		typedef CPose3D  type_value; //!< Used to emulate CPosePDF types, for example, in CNetworkOfPoses

		/** @name STL-like methods and typedefs
		   @{   */
		typedef double         value_type;		//!< The type of the elements
		typedef double&        reference;
		typedef const double&  const_reference;
		typedef std::size_t    size_type;
		typedef std::ptrdiff_t difference_type;


		// size is constant
		enum { static_size = 6 };
		static inline size_type size() { return static_size; }
		static inline bool empty() { return false; }
		static inline size_type max_size() { return static_size; }
		static inline void resize(const size_t n) { if (n!=static_size) throw std::logic_error(format("Try to change the size of CPose3D to %u.",static_cast<unsigned>(n))); }
		/** @} */

	}; // End of class def.


	std::ostream BASE_IMPEXP  & operator << (std::ostream& o, const CPose3D& p);

	/** Unary - operator: return the inverse pose "-p" (Note that is NOT the same than a pose with negative x y z yaw pitch roll) */
	CPose3D BASE_IMPEXP operator -(const CPose3D &p);

	bool BASE_IMPEXP operator==(const CPose3D &p1,const CPose3D &p2);
	bool BASE_IMPEXP operator!=(const CPose3D &p1,const CPose3D &p2);

	} // End of namespace
} // End of namespace

#endif

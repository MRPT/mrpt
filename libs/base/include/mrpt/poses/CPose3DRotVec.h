/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPOSE3DROTVEC_H
#define CPOSE3DROTVEC_H

#include <mrpt/poses/CPose.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/poses_frwds.h>

namespace mrpt
{
namespace poses
{
	DEFINE_SERIALIZABLE_PRE( CPose3DRotVec )

	/** A 3D pose, with a 3D translation and a rotation in 3D parameterized in rotation-vector form (equivalent to axis-angle).
	 *   The 6D transformation in SE(3) stored in this class is kept in two
	 *   separate containers: a 3-array for the rotation vector, and a 3-array for the translation.
	 *
	 *   \code
	 *    CPose3DRotVec  pose;
	 *    pose.m_rotvec[{0:2}]=... // Rotation vector
	 *    pose.m_coords[{0:2}]=... // Translation
	 *   \endcode
	 *
	 *  For a complete descriptionan of Points/Poses, see mrpt::poses::CPoseOrPoint, or refer
	 *    to the <a href="http://www.mrpt.org/2D_3D_Geometry"> 2D/3D Geometry tutorial</a> online.
	 *
	 *  There are Lie algebra methods: \a exp and \a ln (see the methods for documentation).
	 *
	 * \ingroup poses_grp
	 * \sa CPose3DRotVec, CPoseOrPoint,CPoint3D, mrpt::math::CQuaternion
	 */
	class BASE_IMPEXP CPose3DRotVec : public CPose<CPose3DRotVec>, public mrpt::utils::CSerializable
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPose3DRotVec )

	public:
		mrpt::math::CArrayDouble<3>   m_coords; //!< The translation vector [x,y,z]
		mrpt::math::CArrayDouble<3>   m_rotvec; //!< The rotation vector [vx,vy,vz]

		/** @name Constructors
		    @{ */

		/** Default constructor, with all the coordinates set to zero. */
		inline CPose3DRotVec() {
		    m_coords[0]=m_coords[1]=m_coords[2]=0;
		    m_rotvec[0]=m_rotvec[1]=m_rotvec[2]=0;
		}

		/** Fast constructor that leaves all the data uninitialized - call with UNINITIALIZED_POSE as argument */
		inline CPose3DRotVec(TConstructorFlags_Poses constructor_dummy_param) : m_coords(),m_rotvec() {
			MRPT_UNUSED_PARAM(constructor_dummy_param);
		}

		/** Constructor with initilization of the pose */
		inline CPose3DRotVec(const double  vx, const double  vy, const double vz, const double x,const double  y,const double  z) {
		    m_coords[0]= x; m_coords[1]= y; m_coords[2]= z;
		    m_rotvec[0]=vx; m_rotvec[1]=vy; m_rotvec[2]=vz;
		}

		/** Constructor with initilization of the pose from a vector [w1 w2 w3 x y z] */
		inline CPose3DRotVec(const mrpt::math::CArrayDouble<6> &v) {
		    m_rotvec[0]=v[0]; m_rotvec[1]=v[1]; m_rotvec[2]=v[2];
		    m_coords[0]=v[3]; m_coords[1]=v[4]; m_coords[2]=v[5];
		}

		/** Copy constructor */
		inline CPose3DRotVec(const CPose3DRotVec &o)
		{
		    this->m_rotvec = o.m_rotvec;
		    this->m_coords = o.m_coords;
		}

		/** Constructor from a 4x4 homogeneous matrix: */
		explicit CPose3DRotVec(const math::CMatrixDouble44 &m);

		/** Constructor from a CPose3D object.*/
		explicit CPose3DRotVec(const CPose3D &m);

		/** Constructor from a quaternion (which only represents the 3D rotation part) and a 3D displacement. */
		CPose3DRotVec(const mrpt::math::CQuaternionDouble &q, const double x, const double y, const double z );

		/** Constructor from an array with these 6 elements: [w1 w2 w3 x y z]
		  *  where r{ij} are the entries of the 3x3 rotation matrix and t{x,y,z} are the 3D translation of the pose
		  *  \sa setFrom12Vector, getAs12Vector
		  */
		inline explicit CPose3DRotVec(const double *vec6) {
		    m_coords[0]=vec6[3]; m_coords[1]=vec6[4]; m_coords[2]=vec6[5];
		    m_rotvec[0]=vec6[0]; m_rotvec[1]=vec6[1]; m_rotvec[2]=vec6[2];
		}

		/** @} */  // end Constructors


		/** @name Access 3x3 rotation and 4x4 homogeneous matrices
		    @{ */

		/** Returns the corresponding 4x4 homogeneous transformation matrix for the point(translation) or pose (translation+orientation).
		  * \sa getInverseHomogeneousMatrix, getRotationMatrix
		  */
		inline void  getHomogeneousMatrix(mrpt::math::CMatrixDouble44 & out_HM) const {
		    out_HM.block<3,3>(0,0) = getRotationMatrix();
		    out_HM.set_unsafe(0,3,m_coords[0]);
		    out_HM.set_unsafe(1,3,m_coords[1]);
		    out_HM.set_unsafe(2,3,m_coords[2]);
		    out_HM.set_unsafe(3,0,0); out_HM.set_unsafe(3,1,0); out_HM.set_unsafe(3,2,0); out_HM.set_unsafe(3,3,1);
		}

		inline mrpt::math::CMatrixDouble44 getHomogeneousMatrixVal() const { mrpt::math::CMatrixDouble44 M; getHomogeneousMatrix(M); return M;}

		/** Get the 3x3 rotation matrix \sa getHomogeneousMatrix  */
		void getRotationMatrix( mrpt::math::CMatrixDouble33 & ROT ) const;
		//! \overload
		inline const mrpt::math::CMatrixDouble33 getRotationMatrix() const { mrpt::math::CMatrixDouble33 ROT(mrpt::math::UNINITIALIZED_MATRIX); getRotationMatrix(ROT); return ROT; }

		/** @} */  // end rot and HM


		/** @name Pose-pose and pose-point compositions and operators
		    @{ */

		/** The operator \f$ a \oplus b \f$ is the pose compounding operator. */
		inline CPose3DRotVec  operator + (const CPose3DRotVec& b) const
		{
			CPose3DRotVec ret(UNINITIALIZED_POSE);
			ret.composeFrom(*this,b);
			return ret;
		}

		/** The operator \f$ a \oplus b \f$ is the pose compounding operator. */
		CPoint3D  operator + (const CPoint3D& b) const;

		/** The operator \f$ a \oplus b \f$ is the pose compounding operator. */
		CPoint3D  operator + (const CPoint2D& b) const;

		inline void setFromTransformationMatrix( const mrpt::math::CMatrixDouble44 &m )
		{
		    mrpt::math::CArrayDouble<3> aux = rotVecFromRotMat( m );

		    this->m_rotvec[0] = aux[0];
		    this->m_rotvec[1] = aux[1];
		    this->m_rotvec[2] = aux[2];

		    this->m_coords[0] = m.get_unsafe(0,3);
		    this->m_coords[1] = m.get_unsafe(1,3);
		    this->m_coords[2] = m.get_unsafe(2,3);
		}

		void setFromXYZAndAngles( const double x,const double  y,const double  z,const double  yaw=0, const double  pitch=0, const double roll=0 );

        /** Computes the spherical coordinates of a 3D point as seen from the 6D pose specified by this object. For the coordinate system see mrpt::poses::CPose3D */
        void sphericalCoordinates(
            const mrpt::math::TPoint3D &point,
            double &out_range,
            double &out_yaw,
            double &out_pitch ) const;

		/** An alternative, slightly more efficient way of doing \f$ G = P \oplus L \f$ with G and L being 3D points and P this 6D pose.
		  *  If pointers are provided, the corresponding Jacobians are returned.
		  *  See <a href="http://www.mrpt.org/6D_poses:equivalences_compositions_and_uncertainty" >this report</a> for mathematical details.
		  */
		void composePoint(double lx,double ly,double lz, double &gx, double &gy, double &gz,
			mrpt::math::CMatrixFixedNumeric<double,3,3>  *out_jacobian_df_dpoint=NULL,
			mrpt::math::CMatrixFixedNumeric<double,3,6>  *out_jacobian_df_dpose=NULL) const;

		/** An alternative, slightly more efficient way of doing \f$ G = P \oplus L \f$ with G and L being 3D points and P this 6D pose.
		  * \note local_point is passed by value to allow global and local point to be the same variable
		  */
		inline void composePoint(const mrpt::math::TPoint3D local_point, mrpt::math::TPoint3D &global_point) const {
			composePoint(local_point.x,local_point.y,local_point.z,  global_point.x,global_point.y,global_point.z );
		}

		/**  Computes the 3D point L such as \f$ L = G \ominus this \f$.
		  *  If pointers are provided, the corresponding Jacobians are returned.
		  *  See <a href="http://www.mrpt.org/6D_poses:equivalences_compositions_and_uncertainty" >this report</a> for mathematical details.
		  * \sa composePoint, composeFrom
		  */
		void inverseComposePoint(const double gx,const double gy,const double gz,double &lx,double &ly,double &lz,
			mrpt::math::CMatrixFixedNumeric<double,3,3>  *out_jacobian_df_dpoint=NULL,
			mrpt::math::CMatrixFixedNumeric<double,3,6>  *out_jacobian_df_dpose=NULL) const;

		/**  Makes "this = A (+) B"; this method is slightly more efficient than "this= A + B;" since it avoids the temporary object.
		  *  \note A or B can be "this" without problems.
		  */
		void composeFrom(const CPose3DRotVec& A,
                         const CPose3DRotVec& B,
                         mrpt::math::CMatrixFixedNumeric<double,6,6>  *out_jacobian_drvtC_drvtA=NULL,
                         mrpt::math::CMatrixFixedNumeric<double,6,6>  *out_jacobian_drvtC_drvtB=NULL);


		/**  Convert this RVT into a quaternion + XYZ
		  */
        void toQuatXYZ(CPose3DQuat &q) const;

		/** Make \f$ this = this \oplus b \f$  (\a b can be "this" without problems) */
		inline CPose3DRotVec&  operator += (const CPose3DRotVec& b)
		{
			composeFrom(*this,b);
		 	return *this;
		}

		/** Copy operator */
		inline CPose3DRotVec& operator = (const CPose3DRotVec& o)
		{
            this->m_rotvec = o.m_rotvec;
		    this->m_coords = o.m_coords;
		    return *this;
		}

		/**  Makes \f$ this = A \ominus B \f$ this method is slightly more efficient than "this= A - B;" since it avoids the temporary object.
		  *  \note A or B can be "this" without problems.
		  * \sa composeFrom, composePoint
		  */
		void inverseComposeFrom(const CPose3DRotVec& A, const CPose3DRotVec& B );

		/** Compute \f$ RET = this \oplus b \f$  */
		inline CPose3DRotVec  operator - (const CPose3DRotVec& b) const
		{
			CPose3DRotVec ret(UNINITIALIZED_POSE);
			ret.inverseComposeFrom(*this,b);
			return ret;
		}

		/** Convert this pose into its inverse, saving the result in itself. \sa operator- */
		void inverse();

		/** Compute the inverse of this pose and return the result. */
		CPose3DRotVec getInverse() const;

		/** makes: this = p (+) this */
		inline void  changeCoordinatesReference( const CPose3DRotVec & p ) { composeFrom(p,CPose3DRotVec(*this)); }

		/** @} */  // compositions


		/** @name Access and modify contents
			@{ */

        inline double rx() const { return m_rotvec[0]; }
        inline double ry() const { return m_rotvec[1]; }
        inline double rz() const { return m_rotvec[2]; }

        inline double &rx() { return m_rotvec[0]; }
        inline double &ry() { return m_rotvec[1]; }
        inline double &rz() { return m_rotvec[2]; }

		/** Scalar sum of all 6 components: This is diferent from poses composition, which is implemented as "+" operators. */
		inline void addComponents(const CPose3DRotVec &p) {
		    m_coords+=p.m_coords;
		    m_rotvec+=p.m_rotvec;
        }

		/** Scalar multiplication of x,y,z,vx,vy,vz. */
		inline void operator *=(const double s) {
            m_coords*=s;
            m_rotvec*=s;
		}

		/** Create a vector with 3 components according to the input transformation matrix (only the rotation will be taken into account)
		  */
		mrpt::math::CArrayDouble<3> rotVecFromRotMat( const math::CMatrixDouble44 &m );

		/** Set the pose from a 3D position (meters) and yaw/pitch/roll angles (radians) - This method recomputes the internal rotation matrix.
		  * \sa getYawPitchRoll, setYawPitchRoll
		  */
		void  setFromValues(
			const double		x0,
			const double		y0,
			const double		z0,
			const double		vx,
			const double		vy,
			const double		vz )
        {
            m_coords[0]=x0; m_coords[1]=y0; m_coords[2]=z0;
            m_rotvec[0]=vx; m_rotvec[1]=vy; m_rotvec[2]=vz;
        }

		/** Set pose from an array with these 6 elements: [x y z vx vy vz]
		  *  where v{xyz} is the rotation vector and {xyz} the 3D translation of the pose
		  *  \sa getAs6Vector
		  */
		template <class ARRAYORVECTOR>
		inline void setFrom6Vector(const ARRAYORVECTOR &vec6)
		{
		    m_rotvec[0]=vec6[3]; m_rotvec[1]=vec6[4]; m_rotvec[2]=vec6[5];
		    m_coords[0]=vec6[0]; m_coords[1]=vec6[1]; m_coords[2]=vec6[2];
		}

		/** Gets pose as an array with these 6 elements: [x y z vx vy vz]
		  *  where v{xyz} is the rotation vector and {xyz} the 3D translation of the pose
		  *  The target vector MUST ALREADY have space for 6 elements (i.e. no .resize() method will be called).
		  *  \sa setAs6Vector, getAsVector
		  */
		template <class ARRAYORVECTOR>
		inline void getAs6Vector(ARRAYORVECTOR &vec6) const
		{
		    vec6[0]=m_coords[0]; vec6[1]=m_coords[1]; vec6[2]=m_coords[2];
		    vec6[3]=m_rotvec[0]; vec6[4]=m_rotvec[1]; vec6[5]=m_rotvec[2];
		}

		/** Like getAs6Vector() but for dynamic size vectors (required by base class CPoseOrPoint) */
		template <class ARRAYORVECTOR>
		inline void getAsVector(ARRAYORVECTOR &v) const { v.resize(6); getAs6Vector(v); }

		inline const double &operator[](unsigned int i) const
		{
			switch(i)
			{
				case 0:return m_coords[0];
				case 1:return m_coords[1];
				case 2:return m_coords[2];
				case 3:return m_rotvec[0];
				case 4:return m_rotvec[1];
				case 5:return m_rotvec[2];
		 		default:
				throw std::runtime_error("CPose3DRotVec::operator[]: Index of bounds.");
			}
		}
		inline double &operator[](unsigned int i)
		{
			switch(i)
			{
				case 0:return m_coords[0];
				case 1:return m_coords[1];
				case 2:return m_coords[2];
				case 3:return m_rotvec[0];
				case 4:return m_rotvec[1];
				case 5:return m_rotvec[2];
		 		default:
				throw std::runtime_error("CPose3DRotVec::operator[]: Index of bounds.");
			}
		}

		/** Returns a human-readable textual representation of the object: "[x y z rx ry rz]"
		  * \sa fromString
		  */
		void asString(std::string &s) const { s = mrpt::format("[%f %f %f %f %f %f]",m_coords[0],m_coords[1],m_coords[2],m_rotvec[0],m_rotvec[1],m_rotvec[2]); }
		inline std::string asString() const { std::string s; asString(s); return s; }

		/** Set the current object value from a string generated by 'asString' (eg: "[x y z yaw pitch roll]", angles in deg. )
		  * \sa asString
		  * \exception std::exception On invalid format
		  */
		void fromString(const std::string &s) {
		 	mrpt::math::CMatrixDouble  m;
		 	if (!m.fromMatlabStringFormat(s)) THROW_EXCEPTION("Malformed expression in ::fromString");
			ASSERTMSG_(mrpt::math::size(m,1)==1 && mrpt::math::size(m,2)==6, "Wrong size of vector in ::fromString");
		 	for (int i=0;i<3;i++) m_coords[i]=m.get_unsafe(0,i);
		 	for (int i=0;i<3;i++) m_rotvec[i]=m.get_unsafe(0,3+i);
		 }

		/** @} */  // modif. components


		/** @name Lie Algebra methods
		    @{ */

		/** Exponentiate a Vector in the SE(3) Lie Algebra to generate a new CPose3DRotVec (static method). */
		static CPose3DRotVec exp(const mrpt::math::CArrayDouble<6> & vect);

		/** Take the logarithm of the 3x4 matrix defined by this pose, generating the corresponding vector in the SE(3) Lie Algebra. */
		void ln(mrpt::math::CArrayDouble<6> &out_ln) const;

		/** Take the logarithm of the 3x3 rotation matrix part of this pose, generating the corresponding vector in the Lie Algebra. */
		mrpt::math::CArrayDouble<3> ln_rotation() const;

		/** @} */

		typedef CPose3DRotVec  type_value; //!< Used to emulate CPosePDF types, for example, in mrpt::graphs::CNetworkOfPoses
		enum { is_3D_val = 1 };
		static inline bool is_3D() { return is_3D_val!=0; }
		enum { rotation_dimensions = 3 };
		enum { is_PDF_val = 0 };
		static inline bool is_PDF() { return is_PDF_val!=0; }

		inline const type_value & getPoseMean() const { return *this; }
		inline       type_value & getPoseMean()       { return *this; }

		void setToNaN() MRPT_OVERRIDE;

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
		static inline void resize(const size_t n) { if (n!=static_size) throw std::logic_error(format("Try to change the size of CPose3DRotVec to %u.",static_cast<unsigned>(n))); }
		/** @} */

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST( CPose3DRotVec )


	std::ostream BASE_IMPEXP  & operator << (std::ostream& o, const CPose3DRotVec& p);

	/** Unary - operator: return the inverse pose "-p" (Note that is NOT the same than a pose with negative x y z yaw pitch roll) */
	CPose3DRotVec BASE_IMPEXP operator -(const CPose3DRotVec &p);

	bool BASE_IMPEXP operator==(const CPose3DRotVec &p1,const CPose3DRotVec &p2);
	bool BASE_IMPEXP operator!=(const CPose3DRotVec &p1,const CPose3DRotVec &p2);


	} // End of namespace
} // End of namespace

#endif

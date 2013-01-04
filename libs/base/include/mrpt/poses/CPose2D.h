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
#ifndef CPOSE2D_H
#define CPOSE2D_H

#include <mrpt/poses/CPose.h>

namespace mrpt
{
namespace poses
{
	DEFINE_SERIALIZABLE_PRE( CPose2D )

	/** A class used to store a 2D pose.
	 *    A class used to store a 2D pose, including the 2D coordinate
	 *      point and a heading (phi) angle.
	 *
	 *  For a complete description of Points/Poses, see mrpt::poses::CPoseOrPoint, or refer
	 *    to the <a href="http://www.mrpt.org/2D_3D_Geometry" >2D/3D Geometry tutorial</a> in the wiki.
	 *
	 *  <div align=center>
	 *   <img src="CPose2D.gif">
	 *  </div>
	 *
	 * \sa CPoseOrPoint,CPoint2D
	 * \ingroup poses_grp
	 */
	class BASE_IMPEXP CPose2D : public CPose<CPose2D>, public mrpt::utils::CSerializable
	{
	public:
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPose2D )

	public:
		mrpt::math::CArrayDouble<2>   m_coords; //!< [x,y]

	protected:
		double m_phi;  //!< The orientation of the pose, in radians.

	public:
		 /** Default constructor (all coordinates to 0) */
		 CPose2D();

		 /** Constructor from an initial value of the pose.*/
		 CPose2D(const double x,const double y,const double phi);

		 /** Constructor from a CPoint2D object. */
		 CPose2D(const CPoint2D &);

		 /** Aproximation!! Avoid its use, since information is lost. */
		 explicit CPose2D(const CPose3D &);

		 /** Constructor from lightweight object. */
		 CPose2D(const mrpt::math::TPose2D &);

		 /** Constructor from CPoint3D with information loss. */
		 explicit CPose2D(const CPoint3D &);

		 /** Fast constructor that leaves all the data uninitialized - call with UNINITIALIZED_POSE as argument */
		 inline CPose2D(TConstructorFlags_Poses constructor_dummy_param) { }

		 /** Get the phi angle of the 2D pose (in radians) */
		 inline const double &phi() const { return m_phi; }
		 //! \overload
		 inline       double &phi()       { return m_phi; }

		 /** Set the phi angle of the 2D pose (in radians) */
		 inline void phi(double angle) { m_phi=angle; }

		 inline void phi_incr(const double Aphi) { m_phi+=Aphi; }  //!< Increment the PHI angle (without checking the 2 PI range, call normalizePhi is needed)

		/** Returns a 1x3 vector with [x y phi] */
		void getAsVector(vector_double &v) const;
		/// \overload
		void getAsVector(mrpt::math::CArrayDouble<3> &v) const;

		 /** Returns the corresponding 4x4 homogeneous transformation matrix for the point(translation) or pose (translation+orientation).
		   * \sa getInverseHomogeneousMatrix
		   */
		 void  getHomogeneousMatrix(CMatrixDouble44 & out_HM ) const;

		 /** The operator \f$ a = this \oplus D \f$ is the pose compounding operator.
		  */
		 CPose2D  operator + (const CPose2D& D) const ;

		 /** Makes \f$ this = A \oplus B \f$
		  *  \note A or B can be "this" without problems.
		  */
		 void composeFrom(const CPose2D &A, const CPose2D &B);

		 /** The operator \f$ a = this \oplus D \f$ is the pose compounding operator.
		  */
		 CPose3D  operator + (const CPose3D& D) const ;

		 /** The operator \f$ u' = this \oplus u \f$ is the pose/point compounding operator.
		   */
		 CPoint2D operator + (const CPoint2D& u) const ;

		 /** An alternative, slightly more efficient way of doing \f$ G = P \oplus L \f$ with G and L being 2D points and P this 2D pose.  */
		 void composePoint(double lx,double ly,double &gx, double &gy) const;

		 /** The operator \f$ u' = this \oplus u \f$ is the pose/point compounding operator.
		   */
		 CPoint3D operator + (const CPoint3D& u) const ;

		/**  Makes \f$ this = A \ominus B \f$ this method is slightly more efficient than "this= A - B;" since it avoids the temporary object.
		  *  \note A or B can be "this" without problems.
		  * \sa composeFrom, composePoint
		  */
		void inverseComposeFrom(const CPose2D& A, const CPose2D& B );

		/** Compute \f$ RET = this \oplus b \f$  */
		inline CPose2D  operator - (const CPose2D& b) const
		{
			CPose2D ret(UNINITIALIZED_POSE);
			ret.inverseComposeFrom(*this,b);
			return ret;
		}

		 /** Scalar sum of components: This is diferent from poses
		  *    composition, which is implemented as "+" operators in "CPose" derived classes.
		  */
		 void AddComponents(CPose2D &p);

		 /** Scalar multiplication.
		  */
		 void operator *=(const double  s);

		 /** Make \f$ this = this \oplus b \f$  */
		 inline CPose2D&  operator += (const CPose2D& b)
		 {
		 	composeFrom(*this,b);
		 	return *this;
		 }

		 /** Forces "phi" to be in the range [-pi,pi];
		   */
		 void  normalizePhi();

		 /** Returns a human-readable textual representation of the object (eg: "[x y yaw]", yaw in degrees)
		   * \sa fromString
		   */
		void asString(std::string &s) const { s = mrpt::format("[%f %f %f]",x(),y(),RAD2DEG(m_phi)); }
		inline std::string asString() const { std::string s; asString(s); return s; }

		 /** Set the current object value from a string generated by 'asString' (eg: "[0.02 1.04 -0.8]" )
		   * \sa asString
		   * \exception std::exception On invalid format
		   */
		 void fromString(const std::string &s) {
		 	CMatrixDouble  m;
		 	if (!m.fromMatlabStringFormat(s)) THROW_EXCEPTION("Malformed expression in ::fromString");
		 	ASSERTMSG_(mrpt::math::size(m,1)==1 && mrpt::math::size(m,2)==3, "Wrong size of vector in ::fromString");
		 	x( m.get_unsafe(0,0) );
		 	y( m.get_unsafe(0,1) );
		 	phi( DEG2RAD(m.get_unsafe(0,2)) );
		 }

		 inline const double &operator[](unsigned int i)const
		 {
		 	switch(i)
		 	{
		 		case 0:return m_coords[0];
		 		case 1:return m_coords[1];
		 		case 2:return m_phi;
		 		default:
		 		throw std::runtime_error("CPose2D::operator[]: Index of bounds.");
		 	}
		 }
		 inline double &operator[](unsigned int i)
		 {
		 	switch(i)
		 	{
		 		case 0:return m_coords[0];
		 		case 1:return m_coords[1];
		 		case 2:return m_phi;
		 		default:
		 		throw std::runtime_error("CPose2D::operator[]: Index of bounds.");
		 	}
		 }

		/** makes: this = p (+) this */
		inline void  changeCoordinatesReference( const CPose2D & p ) { composeFrom(p,CPose2D(*this)); }

		typedef CPose2D  type_value; //!< Used to emulate CPosePDF types, for example, in mrpt::graphs::CNetworkOfPoses
		enum { is_3D_val = 0 };
		static inline bool is_3D() { return is_3D_val!=0; }
		enum { rotation_dimensions = 2 };
		enum { is_PDF_val = 0 };
		static inline bool is_PDF() { return is_PDF_val!=0; }

		inline const type_value & getPoseMean() const { return *this; }
		inline       type_value & getPoseMean()       { return *this; }

		/** @name STL-like methods and typedefs
		   @{   */
		typedef double         value_type;		//!< The type of the elements
		typedef double&        reference;
		typedef const double&  const_reference;
		typedef std::size_t    size_type;
		typedef std::ptrdiff_t difference_type;

		 // size is constant
		enum { static_size = 3 };
		static inline size_type size() { return static_size; }
		static inline bool empty() { return false; }
		static inline size_type max_size() { return static_size; }
		static inline void resize(const size_t n) { if (n!=static_size) throw std::logic_error(format("Try to change the size of CPose2D to %u.",static_cast<unsigned>(n))); }

		/** @} */

	}; // End of class def.


	std::ostream BASE_IMPEXP & operator << (std::ostream& o, const CPose2D& p);

	/** Unary - operator: return the inverse pose "-p" (Note that is NOT the same than a pose with negative x y phi) */
	CPose2D BASE_IMPEXP operator -(const CPose2D &p);

	mrpt::math::TPoint2D BASE_IMPEXP  operator +(const CPose2D &pose, const mrpt::math::TPoint2D &pnt);  //!< Compose a 2D point from a new coordinate base given by a 2D pose.

	bool BASE_IMPEXP operator==(const CPose2D &p1,const CPose2D &p2);
	bool BASE_IMPEXP operator!=(const CPose2D &p1,const CPose2D &p2);

	typedef mrpt::aligned_containers<CPose2D>::vector_t		StdVector_CPose2D; //!< Eigen aligment-compatible container
	typedef mrpt::aligned_containers<CPose2D>::deque_t 		StdDeque_CPose2D; //!< Eigen aligment-compatible container

	} // End of namespace
} // End of namespace

#endif

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPose3DQuat_H
#define CPose3DQuat_H

#include <mrpt/poses/CPose.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/poses_frwds.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt
{
namespace poses
{
	DEFINE_SERIALIZABLE_PRE( CPose3DQuat )

	/** A class used to store a 3D pose as a translation (x,y,z) and a quaternion (qr,qx,qy,qz).
	 *
	 *  For a complete description of Points/Poses, see mrpt::poses::CPoseOrPoint, or refer
	 *    to the <a href="http://www.mrpt.org/2D_3D_Geometry"> 2D/3D Geometry tutorial</a> in the wiki.
	 *
	 *  To access the translation use x(), y() and z(). To access the rotation, use CPose3DQuat::quat().
	 *
	 *  This class also behaves like a STL container, since it has begin(), end(), iterators, and can be accessed with the [] operator
	 *   with indices running from 0 to 6 to access the  [x y z qr qx qy qz] as if they were a vector. Thus, a CPose3DQuat can be used
	 *   as a 7-vector anywhere the MRPT math functions expect any kind of vector.
	 *
	 *  This class and CPose3D are very similar, and they can be converted to the each other automatically via transformation constructors.
	 *
	 * \sa CPose3D (for a class based on a 4x4 matrix instead of a quaternion), mrpt::math::TPose3DQuat, mrpt::poses::CPose3DQuatPDF for a probabilistic version of this class,  mrpt::math::CQuaternion, CPoseOrPoint
	 * \ingroup poses_grp
	 */
	class BASE_IMPEXP CPose3DQuat : public CPose<CPose3DQuat>, public mrpt::utils::CSerializable
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPose3DQuat )

	public:
		mrpt::math::CArrayDouble<3>     m_coords; //!< The translation vector [x,y,z]
		mrpt::math::CQuaternionDouble   m_quat;   //!< The quaternion.

	public:
		/** Read/Write access to the quaternion representing the 3D rotation. */
		inline       mrpt::math::CQuaternionDouble & quat()       { return m_quat; }
		/** Read-only access to the quaternion representing the 3D rotation. */
		inline const mrpt::math::CQuaternionDouble & quat() const { return m_quat; }

		/** Read/Write access to the translation vector in R^3. */
		inline       mrpt::math::CArrayDouble<3> & xyz()       { return m_coords; }
		/** Read-only access to the translation vector in R^3. */
		inline const mrpt::math::CArrayDouble<3> & xyz() const { return m_coords; }


		/** Default constructor, initialize translation to zeros and quaternion to no rotation. */
		inline CPose3DQuat() : m_quat() { m_coords[0]=m_coords[1]=m_coords[2]=0.; }

		/** Constructor which left all the quaternion members un-initialized, for use when speed is critical; Use UNINITIALIZED_POSE as argument to this constructor. */
		inline CPose3DQuat(mrpt::math::TConstructorFlags_Quaternions ) : m_quat(mrpt::math::UNINITIALIZED_QUATERNION) { }
		/** \overload */
		inline CPose3DQuat(TConstructorFlags_Poses )  : m_quat(mrpt::math::UNINITIALIZED_QUATERNION) { }

		/** Constructor with initilization of the pose - the quaternion is normalized to make sure it's unitary */
		inline CPose3DQuat(const double x,const double y,const double z,const mrpt::math::CQuaternionDouble &q ) : m_quat(q) { m_coords[0]=x; m_coords[1]=y; m_coords[2]=z; m_quat.normalize(); }

		/** Constructor from a CPose3D */
		explicit CPose3DQuat(const CPose3D &p);

		/** Constructor from lightweight object. */
		CPose3DQuat(const mrpt::math::TPose3DQuat &p) : m_quat(p.qr,p.qx,p.qy,p.qz) { x()=p.x; y()=p.y; z()=p.z; }

		/** Constructor from a 4x4 homogeneous transformation matrix.
		  */
		explicit CPose3DQuat(const mrpt::math::CMatrixDouble44 &M);

		/** Returns the corresponding 4x4 homogeneous transformation matrix for the point(translation) or pose (translation+orientation).
		  * \sa getInverseHomogeneousMatrix
		  */
		void  getHomogeneousMatrix(mrpt::math::CMatrixDouble44 & out_HM ) const;

		/** Returns a 1x7 vector with [x y z qr qx qy qz] */
		void getAsVector(mrpt::math::CVectorDouble &v) const;
		/// \overload
		void getAsVector(mrpt::math::CArrayDouble<7> &v) const {
			v[0] = m_coords[0]; v[1] = m_coords[1]; v[2] = m_coords[2];
			v[3] = m_quat[0]; v[4] = m_quat[1]; v[5] = m_quat[2]; v[6] = m_quat[3];
		}

		/**  Makes \f$ this = A \oplus B \f$  this method is slightly more efficient than "this= A + B;" since it avoids the temporary object.
		  *  \note A or B can be "this" without problems.
		  * \sa inverseComposeFrom, composePoint
		  */
		void composeFrom(const CPose3DQuat& A, const CPose3DQuat& B );

		/**  Makes \f$ this = A \ominus B \f$ this method is slightly more efficient than "this= A - B;" since it avoids the temporary object.
		  *  \note A or B can be "this" without problems.
		  * \sa composeFrom, composePoint
		  */
		void inverseComposeFrom(const CPose3DQuat& A, const CPose3DQuat& B );

		/**  Computes the 3D point G such as \f$ G = this \oplus L \f$.
		  * \sa composeFrom, inverseComposePoint
		  */
		void composePoint(const double lx,const double ly,const double lz,double &gx,double &gy,double &gz,
			 mrpt::math::CMatrixFixedNumeric<double,3,3>  *out_jacobian_df_dpoint = NULL,
			 mrpt::math::CMatrixFixedNumeric<double,3,7>  *out_jacobian_df_dpose = NULL ) const;

		/**  Computes the 3D point L such as \f$ L = G \ominus this \f$.
		  * \sa composePoint, composeFrom
		  */
		void inverseComposePoint(const double gx,const double gy,const double gz,double &lx,double &ly,double &lz,
			 mrpt::math::CMatrixFixedNumeric<double,3,3>  *out_jacobian_df_dpoint = NULL,
			 mrpt::math::CMatrixFixedNumeric<double,3,7>  *out_jacobian_df_dpose = NULL ) const;

		/**  Computes the 3D point G such as \f$ G = this \oplus L \f$.
		  *  POINT1 and POINT1 can be anything supporing [0],[1],[2].
		  * \sa composePoint    */
		template <class POINT1,class POINT2> inline void composePoint( const POINT1 &L, POINT2 &G) const { composePoint(L[0],L[1],L[2], G[0],G[1],G[2]); }

		/**  Computes the 3D point L such as \f$ L = G \ominus this \f$.  \sa inverseComposePoint */
		template <class POINT1,class POINT2> inline void inverseComposePoint( const POINT1 &G, POINT2 &L) const { inverseComposePoint(G[0],G[1],G[2],L[0],L[1],L[2]); }

		/**  Computes the 3D point G such as \f$ G = this \oplus L \f$.  \sa composePoint    */
		inline CPoint3D operator +( const CPoint3D &L) const { CPoint3D G; composePoint(L[0],L[1],L[2], G[0],G[1],G[2]); return G; }

		/**  Computes the 3D point G such as \f$ G = this \oplus L \f$.  \sa composePoint    */
		inline mrpt::math::TPoint3D operator +( const mrpt::math::TPoint3D &L) const { mrpt::math::TPoint3D G; composePoint(L[0],L[1],L[2], G[0],G[1],G[2]); return G; }

		/**  Computes the 3D point L such as \f$ L = G \ominus this \f$.  \sa inverseComposePoint    */
		inline CPoint3D operator -( const CPoint3D &G) const { CPoint3D L; inverseComposePoint(G[0],G[1],G[2], L[0],L[1],L[2]); return L; }

		/**  Computes the 3D point L such as \f$ L = G \ominus this \f$.  \sa inverseComposePoint    */
		inline mrpt::math::TPoint3D operator -( const mrpt::math::TPoint3D &G) const { mrpt::math::TPoint3D L; inverseComposePoint(G[0],G[1],G[2], L[0],L[1],L[2]); return L; }

		/** Scalar multiplication (all x y z qr qx qy qz elements are multiplied by the scalar).
		  */
		virtual void operator *=(const double  s);

		/** Make \f$ this = this \oplus b \f$  */
		inline CPose3DQuat&  operator += (const CPose3DQuat& b)
		{
			composeFrom(*this,b);
		 	return *this;
		}

		/** Return the composed pose \f$ ret = this \oplus p \f$  */
		inline CPose3DQuat operator + (const CPose3DQuat& p) const
		{
			CPose3DQuat ret;
			ret.composeFrom(*this,p);
		 	return ret;
		}

		/** Make \f$ this = this \ominus b \f$  */
		inline CPose3DQuat&  operator -= (const CPose3DQuat& b)
		{
			inverseComposeFrom(*this,b);
		 	return *this;
		}

		/** Return the composed pose \f$ ret = this \ominus p \f$  */
		inline CPose3DQuat operator - (const CPose3DQuat& p) const
		{
			CPose3DQuat ret;
			ret.inverseComposeFrom(*this,p);
		 	return ret;
		}

		/** Convert this pose into its inverse, saving the result in itself. \sa operator- */
		void inverse();

		 /** Returns a human-readable textual representation of the object (eg: "[x y z qr qx qy qz]", angles in degrees.)
		   * \sa fromString
		   */
		void asString(std::string &s) const { s = mrpt::format("[%f %f %f %f %f %f %f]",m_coords[0],m_coords[1],m_coords[2],m_quat[0],m_quat[1],m_quat[2],m_quat[3]); }
		inline std::string asString() const { std::string s; asString(s); return s; }

		 /** Set the current object value from a string generated by 'asString' (eg: "[0.02 1.04 -0.8 1 0 0 0]" )
		   * \sa asString
		   * \exception std::exception On invalid format
		   */
		 void fromString(const std::string &s) {
		 	mrpt::math::CMatrixDouble  m;
		 	if (!m.fromMatlabStringFormat(s)) THROW_EXCEPTION("Malformed expression in ::fromString");
			ASSERTMSG_(mrpt::math::size(m,1)==1 && mrpt::math::size(m,2)==7, "Wrong size of vector in ::fromString");
			m_coords[0] = m.get_unsafe(0,0); m_coords[1] = m.get_unsafe(0,1); m_coords[2] = m.get_unsafe(0,2);
			m_quat[0] = m.get_unsafe(0,3); m_quat[1] = m.get_unsafe(0,4); m_quat[2] = m.get_unsafe(0,5); m_quat[3] = m.get_unsafe(0,6);
		 }

		/** Read only [] operator */
		inline const double &operator[](unsigned int i) const
		{
			switch(i)
			{
				case 0:return m_coords[0];
				case 1:return m_coords[1];
				case 2:return m_coords[2];
				case 3:return m_quat[0];
				case 4:return m_quat[1];
				case 5:return m_quat[2];
				case 6:return m_quat[3];
		 		default:
				throw std::runtime_error("CPose3DQuat::operator[]: Index of bounds.");
			}
		}
		/** Read/write [] operator */
		inline double &operator[](unsigned int i)
		{
			switch(i)
			{
				case 0:return m_coords[0];
				case 1:return m_coords[1];
				case 2:return m_coords[2];
				case 3:return m_quat[0];
				case 4:return m_quat[1];
				case 5:return m_quat[2];
				case 6:return m_quat[3];
		 		default:
				throw std::runtime_error("CPose3DQuat::operator[]: Index of bounds.");
			}
		}

        /** Computes the spherical coordinates of a 3D point as seen from the 6D pose specified by this object.
          *  For the coordinate system see the top of this page.
		  *  If the matrix pointers are not NULL, the Jacobians will be also computed for the range-yaw-pitch variables wrt the passed 3D point and this 7D pose.
          */
        void sphericalCoordinates(
            const mrpt::math::TPoint3D &point,
            double &out_range,
            double &out_yaw,
            double &out_pitch,
			mrpt::math::CMatrixFixedNumeric<double,3,3> *out_jacob_dryp_dpoint = NULL,
			mrpt::math::CMatrixFixedNumeric<double,3,7> *out_jacob_dryp_dpose = NULL
			) const;

	public:
		typedef CPose3DQuat type_value; //!< Used to emulate CPosePDF types, for example, in mrpt::graphs::CNetworkOfPoses
		enum { is_3D_val = 1 };
		static inline bool is_3D() { return is_3D_val!=0; }
		enum { rotation_dimensions = 3 };
		enum { is_PDF_val = 1 };
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
		enum { static_size = 7 };
		static inline size_type size() { return static_size; }
		static inline bool empty() { return false; }
		static inline size_type max_size() { return static_size; }
		static inline void resize(const size_t n) { if (n!=static_size) throw std::logic_error(format("Try to change the size of CPose3DQuat to %u.",static_cast<unsigned>(n))); }

		inline void assign(const size_t N, const double val)
		{
			if (N!=7) throw std::runtime_error("CPose3DQuat::assign: Try to resize to length!=7.");
			m_coords.fill(val);
			m_quat.fill(val);
		}

		struct iterator : public std::iterator<std::random_access_iterator_tag,value_type>
		{
		private:
			typedef std::iterator<std::random_access_iterator_tag,value_type> iterator_base;
			CPose3DQuat *m_obj;		//!< A reference to the source of this iterator
			size_t		m_cur_idx;	//!< The iterator points to this element.
			typedef value_type T; //!< The type of the matrix elements

			inline void check_limits(bool allow_end = false) const
			{
	#ifdef _DEBUG
				ASSERTMSG_(m_obj!=NULL,"non initialized iterator");
				if (m_cur_idx> (allow_end ? 7u : 6u) ) THROW_EXCEPTION("Index out of range in iterator.")
	#else
				MRPT_UNUSED_PARAM(allow_end);
	#endif
			}
		public:
			inline bool operator <(const iterator &it2) const { return m_cur_idx < it2.m_cur_idx; }
			inline bool operator >(const iterator &it2) const { return m_cur_idx > it2.m_cur_idx; }
			inline iterator() : m_obj(NULL),m_cur_idx(0) { }
			inline iterator(CPose3DQuat &obj, size_t start_idx) : m_obj(&obj),m_cur_idx(start_idx) {  check_limits(true); /*Dont report as error an iterator to end()*/ }
			inline CPose3DQuat::reference operator*() const	{  check_limits();  return (*m_obj)[m_cur_idx];  }
			inline iterator &operator++() {
				check_limits();
				++m_cur_idx;
				return *this;
			}
			inline iterator operator++(int)	{
				iterator it=*this;
				++*this;
				return it;
			}
			inline iterator &operator--()	{
				--m_cur_idx;
				check_limits();
				return *this;
			}
			inline iterator operator--(int)	{
				iterator it=*this;
				--*this;
				return it;
			}
			inline iterator &operator+=(iterator_base::difference_type off)	{
				m_cur_idx+=off;
				check_limits(true);
				return *this;
			}
			inline iterator operator+(iterator_base::difference_type off) const	{
				iterator it=*this;
				it+=off;
				return it;
			}
			inline iterator &operator-=(iterator_base::difference_type off)	{
				return (*this)+=(-off);
			}
			inline iterator operator-(iterator_base::difference_type off) const	{
				iterator it=*this;
				it-=off;
				return it;
			}
			inline iterator_base::difference_type operator-(const iterator &it) const	{ return m_cur_idx - it.m_cur_idx; }
			inline CPose3DQuat::reference operator[](iterator_base::difference_type off) const { return (*m_obj)[m_cur_idx+off]; }
			inline bool operator==(const iterator &it) const { return m_obj==it.m_obj && m_cur_idx==it.m_cur_idx; }
			inline bool operator!=(const iterator &it) const { return !(operator==(it)); }
		}; // end iterator

		struct const_iterator : public std::iterator<std::random_access_iterator_tag,value_type>
		{
		private:
			typedef std::iterator<std::random_access_iterator_tag,value_type> iterator_base;
			const CPose3DQuat *m_obj;		//!< A reference to the source of this iterator
			size_t		m_cur_idx;	//!< The iterator points to this element.
			typedef value_type T; //!< The type of the matrix elements

			inline void check_limits(bool allow_end = false) const
			{
	#ifdef _DEBUG
				ASSERTMSG_(m_obj!=NULL,"non initialized iterator");
				if (m_cur_idx> (allow_end ? 7u : 6u) ) THROW_EXCEPTION("Index out of range in iterator.")
	#else
				MRPT_UNUSED_PARAM(allow_end);
	#endif
			}
		public:
			inline bool operator <(const const_iterator &it2) const { return m_cur_idx < it2.m_cur_idx; }
			inline bool operator >(const const_iterator &it2) const { return m_cur_idx > it2.m_cur_idx; }
			inline const_iterator() : m_obj(NULL),m_cur_idx(0) { }
			inline const_iterator(const CPose3DQuat &obj, size_t start_idx) : m_obj(&obj),m_cur_idx(start_idx) {  check_limits(true); /*Dont report as error an iterator to end()*/ }
			inline CPose3DQuat::const_reference operator*() const	{  check_limits();  return (*m_obj)[m_cur_idx];  }
			inline const_iterator &operator++() {
				check_limits();
				++m_cur_idx;
				return *this;
			}
			inline const_iterator operator++(int)	{
				const_iterator it=*this;
				++*this;
				return it;
			}
			inline const_iterator &operator--()	{
				--m_cur_idx;
				check_limits();
				return *this;
			}
			inline const_iterator operator--(int)	{
				const_iterator it=*this;
				--*this;
				return it;
			}
			inline const_iterator &operator+=(iterator_base::difference_type off)	{
				m_cur_idx+=off;
				check_limits(true);
				return *this;
			}
			inline const_iterator operator+(iterator_base::difference_type off) const	{
				const_iterator it=*this;
				it+=off;
				return it;
			}
			inline const_iterator &operator-=(iterator_base::difference_type off)	{
				return (*this)+=(-off);
			}
			inline const_iterator operator-(iterator_base::difference_type off) const	{
				const_iterator it=*this;
				it-=off;
				return it;
			}
			inline iterator_base::difference_type operator-(const const_iterator &it) const	{ return m_cur_idx - it.m_cur_idx; }
			inline CPose3DQuat::const_reference operator[](iterator_base::difference_type off) const { return (*m_obj)[m_cur_idx+off]; }
			inline bool operator==(const const_iterator &it) const { return m_obj==it.m_obj && m_cur_idx==it.m_cur_idx; }
			inline bool operator!=(const const_iterator &it) const { return !(operator==(it)); }
		}; // end const_iterator

		typedef std::reverse_iterator<iterator> 		reverse_iterator;
		typedef std::reverse_iterator<const_iterator> 	const_reverse_iterator;
		inline iterator 		begin()   { return iterator(*this,0); }
		inline iterator 		end()     { return iterator(*this,static_size); }
		inline const_iterator 	begin() const	{ return const_iterator(*this,0); }
		inline const_iterator 	end() const		{ return const_iterator(*this,static_size); }
		inline reverse_iterator 		rbegin() 		{ return reverse_iterator(end()); }
		inline const_reverse_iterator 	rbegin() const 	{ return const_reverse_iterator(end()); }
		inline reverse_iterator 		rend() 			{ return reverse_iterator(begin()); }
		inline const_reverse_iterator 	rend() const 	{ return const_reverse_iterator(begin()); }


		void swap (CPose3DQuat& o)
		{
			std::swap(o.m_coords, m_coords);
			o.m_quat.swap(m_quat);
		}

		/** @} */
		//! See ops_containers.h
		typedef CPose3DQuat  mrpt_autotype;
		//DECLARE_MRPT_CONTAINER_TYPES

		void setToNaN() MRPT_OVERRIDE;

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST( CPose3DQuat )

	std::ostream BASE_IMPEXP  & operator << (std::ostream& o, const CPose3DQuat& p);

	/** Unary - operator: return the inverse pose "-p" (Note that is NOT the same than a pose with all its arguments multiplied by "-1") */
	CPose3DQuat BASE_IMPEXP operator -(const CPose3DQuat &p);



	} // End of namespace
} // End of namespace

#endif

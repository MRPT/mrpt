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
#ifndef CPose3DQuat_H
#define CPose3DQuat_H

#include <mrpt/poses/CPose.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt
{
namespace poses
{
	using namespace mrpt::math;

	class CPose3D;

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPose3DQuat, CPose )

	/** A class used to store a 3D pose as a translation (x,y,z) and a quaternion (qr,qx,qy,qz).
	 *
	 *  For a complete description of Points/Poses, see mrpt::poses::CPoseOrPoint, or refer
	 *    to the <a href="http://www.mrpt.org/2D_3D_Geometry">2D/3D Geometry tutorial</a> in the wiki.
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
	 */
	class BASE_IMPEXP CPose3DQuat : public CPose
	{
		friend class CPose;
		friend class CPose2D;
		friend class CPoint;
		friend std::ostream BASE_IMPEXP & operator << (std::ostream& o, const CPose3DQuat& p);

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPose3DQuat )

	protected:
		mrpt::math::CQuaternionDouble   m_quat; //!< The quaternion.

	public:
		/** Read/Write access to the quaternion representing the 3D rotation. */
		inline mrpt::math::CQuaternionDouble & quat() { return m_quat; }

		/** Read-only access to the quaternion representing the 3D rotation. */
		inline const mrpt::math::CQuaternionDouble & quat() const { return m_quat; }

		/** Default constructor, initialize translation to zeros and quaternion to no rotation. */
		CPose3DQuat();

		/** Constructor which left all the quaternion members un-initialized, for use when speed is critical; Use UNINITIALIZED_POSE as argument to this constructor. */
		CPose3DQuat(TConstructorFlags_Quaternions constructor_dummy_param);

		/** Constructor with initilization of the pose */
		CPose3DQuat(const double x,const double y,const double z,const mrpt::math::CQuaternionDouble &q );

		/** Constructor from a CPose3D */
		CPose3DQuat(const CPose3D &p);

		/** Constructor from lightweight object.
		*/
		CPose3DQuat(const mrpt::math::TPose3DQuat &p);

		/** Constructor from a 4x4 homogeneous transformation matrix.
		  */
		explicit CPose3DQuat(const CMatrixDouble44 &M);

		/** Returns the corresponding 4x4 homogeneous transformation matrix for the point(translation) or pose (translation+orientation).
		  * \sa getInverseHomogeneousMatrix
		  */
		void  getHomogeneousMatrix(CMatrixDouble44 & out_HM ) const;

		/** Returns a 1x7 vector with [x y z qr qx qy qz] */
		void getAsVector(vector_double &v) const;

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
		inline TPoint3D operator +( const TPoint3D &L) const { TPoint3D G; composePoint(L[0],L[1],L[2], G[0],G[1],G[2]); return G; }

		/**  Computes the 3D point L such as \f$ L = G \ominus this \f$.  \sa inverseComposePoint    */
		inline CPoint3D operator -( const CPoint3D &G) const { CPoint3D L; inverseComposePoint(G[0],G[1],G[2], L[0],L[1],L[2]); return L; }

		/**  Computes the 3D point L such as \f$ L = G \ominus this \f$.  \sa inverseComposePoint    */
		inline TPoint3D operator -( const TPoint3D &G) const { TPoint3D L; inverseComposePoint(G[0],G[1],G[2], L[0],L[1],L[2]); return L; }

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

		 /** Returns a human-readable textual representation of the object (eg: "[x y z qr qx qy qz]", angles in degrees.)
		   * \sa fromString
		   */
		void asString(std::string &s) const { s = mrpt::format("[%f %f %f %f %f %f %f]",m_x,m_y,m_z,m_quat[0],m_quat[1],m_quat[2],m_quat[3]); }
		inline std::string asString() const { std::string s; asString(s); return s; }

		 /** Set the current object value from a string generated by 'asString' (eg: "[0.02 1.04 -0.8 1 0 0 0]" )
		   * \sa asString
		   * \exception std::exception On invalid format
		   */
		 void fromString(const std::string &s) {
		 	CMatrixDouble  m;
		 	if (!m.fromMatlabStringFormat(s)) THROW_EXCEPTION("Malformed expression in ::fromString");
			ASSERTMSG_(mrpt::math::size(m,1)==1 && mrpt::math::size(m,2)==7, "Wrong size of vector in ::fromString");
			m_x = m.get_unsafe(0,0); m_y = m.get_unsafe(0,1); m_z = m.get_unsafe(0,2);
			m_quat[0] = m.get_unsafe(0,3); m_quat[1] = m.get_unsafe(0,4); m_quat[2] = m.get_unsafe(0,5); m_quat[3] = m.get_unsafe(0,6);
		 }

		/** Read only [] operator */
		inline const double &operator[](unsigned int i) const
		{
			switch(i)
			{
				case 0:return m_x;
				case 1:return m_y;
				case 2:return m_z;
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
				case 0:return m_x;
				case 1:return m_y;
				case 2:return m_z;
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
            const TPoint3D &point,
            double &out_range,
            double &out_yaw,
            double &out_pitch,
			mrpt::math::CMatrixFixedNumeric<double,3,3> *out_jacob_dryp_dpoint = NULL,
			mrpt::math::CMatrixFixedNumeric<double,3,7> *out_jacob_dryp_dpose = NULL
			) const;

	public:
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

		void assign(const size_t N, const double val)
		{
			if (N!=7) throw std::runtime_error("CPose3DQuat::assign: Try to resize to length!=7.");
			m_x = m_y = m_z = val;
			m_quat[0]=m_quat[1]=m_quat[2]=m_quat[3]= val;
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
			std::swap(o.m_x,m_x); std::swap(o.m_y,m_y); std::swap(o.m_z,m_z);
			o.m_quat.swap(m_quat);
		}

		/** @} */
		//! See ops_containers.h
		typedef CPose3DQuat  mrpt_autotype;
		DECLARE_MRPT_CONTAINER_TYPES


	}; // End of class def.

	std::ostream BASE_IMPEXP  & operator << (std::ostream& o, const CPose3DQuat& p);


	} // End of namespace
} // End of namespace

#endif

/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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
#ifndef _MRPT_CArray_H
#define _MRPT_CArray_H

#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/utils/CSerializable.h>  // Used only for the extension to TTypeName

namespace mrpt
{
namespace math
{
	// ----------------  CArray -------------------------
	/** A STL container (as wrapper) for arrays of constant size defined at compile time - <b>Users will most likely prefer to use CArrayPOD and its derived classes instead</b>.
	 *
	 * This code is an adapted version from Boost, modifed for its integration
	 *	within MRPT (JLBC, Dec/2009) (Renamed array -> CArray to avoid possible potential conflicts).
	 *
	 * See
	 *      http://www.josuttis.com/cppcode
	 * for details and the latest version.
	 * See
	 *      http://www.boost.org/libs/array for Documentation.
	 * for documentation.
	 *
	 * (C) Copyright Nicolai M. Josuttis 2001.
	 * Permission to copy, use, modify, sell and distribute this software
	 * is granted provided this copyright notice appears in all copies.
	 * This software is provided "as is" without express or implied
	 * warranty, and with no claim as to its suitability for any purpose.
	 *
	 * 29 Jan 2004 - minor fixes (Nico Josuttis)
	 * 04 Dec 2003 - update to synch with library TR1 (Alisdair Meredith)
	 * 23 Aug 2002 - fix for Non-MSVC compilers combined with MSVC libraries.
	 * 05 Aug 2001 - minor update (Nico Josuttis)
	 * 20 Jan 2001 - STLport fix (Beman Dawes)
	 * 29 Sep 2000 - Initial Revision (Nico Josuttis)
	 *
	 * Jan 30, 2004
	 *
	 * \note This class DOES NOT support mathematical operations on its elements: it's a generic container, it doesn't assume they are numerical.
	 * \note For a summary and classification of all MRPT vector, array and matrix classes see: http://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes
	 *
     * \sa CArrayNumeric (for another, non-related base template class that DOES support maths)
	 * \ingroup mrpt_base_grp
	 */
    template <typename T, std::size_t N>
    class CArray {
      public:
        T elems[N];    // fixed-size array of elements of type T

      public:
        // type definitions
        typedef T              value_type;
        typedef T*             iterator;
        typedef const T*       const_iterator;
        typedef T&             reference;
        typedef const T&       const_reference;
        typedef std::size_t    size_type;
        typedef std::ptrdiff_t difference_type;

        // iterator support
        inline iterator begin() { return elems; }
        inline const_iterator begin() const { return elems; }
        inline iterator end() { return elems+N; }
        inline const_iterator end() const { return elems+N; }

        // reverse iterator support
#if !defined(BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION) && !defined(BOOST_MSVC_STD_ITERATOR) && !defined(BOOST_NO_STD_ITERATOR_TRAITS)
        typedef std::reverse_iterator<iterator> reverse_iterator;
        typedef std::reverse_iterator<const_iterator> const_reverse_iterator;
#elif defined(_MSC_VER) && (_MSC_VER == 1300) && defined(BOOST_DINKUMWARE_STDLIB) && (BOOST_DINKUMWARE_STDLIB == 310)
        // workaround for broken reverse_iterator in VC7
        typedef std::reverse_iterator<std::_Ptrit<value_type, difference_type, iterator,
                                      reference, iterator, reference> > reverse_iterator;
        typedef std::reverse_iterator<std::_Ptrit<value_type, difference_type, const_iterator,
                                      const_reference, iterator, reference> > const_reverse_iterator;
#else
        // workaround for broken reverse_iterator implementations
        typedef std::reverse_iterator<iterator,T> reverse_iterator;
        typedef std::reverse_iterator<const_iterator,T> const_reverse_iterator;
#endif

        reverse_iterator rbegin() { return reverse_iterator(end()); }
        const_reverse_iterator rbegin() const {
            return const_reverse_iterator(end());
        }
        reverse_iterator rend() { return reverse_iterator(begin()); }
        const_reverse_iterator rend() const {
            return const_reverse_iterator(begin());
        }

        // operator[]
        inline reference operator[](size_type i) { return elems[i]; }
        inline const_reference operator[](size_type i) const { return elems[i]; }

        // at() with range check
        reference at(size_type i) { rangecheck(i); return elems[i]; }
        const_reference at(size_type i) const { rangecheck(i); return elems[i]; }

        // front() and back()
        reference front() { return elems[0]; }
        const_reference front() const { return elems[0]; }
        reference back() { return elems[N-1]; }
        const_reference back() const { return elems[N-1]; }

        // size is constant
        static inline size_type size() { return N; }
        static bool empty() { return false; }
        static size_type max_size() { return N; }
        enum { static_size = N };

		/** This method has no effects in this class, but raises an exception if the expected size does not match */
		inline void resize(const size_t nElements) {
			if (nElements!=N)
				throw std::logic_error(format("Try to change the size of a %u-CArray to %u.",static_cast<unsigned>(N),static_cast<unsigned>(nElements)));
		}

        // swap (note: linear complexity in N, constant for given instantiation)
        void swap (CArray<T,N>& y) {
            std::swap_ranges(begin(),end(),y.begin());
        }

        // direct access to data (read-only)
        const T* data() const { return elems; }

        // use array as C array (direct read/write access to data)
        T* data() { return elems; }

        // assignment with type conversion
        template <typename T2>
        CArray<T,N>& operator= (const CArray<T2,N>& rhs) {
            std::copy(rhs.begin(),rhs.end(), begin());
            return *this;
        }

        // assign one value to all elements
        inline void assign (const T& value)
        {
			for (size_t i=0;i<N;i++) elems[i]=value;
        }
        // assign (compatible with std::vector's one) (by JLBC for MRPT)
        void assign (const size_t n, const T& value)
        {
        	ASSERTDEB_(N==n);
			for (size_t i=0;i<N;i++) elems[i]=value;
        }

		//assign a range of values corresponding to a pair of iterators (by PMO for MRPT)
		template<typename I> void assign(I b,const I &e)	{
			ASSERTDEB_(std::distance(b,e)==N);
			for (iterator i=begin();i<end();++i) *i=*(b++);
		}

      private:
        // check range (may be private because it is static)
        static void rangecheck (size_type i) {
            if (i >= size()) {
                throw std::out_of_range("CArray<>: index out of range");
            }
        }

    };

// partial specialization for arrays of size 0
    template <typename T>
    class CArray<T,0> {
      public:
        char c;  // to ensure different array intances return unique values for begin/end

      public:
        // type definitions
        typedef T              value_type;
        typedef T*             iterator;
        typedef const T*       const_iterator;
        typedef T&             reference;
        typedef const T&       const_reference;
        typedef std::size_t    size_type;
        typedef std::ptrdiff_t difference_type;

        // iterator support
        iterator begin() { return reinterpret_cast< iterator >( &c ); }
        const_iterator begin() const { return reinterpret_cast< const_iterator >( &c ); }
        iterator end() { return reinterpret_cast< iterator >( &c ); }
        const_iterator end() const { return reinterpret_cast< const_iterator >( &c ); }

        // reverse iterator support
#if !defined(BOOST_MSVC_STD_ITERATOR) && !defined(BOOST_NO_STD_ITERATOR_TRAITS)
        typedef std::reverse_iterator<iterator> reverse_iterator;
        typedef std::reverse_iterator<const_iterator> const_reverse_iterator;
#elif defined(_MSC_VER) && (_MSC_VER == 1300) && defined(BOOST_DINKUMWARE_STDLIB) && (BOOST_DINKUMWARE_STDLIB == 310)
        // workaround for broken reverse_iterator in VC7
        typedef std::reverse_iterator<std::_Ptrit<value_type, difference_type, iterator,
                                      reference, iterator, reference> > reverse_iterator;
        typedef std::reverse_iterator<std::_Ptrit<value_type, difference_type, const_iterator,
                                      const_reference, iterator, reference> > const_reverse_iterator;
#else
        // workaround for broken reverse_iterator implementations
        typedef std::reverse_iterator<iterator,T> reverse_iterator;
        typedef std::reverse_iterator<const_iterator,T> const_reverse_iterator;
#endif

        reverse_iterator rbegin() { return reverse_iterator(end()); }
        const_reverse_iterator rbegin() const {
            return const_reverse_iterator(end());
        }
        reverse_iterator rend() { return reverse_iterator(begin()); }
        const_reverse_iterator rend() const {
            return const_reverse_iterator(begin());
        }

        // at() with range check
        reference at(size_type i) {
            throw std::out_of_range("CArray<0>: index out of range");
        }
        const_reference at(size_type i) const {
            throw std::out_of_range("<0>: index out of range");
        }

        // size is constant
        static size_type size() { return 0; }
        static bool empty() { return true; }
        static size_type max_size() { return 0; }
        enum { static_size = 0 };

        // swap
        void swap (CArray<T,0>& y) {
            //  could swap value of c, but value is not part of documented array state
        }

        // direct access to data
        const T* data() const { return NULL; }
        T* data() { return NULL; }

        // assignment with type conversion
        template < typename T2 >
        CArray< T,0 >& operator= (const CArray< T2, 0>& rhs) {
            return *this;
        }

        //  Calling these operations are undefined behaviour for 0-size arrays,
        //  but Library TR1 requires their presence.
        // operator[]
        inline reference operator[](size_type i) { makes_no_sense(); static T dumm=0; return dumm; }
        inline const_reference operator[](size_type i) const { makes_no_sense(); static T dumm=0; return dumm; }

        // front() and back()
        reference front() { makes_no_sense(); }
        const_reference front() const { makes_no_sense(); }
        reference back() { makes_no_sense(); }
        const_reference back() const { makes_no_sense(); }

      private:
        // helper for operations that have undefined behaviour for 0-size arrays,
        //  assert( false ); added to make lack of support clear
        static void makes_no_sense () {
            //assert(true);
            throw std::out_of_range("CArray<0>: index out of range");
        }
    };

    // comparisons
    template<class T, std::size_t N>
    bool operator== (const CArray<T,N>& x, const CArray<T,N>& y) {
        return std::equal(x.begin(), x.end(), y.begin());
    }
    template<class T, std::size_t N>
    bool operator< (const CArray<T,N>& x, const CArray<T,N>& y) {
        return std::lexicographical_compare(x.begin(),x.end(),y.begin(),y.end());
    }
    template<class T, std::size_t N>
    bool operator!= (const CArray<T,N>& x, const CArray<T,N>& y) {
        return !(x==y);
    }
    template<class T, std::size_t N>
    bool operator> (const CArray<T,N>& x, const CArray<T,N>& y) {
        return y<x;
    }
    template<class T, std::size_t N>
    bool operator<= (const CArray<T,N>& x, const CArray<T,N>& y) {
        return !(y<x);
    }
    template<class T, std::size_t N>
    bool operator>= (const CArray<T,N>& x, const CArray<T,N>& y) {
        return !(x<y);
    }




	// ----------------  CArrayNumeric -------------------------

    /** CArrayNumeric is an array for numeric types supporting several mathematical operations (actually, just a wrapper on Eigen::Matrix<T,N,1>)
      * \sa CArrayFloat, CArrayDouble, CArray
      */
    template <typename T, std::size_t N>
    class CArrayNumeric : public Eigen::Matrix<T,N,1>
    {
	public:
		typedef Eigen::Matrix<T,N,1> Base;

    	CArrayNumeric() {}  //!< Default constructor
    	/** Constructor from initial values ptr[0]-ptr[N-1] */
    	CArrayNumeric(const T*ptr) : Eigen::Matrix<T,N,1>(ptr) {}

		MRPT_MATRIX_CONSTRUCTORS_FROM_POSES(CArrayNumeric)

		/** Initialization from a vector-like source, that is, anything implementing operator[]. */
    	template <class ARRAYLIKE>
    	explicit CArrayNumeric(const ARRAYLIKE &obj) : Eigen::Matrix<T,N,1>(obj) {}

		template<typename OtherDerived>
		inline CArrayNumeric<T,N> & operator= (const Eigen::MatrixBase <OtherDerived>& other) {
			Base::operator=(other);
			return *this;
		}

    };

	// --------------  Partial specializations of CArrayNumeric -----------

    /** A partial specialization of CArrayNumeric for float numbers.
      * \sa CArrayNumeric, CArray */
    template <std::size_t N>
    class CArrayFloat : public CArrayNumeric<float,N>
    {
	public:
		typedef CArrayNumeric<float,N> Base;
		typedef CArrayFloat<N> mrpt_autotype;

    	CArrayFloat() {}  //!< Default constructor
    	CArrayFloat(const float*ptr) : CArrayNumeric<float,N>(ptr) {} //!< Constructor from initial values ptr[0]-ptr[N-1]

		MRPT_MATRIX_CONSTRUCTORS_FROM_POSES(CArrayFloat)

		/** Initialization from a vector-like source, that is, anything implementing operator[]. */
    	template <class ARRAYLIKE>
    	explicit CArrayFloat(const ARRAYLIKE &obj) : CArrayNumeric<float,N>(obj) {}
		MRPT_EIGEN_DERIVED_CLASS_CTOR_OPERATOR_EQUAL(CArrayFloat) // Implements ctor and "operator =" for any other Eigen class
    };

    /** A partial specialization of CArrayNumeric for double numbers.
      * \sa CArrayNumeric, CArray */
    template <std::size_t N>
    class CArrayDouble : public CArrayNumeric<double,N>
    {
	public:
		typedef CArrayNumeric<double,N> Base;
		typedef CArrayDouble<N> mrpt_autotype;

    	CArrayDouble() {}  //!< Default constructor
    	CArrayDouble(const double*ptr) : CArrayNumeric<double,N>(ptr) {} //!< Constructor from initial values ptr[0]-ptr[N-1]

		MRPT_MATRIX_CONSTRUCTORS_FROM_POSES(CArrayDouble)

		/** Initialization from a vector-like source, that is, anything implementing operator[]. */
    	template <class ARRAYLIKE>
    	explicit CArrayDouble(const ARRAYLIKE &obj) : CArrayNumeric<double,N>(obj) {}
		MRPT_EIGEN_DERIVED_CLASS_CTOR_OPERATOR_EQUAL(CArrayDouble) // Implements ctor and "operator =" for any other Eigen class
    };

    /** A partial specialization of CArrayNumeric for int numbers.
      * \sa CArrayNumeric, CArray */
    template <std::size_t N>
    class CArrayInt : public CArrayNumeric<int,N>
    {
	public:
		typedef CArrayNumeric<int,N> Base;
		typedef CArrayInt<N> mrpt_autotype;

    	CArrayInt() {}  //!< Default constructor
    	CArrayInt(const int*ptr) : CArrayNumeric<int,N>(ptr) {} //!< Constructor from initial values ptr[0]-ptr[N-1]
		MRPT_EIGEN_DERIVED_CLASS_CTOR_OPERATOR_EQUAL(CArrayInt) // Implements ctor and "operator =" for any other Eigen class
    };

    /** A partial specialization of CArrayNumeric for unsigned int numbers.
      * \sa CArrayNumeric, CArray */
    template <std::size_t N>
    class CArrayUInt : public CArrayNumeric<unsigned int,N>
    {
	public:
		typedef CArrayNumeric<unsigned int,N> Base;
		typedef CArrayUInt<N> mrpt_autotype;

    	CArrayUInt() {}  //!< Default constructor
    	CArrayUInt(const unsigned int*ptr) : CArrayNumeric<unsigned int,N>(ptr) {} //!< Constructor from initial values ptr[0]-ptr[N-1]
		MRPT_EIGEN_DERIVED_CLASS_CTOR_OPERATOR_EQUAL(CArrayUInt) // Implements ctor and "operator =" for any other Eigen class
    };

	/** Auxiliary class used in CMatrixTemplate:size(), CMatrixTemplate::resize(), CMatrixFixedNumeric::size(), CMatrixFixedNumeric::resize(), to mimic the behavior of STL-containers */
	struct CMatrixTemplateSize : public Eigen::Matrix<size_t,2,1>
	{
		typedef Eigen::Matrix<size_t,2,1> Base;
		typedef CMatrixTemplateSize mrpt_autotype;

		inline CMatrixTemplateSize() : Eigen::Matrix<size_t,2,1>() {}
		inline CMatrixTemplateSize(const size_t *d) : Eigen::Matrix<size_t,2,1>(d) {}

		inline bool operator==(const CMatrixTemplateSize&o) const { return Eigen::Matrix<size_t,2,1>::operator()(0)==o[0] && Eigen::Matrix<size_t,2,1>::operator()(1)==o[1]; }
		inline bool operator!=(const CMatrixTemplateSize&o) const { return !(*this==o); }
		/** This operator allows the size(N,M) to be compared with a plain size_t N*M  */
		inline operator size_t(void) const { return Eigen::Matrix<size_t,2,1>::operator()(0)*Eigen::Matrix<size_t,2,1>::operator()(1); }
		MRPT_EIGEN_DERIVED_CLASS_CTOR_OPERATOR_EQUAL(CMatrixTemplateSize) // Implements ctor and "operator =" for any other Eigen class
	};

} // End of namespace

	namespace utils
	{
		// Extensions to mrpt::utils::TTypeName for matrices:
		template<typename T,size_t N> struct TTypeName <mrpt::math::CArrayNumeric<T,N> > {
			static std::string get() { return mrpt::format("CArrayNumeric<%s,%u>",TTypeName<T>::get().c_str(),static_cast<unsigned int>(N)); } };
		template<size_t N> struct TTypeName <mrpt::math::CArrayDouble<N> > {
			static std::string get() { return mrpt::format("CArrayNumeric<double,%u>",static_cast<unsigned int>(N)); } };
		template<size_t N> struct TTypeName <mrpt::math::CArrayFloat<N> > {
			static std::string get() { return mrpt::format("CArrayNumeric<float,%u>",static_cast<unsigned int>(N)); } };
	}


} // End of namespace


#endif

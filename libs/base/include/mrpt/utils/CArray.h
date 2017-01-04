/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/core_defs.h>
#include <stdexcept>

namespace mrpt
{
namespace utils
{
	// ----------------  CArray -------------------------
	/** A STL container (as wrapper) for arrays of constant size defined at compile time.
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
     * \sa mrpt::math::CArrayNumeric (for another, non-related base template class that DOES support maths)
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
        #ifdef _DEBUG
            if (N!=n) throw std::out_of_range("CArray<>: assign() of incorrect length");
        #endif
			for (size_t i=0;i<N;i++) elems[i]=value;
        }

		//assign a range of values corresponding to a pair of iterators (by PMO for MRPT)
		template<typename I> void assign(I b,const I &e)	{
        #ifdef _DEBUG
            if (std::distance(b,e)!=N) throw std::out_of_range("CArray<>: assign() of incorrect length");
        #endif
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
        iterator end() { return reinterpret_cast< iterator >( &c ); } //-V524
        const_iterator end() const { return reinterpret_cast< const_iterator >( &c ); } //-V524

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
			MRPT_UNUSED_PARAM(i);
            throw std::out_of_range("CArray<0>: index out of range");
        }
        const_reference at(size_type i) const {
			MRPT_UNUSED_PARAM(i);
            throw std::out_of_range("<0>: index out of range");
        }

        // size is constant
        static size_type size() { return 0; }
        static bool empty() { return true; }
        static size_type max_size() { return 0; }
        enum { static_size = 0 };

        // swap
        void swap (CArray<T,0>& y) {
			MRPT_UNUSED_PARAM(y);
            //  could swap value of c, but value is not part of documented array state
        }

        // direct access to data
        const T* data() const { return NULL; }
        T* data() { return NULL; }

        // assignment with type conversion
        template < typename T2 >
        CArray< T,0 >& operator= (const CArray< T2, 0>& rhs) {
			MRPT_UNUSED_PARAM(rhs);
            return *this;
        }

        //  Calling these operations are undefined behaviour for 0-size arrays,
        //  but Library TR1 requires their presence.
        // operator[]
        inline reference operator[](size_type ) { makes_no_sense(); static T dumm=0; return dumm; }
        inline const_reference operator[](size_type ) const { makes_no_sense(); static T dumm=0; return dumm; }

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

} // End of namespace
} // End of namespace

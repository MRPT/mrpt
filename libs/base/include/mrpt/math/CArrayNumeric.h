/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef _MRPT_CArrayNumeric_H
#define _MRPT_CArrayNumeric_H

#include <mrpt/utils/core_defs.h>
#include <mrpt/utils/types_math.h>  // Eigen
#include <mrpt/utils/TTypeName.h>
#include <mrpt/math/point_poses2vectors.h> // MRPT_MATRIX_CONSTRUCTORS_FROM_POSES()

namespace mrpt
{
namespace math
{
    /** CArrayNumeric is an array for numeric types supporting several mathematical operations (actually, just a wrapper on Eigen::Matrix<T,N,1>)
      * \sa CArrayFloat, CArrayDouble, CArray
      */
    template <typename T, std::size_t N>
    class CArrayNumeric : public Eigen::Matrix<T,N,1>
    {
	public:
        typedef T                    value_type;
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

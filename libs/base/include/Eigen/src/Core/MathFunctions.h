// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2006-2010 Benoit Jacob <jacob.benoit.1@gmail.com>
//
// Eigen is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Alternatively, you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// Eigen is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License and a copy of the GNU General Public License along with
// Eigen. If not, see <http://www.gnu.org/licenses/>.

#ifndef EIGEN_MATHFUNCTIONS_H
#define EIGEN_MATHFUNCTIONS_H

namespace internal {

/** \internal \struct global_math_functions_filtering_base
  *
  * What it does:
  * Defines a typedef 'type' as follows:
  * - if type T has a member typedef Eigen_BaseClassForSpecializationOfGlobalMathFuncImpl, then
  *   global_math_functions_filtering_base<T>::type is a typedef for it.
  * - otherwise, global_math_functions_filtering_base<T>::type is a typedef for T.
  *
  * How it's used:
  * To allow to defined the global math functions (like sin...) in certain cases, like the Array expressions.
  * When you do sin(array1+array2), the object array1+array2 has a complicated expression type, all what you want to know
  * is that it inherits ArrayBase. So we implement a partial specialization of sin_impl for ArrayBase<Derived>.
  * So we must make sure to use sin_impl<ArrayBase<Derived> > and not sin_impl<Derived>, otherwise our partial specialization
  * won't be used. How does sin know that? That's exactly what global_math_functions_filtering_base tells it.
  *
  * How it's implemented:
  * SFINAE in the style of enable_if. Highly susceptible of breaking compilers. With GCC, it sure does work, but if you replace
  * the typename dummy by an integer template parameter, it doesn't work anymore!
  */

template<typename T, typename dummy = void>
struct global_math_functions_filtering_base
{
  typedef T type;
};

template<typename T> struct always_void { typedef void type; };

template<typename T>
struct global_math_functions_filtering_base
  <T,
   typename always_void<typename T::Eigen_BaseClassForSpecializationOfGlobalMathFuncImpl>::type
  >
{
  typedef typename T::Eigen_BaseClassForSpecializationOfGlobalMathFuncImpl type;
};

#define EIGEN_MATHFUNC_IMPL(func, scalar) func##_impl<typename global_math_functions_filtering_base<scalar>::type>
#define EIGEN_MATHFUNC_RETVAL(func, scalar) typename func##_retval<typename global_math_functions_filtering_base<scalar>::type>::type


/****************************************************************************
* Implementation of real                                                 *
****************************************************************************/

template<typename Scalar>
struct real_impl
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  static inline RealScalar run(const Scalar& x)
  {
    return x;
  }
};

template<typename RealScalar>
struct real_impl<std::complex<RealScalar> >
{
  static inline RealScalar run(const std::complex<RealScalar>& x)
  {
    return std::real(x);
  }
};

template<typename Scalar>
struct real_retval
{
  typedef typename NumTraits<Scalar>::Real type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(real, Scalar) real(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(real, Scalar)::run(x);
}

/****************************************************************************
* Implementation of imag                                                 *
****************************************************************************/

template<typename Scalar>
struct imag_impl
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  static inline RealScalar run(const Scalar&)
  {
    return RealScalar(0);
  }
};

template<typename RealScalar>
struct imag_impl<std::complex<RealScalar> >
{
  static inline RealScalar run(const std::complex<RealScalar>& x)
  {
    return std::imag(x);
  }
};

template<typename Scalar>
struct imag_retval
{
  typedef typename NumTraits<Scalar>::Real type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(imag, Scalar) imag(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(imag, Scalar)::run(x);
}

/****************************************************************************
* Implementation of real_ref                                             *
****************************************************************************/

template<typename Scalar>
struct real_ref_impl
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  static inline RealScalar& run(Scalar& x)
  {
    return reinterpret_cast<RealScalar*>(&x)[0];
  }
  static inline const RealScalar& run(const Scalar& x)
  {
    return reinterpret_cast<const RealScalar*>(&x)[0];
  }
};

template<typename Scalar>
struct real_ref_retval
{
  typedef typename NumTraits<Scalar>::Real & type;
};

template<typename Scalar>
inline typename add_const< EIGEN_MATHFUNC_RETVAL(real_ref, Scalar) >::type real_ref(const Scalar& x)
{
  return real_ref_impl<Scalar>::run(x);
}

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(real_ref, Scalar) real_ref(Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(real_ref, Scalar)::run(x);
}

/****************************************************************************
* Implementation of imag_ref                                             *
****************************************************************************/

template<typename Scalar, bool IsComplex>
struct imag_ref_default_impl
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  static inline RealScalar& run(Scalar& x)
  {
    return reinterpret_cast<RealScalar*>(&x)[1];
  }
  static inline const RealScalar& run(const Scalar& x)
  {
    return reinterpret_cast<RealScalar*>(&x)[1];
  }
};

template<typename Scalar>
struct imag_ref_default_impl<Scalar, false>
{
  static inline Scalar run(Scalar&)
  {
    return Scalar(0);
  }
  static inline const Scalar run(const Scalar&)
  {
    return Scalar(0);
  }
};

template<typename Scalar>
struct imag_ref_impl : imag_ref_default_impl<Scalar, NumTraits<Scalar>::IsComplex> {};

template<typename Scalar>
struct imag_ref_retval
{
  typedef typename NumTraits<Scalar>::Real & type;
};

template<typename Scalar>
inline typename add_const< EIGEN_MATHFUNC_RETVAL(imag_ref, Scalar) >::type imag_ref(const Scalar& x)
{
  return imag_ref_impl<Scalar>::run(x);
}

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(imag_ref, Scalar) imag_ref(Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(imag_ref, Scalar)::run(x);
}

/****************************************************************************
* Implementation of conj                                                 *
****************************************************************************/

template<typename Scalar>
struct conj_impl
{
  static inline Scalar run(const Scalar& x)
  {
    return x;
  }
};

template<typename RealScalar>
struct conj_impl<std::complex<RealScalar> >
{
  static inline std::complex<RealScalar> run(const std::complex<RealScalar>& x)
  {
    return std::conj(x);
  }
};

template<typename Scalar>
struct conj_retval
{
  typedef Scalar type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(conj, Scalar) conj(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(conj, Scalar)::run(x);
}

/****************************************************************************
* Implementation of abs                                                  *
****************************************************************************/

template<typename Scalar>
struct abs_impl
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  static inline RealScalar run(const Scalar& x)
  {
    return std::abs(x);
  }
};

template<typename Scalar>
struct abs_retval
{
  typedef typename NumTraits<Scalar>::Real type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(abs, Scalar) abs(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(abs, Scalar)::run(x);
}

/****************************************************************************
* Implementation of abs2                                                 *
****************************************************************************/

template<typename Scalar>
struct abs2_impl
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  static inline RealScalar run(const Scalar& x)
  {
    return x*x;
  }
};

template<typename RealScalar>
struct abs2_impl<std::complex<RealScalar> >
{
  static inline RealScalar run(const std::complex<RealScalar>& x)
  {
    return std::norm(x);
  }
};

template<typename Scalar>
struct abs2_retval
{
  typedef typename NumTraits<Scalar>::Real type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(abs2, Scalar) abs2(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(abs2, Scalar)::run(x);
}

/****************************************************************************
* Implementation of norm1                                                *
****************************************************************************/

template<typename Scalar, bool IsComplex>
struct norm1_default_impl
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  static inline RealScalar run(const Scalar& x)
  {
    return abs(real(x)) + abs(imag(x));
  }
};

template<typename Scalar>
struct norm1_default_impl<Scalar, false>
{
  static inline Scalar run(const Scalar& x)
  {
    return abs(x);
  }
};

template<typename Scalar>
struct norm1_impl : norm1_default_impl<Scalar, NumTraits<Scalar>::IsComplex> {};

template<typename Scalar>
struct norm1_retval
{
  typedef typename NumTraits<Scalar>::Real type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(norm1, Scalar) norm1(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(norm1, Scalar)::run(x);
}

/****************************************************************************
* Implementation of hypot                                                *
****************************************************************************/

template<typename Scalar>
struct hypot_impl
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  static inline RealScalar run(const Scalar& x, const Scalar& y)
  {
    RealScalar _x = abs(x);
    RealScalar _y = abs(y);
    RealScalar p = std::max(_x, _y);
    RealScalar q = std::min(_x, _y);
    RealScalar qp = q/p;
    return p * sqrt(RealScalar(1) + qp*qp);
  }
};

template<typename Scalar>
struct hypot_retval
{
  typedef typename NumTraits<Scalar>::Real type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(hypot, Scalar) hypot(const Scalar& x, const Scalar& y)
{
  return EIGEN_MATHFUNC_IMPL(hypot, Scalar)::run(x, y);
}

/****************************************************************************
* Implementation of cast                                                 *
****************************************************************************/

template<typename OldType, typename NewType>
struct cast_impl
{
  static inline NewType run(const OldType& x)
  {
    return static_cast<NewType>(x);
  }
};

// here, for once, we're plainly returning NewType: we don't want cast to do weird things.

template<typename OldType, typename NewType>
inline NewType cast(const OldType& x)
{
  return cast_impl<OldType, NewType>::run(x);
}

/****************************************************************************
* Implementation of sqrt                                                 *
****************************************************************************/

template<typename Scalar, bool IsInteger>
struct sqrt_default_impl
{
  static inline Scalar run(const Scalar& x)
  {
    return std::sqrt(x);
  }
};

template<typename Scalar>
struct sqrt_default_impl<Scalar, true>
{
  static inline Scalar run(const Scalar&)
  {
    EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar)
    return Scalar(0);
  }
};

template<typename Scalar>
struct sqrt_impl : sqrt_default_impl<Scalar, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar>
struct sqrt_retval
{
  typedef Scalar type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(sqrt, Scalar) sqrt(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(sqrt, Scalar)::run(x);
}

/****************************************************************************
* Implementation of exp                                                  *
****************************************************************************/

template<typename Scalar, bool IsInteger>
struct exp_default_impl
{
  static inline Scalar run(const Scalar& x)
  {
    return std::exp(x);
  }
};

template<typename Scalar>
struct exp_default_impl<Scalar, true>
{
  static inline Scalar run(const Scalar&)
  {
    EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar)
    return Scalar(0);
  }
};

template<typename Scalar>
struct exp_impl : exp_default_impl<Scalar, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar>
struct exp_retval
{
  typedef Scalar type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(exp, Scalar) exp(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(exp, Scalar)::run(x);
}

/****************************************************************************
* Implementation of cos                                                  *
****************************************************************************/

template<typename Scalar, bool IsInteger>
struct cos_default_impl
{
  static inline Scalar run(const Scalar& x)
  {
    return std::cos(x);
  }
};

template<typename Scalar>
struct cos_default_impl<Scalar, true>
{
  static inline Scalar run(const Scalar&)
  {
    EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar)
    return Scalar(0);
  }
};

template<typename Scalar>
struct cos_impl : cos_default_impl<Scalar, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar>
struct cos_retval
{
  typedef Scalar type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(cos, Scalar) cos(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(cos, Scalar)::run(x);
}

/****************************************************************************
* Implementation of sin                                                  *
****************************************************************************/

template<typename Scalar, bool IsInteger>
struct sin_default_impl
{
  static inline Scalar run(const Scalar& x)
  {
    return std::sin(x);
  }
};

template<typename Scalar>
struct sin_default_impl<Scalar, true>
{
  static inline Scalar run(const Scalar&)
  {
    EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar)
    return Scalar(0);
  }
};

template<typename Scalar>
struct sin_impl : sin_default_impl<Scalar, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar>
struct sin_retval
{
  typedef Scalar type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(sin, Scalar) sin(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(sin, Scalar)::run(x);
}

/****************************************************************************
* Implementation of log                                                  *
****************************************************************************/

template<typename Scalar, bool IsInteger>
struct log_default_impl
{
  static inline Scalar run(const Scalar& x)
  {
    return std::log(x);
  }
};

template<typename Scalar>
struct log_default_impl<Scalar, true>
{
  static inline Scalar run(const Scalar&)
  {
    EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar)
    return Scalar(0);
  }
};

template<typename Scalar>
struct log_impl : log_default_impl<Scalar, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar>
struct log_retval
{
  typedef Scalar type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(log, Scalar) log(const Scalar& x)
{
  return EIGEN_MATHFUNC_IMPL(log, Scalar)::run(x);
}

/****************************************************************************
* Implementation of atan2                                                *
****************************************************************************/

template<typename Scalar, bool IsInteger>
struct atan2_default_impl
{
  typedef Scalar retval;
  static inline Scalar run(const Scalar& x, const Scalar& y)
  {
    return std::atan2(x, y);
  }
};

template<typename Scalar>
struct atan2_default_impl<Scalar, true>
{
  static inline Scalar run(const Scalar&, const Scalar&)
  {
    EIGEN_STATIC_ASSERT_NON_INTEGER(Scalar)
    return Scalar(0);
  }
};

template<typename Scalar>
struct atan2_impl : atan2_default_impl<Scalar, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar>
struct atan2_retval
{
  typedef Scalar type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(atan2, Scalar) atan2(const Scalar& x, const Scalar& y)
{
  return EIGEN_MATHFUNC_IMPL(atan2, Scalar)::run(x, y);
}

/****************************************************************************
* Implementation of pow                                                  *
****************************************************************************/

template<typename Scalar, bool IsInteger>
struct pow_default_impl
{
  typedef Scalar retval;
  static inline Scalar run(const Scalar& x, const Scalar& y)
  {
    return std::pow(x, y);
  }
};

template<typename Scalar>
struct pow_default_impl<Scalar, true>
{
  static inline Scalar run(Scalar x, Scalar y)
  {
    Scalar res = 1;
    eigen_assert(!NumTraits<Scalar>::IsSigned || y >= 0);
    if(y & 1) res *= x;
    y >>= 1;
    while(y)
    {
      x *= x;
      if(y&1) res *= x;
      y >>= 1;
    }
    return res;
  }
};

template<typename Scalar>
struct pow_impl : pow_default_impl<Scalar, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar>
struct pow_retval
{
  typedef Scalar type;
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(pow, Scalar) pow(const Scalar& x, const Scalar& y)
{
  return EIGEN_MATHFUNC_IMPL(pow, Scalar)::run(x, y);
}

/****************************************************************************
* Implementation of random                                               *
****************************************************************************/

template<typename Scalar,
         bool IsComplex,
         bool IsInteger>
struct random_default_impl {};

template<typename Scalar>
struct random_impl : random_default_impl<Scalar, NumTraits<Scalar>::IsComplex, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar>
struct random_retval
{
  typedef Scalar type;
};

template<typename Scalar> inline EIGEN_MATHFUNC_RETVAL(random, Scalar) random(const Scalar& x, const Scalar& y);
template<typename Scalar> inline EIGEN_MATHFUNC_RETVAL(random, Scalar) random();

template<typename Scalar>
struct random_default_impl<Scalar, false, false>
{
  static inline Scalar run(const Scalar& x, const Scalar& y)
  {
    return x + (y-x) * Scalar(std::rand()) / float(RAND_MAX);
  }
  static inline Scalar run()
  {
    return run(Scalar(NumTraits<Scalar>::IsSigned ? -1 : 0), Scalar(1));
  }
};

template<typename Scalar>
struct random_default_impl<Scalar, false, true>
{
  static inline Scalar run(const Scalar& x, const Scalar& y)
  {
    return x + Scalar((y-x+1) * (std::rand() / (RAND_MAX + typename NumTraits<Scalar>::NonInteger(1))));
  }
  static inline Scalar run()
  {
    return run(Scalar(NumTraits<Scalar>::IsSigned ? -10 : 0), Scalar(10));
  }
};

template<typename Scalar>
struct random_default_impl<Scalar, true, false>
{
  static inline Scalar run(const Scalar& x, const Scalar& y)
  {
    return Scalar(random(real(x), real(y)),
                  random(imag(x), imag(y)));
  }
  static inline Scalar run()
  {
    typedef typename NumTraits<Scalar>::Real RealScalar;
    return Scalar(random<RealScalar>(), random<RealScalar>());
  }
};

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(random, Scalar) random(const Scalar& x, const Scalar& y)
{
  return EIGEN_MATHFUNC_IMPL(random, Scalar)::run(x, y);
}

template<typename Scalar>
inline EIGEN_MATHFUNC_RETVAL(random, Scalar) random()
{
  return EIGEN_MATHFUNC_IMPL(random, Scalar)::run();
}

/****************************************************************************
* Implementation of fuzzy comparisons                                       *
****************************************************************************/

template<typename Scalar,
         bool IsComplex,
         bool IsInteger>
struct scalar_fuzzy_default_impl {};

template<typename Scalar>
struct scalar_fuzzy_default_impl<Scalar, false, false>
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  template<typename OtherScalar>
  static inline bool isMuchSmallerThan(const Scalar& x, const OtherScalar& y, const RealScalar& prec)
  {
    return abs(x) <= abs(y) * prec;
  }
  static inline bool isApprox(const Scalar& x, const Scalar& y, const RealScalar& prec)
  {
    return abs(x - y) <= std::min(abs(x), abs(y)) * prec;
  }
  static inline bool isApproxOrLessThan(const Scalar& x, const Scalar& y, const RealScalar& prec)
  {
    return x <= y || isApprox(x, y, prec);
  }
};

template<typename Scalar>
struct scalar_fuzzy_default_impl<Scalar, false, true>
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  template<typename OtherScalar>
  static inline bool isMuchSmallerThan(const Scalar& x, const Scalar&, const RealScalar&)
  {
    return x == Scalar(0);
  }
  static inline bool isApprox(const Scalar& x, const Scalar& y, const RealScalar&)
  {
    return x == y;
  }
  static inline bool isApproxOrLessThan(const Scalar& x, const Scalar& y, const RealScalar&)
  {
    return x <= y;
  }
};

template<typename Scalar>
struct scalar_fuzzy_default_impl<Scalar, true, false>
{
  typedef typename NumTraits<Scalar>::Real RealScalar;
  template<typename OtherScalar>
  static inline bool isMuchSmallerThan(const Scalar& x, const OtherScalar& y, const RealScalar& prec)
  {
    return abs2(x) <= abs2(y) * prec * prec;
  }
  static inline bool isApprox(const Scalar& x, const Scalar& y, const RealScalar& prec)
  {
    return abs2(x - y) <= std::min(abs2(x), abs2(y)) * prec * prec;
  }
};

template<typename Scalar>
struct scalar_fuzzy_impl : scalar_fuzzy_default_impl<Scalar, NumTraits<Scalar>::IsComplex, NumTraits<Scalar>::IsInteger> {};

template<typename Scalar, typename OtherScalar>
inline bool isMuchSmallerThan(const Scalar& x, const OtherScalar& y,
                                   typename NumTraits<Scalar>::Real precision = NumTraits<Scalar>::dummy_precision())
{
  return scalar_fuzzy_impl<Scalar>::template isMuchSmallerThan<OtherScalar>(x, y, precision);
}

template<typename Scalar>
inline bool isApprox(const Scalar& x, const Scalar& y,
                          typename NumTraits<Scalar>::Real precision = NumTraits<Scalar>::dummy_precision())
{
  return scalar_fuzzy_impl<Scalar>::isApprox(x, y, precision);
}

template<typename Scalar>
inline bool isApproxOrLessThan(const Scalar& x, const Scalar& y,
                                    typename NumTraits<Scalar>::Real precision = NumTraits<Scalar>::dummy_precision())
{
  return scalar_fuzzy_impl<Scalar>::isApproxOrLessThan(x, y, precision);
}

/******************************************
***  The special case of the  bool type ***
******************************************/

template<> struct random_impl<bool>
{
  static inline bool run()
  {
    return random<int>(0,1)==0 ? false : true;
  }
};

template<> struct scalar_fuzzy_impl<bool>
{
  static inline bool isApprox(bool x, bool y, bool)
  {
    return x == y;
  }
};

} // end namespace internal

#endif // EIGEN_MATHFUNCTIONS_H

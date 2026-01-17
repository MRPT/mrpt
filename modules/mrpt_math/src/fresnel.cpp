/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/math/fresnel.h>

#include <cfloat>
#include <cmath>

// Based on code (public domain) in http://www.mymathlib.com/functions/

namespace
{
// Constants
constexpr long double kSqrt2OverPi = 7.978845608028653558798921198687637369517e-1L;
constexpr long double kSqrt2Pi = 2.506628274631000502415765284811045253006L;
constexpr int kNumAsymptoticTerms = 35;

// Forward declarations
long double powerSeriesS(long double x);
long double powerSeriesC(long double x);
long double fresnelAuxiliaryCosineIntegral(long double x);
long double fresnelAuxiliarySineIntegral(long double x);
long double chebyshevTnSeries(long double x, const long double coeffs[], int degree);

/**
 * @brief Evaluates Chebyshev polynomial series using Clenshaw's recursion algorithm
 *
 * This function computes p(x) = sum_{k=0}^{degree} coeffs[k] * T_k(x)
 * where T_k are Chebyshev polynomials of the first kind.
 *
 * @param x Point at which to evaluate the polynomial
 * @param coeffs Array of Chebyshev coefficients
 * @param degree Degree of the polynomial
 * @return Value of the polynomial at x
 */
long double chebyshevTnSeries(long double x, const long double coeffs[], int degree)
{
  if (degree < 0)
  {
    {
      return 0.0L;
    }
  }

  long double yPrev2 = 0.0L;
  long double yPrev1 = 0.0L;
  long double yCurrent = 0.0L;
  const long double twoX = x + x;

  // Apply Clenshaw's recursion, saving the last iteration
  for (int k = degree; k >= 1; k--)
  {
    yCurrent = twoX * yPrev1 - yPrev2 + coeffs[k];
    yPrev2 = yPrev1;
    yPrev1 = yCurrent;
  }

  // Final iteration
  return x * yPrev1 - yPrev2 + coeffs[0];
}

/**
 * @brief Computes power series representation of Fresnel sine integral
 *
 * For |x| < 0.5, uses the series:
 * S(x) = x^3 * sqrt(2/pi) * sum_{j=0}^inf [(-x^4)^j / ((4j+3)(2j+1)!)]
 *
 * @param x Argument of the Fresnel sine integral
 * @return Value of S(x)
 */
long double powerSeriesS(long double x)
{
  if (x == 0.0L)
  {
    return 0.0L;
  }

  const long double x2 = x * x;
  const long double x3 = x * x2;
  const long double x4 = -x2 * x2;

  long double xPower = 1.0L;
  long double seriesSum = 1.0L / 3.0L;
  long double prevSum = 0.0L;
  long double factorial = 1.0L;
  int j = 0;

  while (std::abs(seriesSum - prevSum) > LDBL_EPSILON * std::abs(prevSum))
  {
    prevSum = seriesSum;
    j++;
    factorial *= static_cast<long double>(2 * j);
    factorial *= static_cast<long double>(2 * j + 1);
    xPower *= x4;
    long double term = xPower / factorial;
    term /= static_cast<long double>(4 * j + 3);
    seriesSum += term;
  }

  return x3 * kSqrt2OverPi * seriesSum;
}

/**
 * @brief Computes power series representation of Fresnel cosine integral
 *
 * For |x| < 0.5, uses the series:
 * C(x) = x * sqrt(2/pi) * sum_{j=0}^inf [(-x^4)^j / ((4j+1)(2j)!)]
 *
 * @param x Argument of the Fresnel cosine integral
 * @return Value of C(x)
 */
long double powerSeriesC(long double x)
{
  if (x == 0.0L)
  {
    return 0.0L;
  }

  const long double x2 = x * x;
  const long double x4 = -x2 * x2;

  long double xPower = 1.0L;
  long double seriesSum = 1.0L;
  long double prevSum = 0.0L;
  long double factorial = 1.0L;
  int j = 0;

  while (std::abs(seriesSum - prevSum) > LDBL_EPSILON * std::abs(prevSum))
  {
    prevSum = seriesSum;
    j++;
    factorial *= static_cast<long double>(2 * j);
    factorial *= static_cast<long double>(2 * j - 1);
    xPower *= x4;
    long double term = xPower / factorial;
    term /= static_cast<long double>(4 * j + 1);
    seriesSum += term;
  }

  return x * kSqrt2OverPi * seriesSum;
}

// Chebyshev expansions for sine auxiliary integral
long double sineChebyshevExpansion0To1(long double x)
{
  static const long double coeffs[] = {
      +2.560134650043040830997e-1L,  -1.993005146464943284549e-1L,  +4.025503636721387266117e-2L,
      -4.459600454502960250729e-3L,  +6.447097305145147224459e-5L,  +7.544218493763717599380e-5L,
      -1.580422720690700333493e-5L,  +1.755845848573471891519e-6L,  -9.289769688468301734718e-8L,
      -5.624033192624251079833e-9L,  +1.854740406702369495830e-9L,  -2.174644768724492443378e-10L,
      +1.392899828133395918767e-11L, -6.989216003725983789869e-14L, -9.959396121060010838331e-14L,
      +1.312085140393647257714e-14L, -9.240470383522792593305e-16L, +2.472168944148817385152e-17L,
      +2.834615576069400293894e-18L, -4.650983461314449088349e-19L, +3.544083040732391556797e-20L};
  constexpr int degree = sizeof(coeffs) / sizeof(long double) - 1;
  constexpr long double midpoint = 0.5L;
  constexpr long double scale = 0.5L;

  return chebyshevTnSeries((x - midpoint) / scale, coeffs, degree);
}

long double sineChebyshevExpansion1To3(long double x)
{
  static const long double coeffs[] = {
      +3.470341566046115476477e-2L,  -3.855580521778624043304e-2L,  +1.420604309383996764083e-2L,
      -4.037349972538938202143e-3L,  +9.292478174580997778194e-4L,  -1.742730601244797978044e-4L,
      +2.563352976720387343201e-5L,  -2.498437524746606551732e-6L,  -1.334367201897140224779e-8L,
      +7.436854728157752667212e-8L,  -2.059620371321272169176e-8L,  +3.753674773239250330547e-9L,
      -5.052913010605479996432e-10L, +4.580877371233042345794e-11L, -7.664740716178066564952e-13L,
      -7.200170736686941995387e-13L, +1.812701686438975518372e-13L, -2.799876487275995466163e-14L,
      +3.048940815174731772007e-15L, -1.936754063718089166725e-16L, -7.653673328908379651914e-18L,
      +4.534308864750374603371e-18L, -8.011054486030591219007e-19L, +9.374587915222218230337e-20L,
      -7.144943099280650363024e-21L, +1.105276695821552769144e-22L, +6.989334213887669628647e-23L};
  constexpr int degree = sizeof(coeffs) / sizeof(long double) - 1;
  constexpr long double midpoint = 2.0L;

  return chebyshevTnSeries(x - midpoint, coeffs, degree);
}

long double sineChebyshevExpansion3To5(long double x)
{
  static const long double coeffs[] = {
      +3.684922395955255848372e-3L,  -2.624595437764014386717e-3L,  +6.329162500611499391493e-4L,
      -1.258275676151483358569e-4L,  +2.207375763252044217165e-5L,  -3.521929664607266176132e-6L,
      +5.186211398012883705616e-7L,  -7.095056569102400546407e-8L,  +9.030550018646936241849e-9L,
      -1.066057806832232908641e-9L,  +1.157128073917012957550e-10L, -1.133877461819345992066e-11L,
      +9.633572308791154852278e-13L, -6.336675771012312827721e-14L, +1.634407356931822107368e-15L,
      +3.944542177576016972249e-16L, -9.577486627424256130607e-17L, +1.428772744117447206807e-17L,
      -1.715342656474756703926e-18L, +1.753564314320837957805e-19L, -1.526125102356904908532e-20L,
      +1.070275366865736879194e-21L, -4.783978662888842165071e-23L};
  constexpr int degree = sizeof(coeffs) / sizeof(long double) - 1;
  constexpr long double midpoint = 4.0L;

  return chebyshevTnSeries(x - midpoint, coeffs, degree);
}

long double sineChebyshevExpansion5To7(long double x)
{
  static const long double coeffs[] = {
      +1.000801217561417083840e-3L,  -4.915205279689293180607e-4L,  +8.133163567827942356534e-5L,
      -1.120758739236976144656e-5L,  +1.384441872281356422699e-6L,  -1.586485067224130537823e-7L,
      +1.717840749804993618997e-8L,  -1.776373217323590289701e-9L,  +1.765399783094380160549e-10L,
      -1.692470022450343343158e-11L, +1.568238301528778401489e-12L, -1.405356860742769958771e-13L,
      +1.217377701691787512346e-14L, -1.017697418261094517680e-15L, +8.186068056719295045596e-17L,
      -6.305153620995673221364e-18L, +4.614110100197028845266e-19L, -3.165914620159266813849e-20L,
      +1.986716456911232767045e-21L, -1.078418278174434671506e-22L, +4.255983404468350776788e-24L};
  constexpr int degree = sizeof(coeffs) / sizeof(long double) - 1;
  constexpr long double midpoint = 6.0L;

  return chebyshevTnSeries(x - midpoint, coeffs, degree);
}

/**
 * @brief Asymptotic series for auxiliary Fresnel sine integral
 *
 * For large x > 7, uses:
 * g(x) ~ 1/(x^3 * sqrt(8*pi)) * [1 - 15/(4*x^4) + 945/(16*x^8) + ...]
 *
 * @param x Argument (must be > 7)
 * @return Value of g(x)
 */
long double sineAsymptoticSeries(long double x)
{
  const long double x2 = x * x;
  const long double x4 = -4.0L * x2 * x2;

  long double xPower = 1.0L;
  long double factorial = 1.0L;
  long double terms[kNumAsymptoticTerms + 1];
  const long double epsilon = LDBL_EPSILON / 4.0L;

  int j = 5;
  int numTerms = 0;

  terms[0] = 1.0L;
  terms[kNumAsymptoticTerms] = 0.0L;

  for (int i = 1; i < kNumAsymptoticTerms; i++)
  {
    factorial *= static_cast<long double>(j) * static_cast<long double>(j - 2);
    xPower *= x4;
    terms[i] = factorial / xPower;
    j += 4;

    if (std::abs(terms[i]) >= std::abs(terms[i - 1]))
    {
      numTerms = i - 1;
      break;
    }
    if (std::abs(terms[i]) <= epsilon)
    {
      numTerms = i;
      break;
    }
  }

  long double sum = 0.0L;
  for (int i = numTerms; i >= 0; i--)
  {
    sum += terms[i];
  }

  sum /= (x * kSqrt2Pi);
  return sum / (x2 + x2);
}

/**
 * @brief Fresnel auxiliary sine integral g(x)
 *
 * Computes the integral from 0 to infinity of:
 * sqrt(2/pi) * exp(-2*x*t) * sin(t^2) dt
 *
 * @param x Argument (must be >= 0)
 * @return Value of g(x)
 */
long double fresnelAuxiliarySineIntegral(long double x)
{
  if (x == 0.0L)
  {
    return 0.5L;
  }
  if (x <= 1.0L)
  {
    return sineChebyshevExpansion0To1(x);
  }
  if (x <= 3.0L)
  {
    return sineChebyshevExpansion1To3(x);
  }
  if (x <= 5.0L)
  {
    return sineChebyshevExpansion3To5(x);
  }
  if (x <= 7.0L)
  {
    return sineChebyshevExpansion5To7(x);
  }
  return sineAsymptoticSeries(x);
}

// Chebyshev expansions for cosine auxiliary integral
long double cosineChebyshevExpansion0To1(long double x)
{
  static const long double coeffs[] = {
      +4.200987560240514577713e-1L,  -9.358785913634965235904e-2L,  -7.642539415723373644927e-3L,
      +4.958117751796130135544e-3L,  -9.750236036106120253456e-4L,  +1.075201474958704192865e-4L,
      -4.415344769301324238886e-6L,  -7.861633919783064216022e-7L,  +1.919240966215861471754e-7L,
      -2.175775608982741065385e-8L,  +1.296559541430849437217e-9L,  +2.207205095025162212169e-11L,
      -1.479219615873704298874e-11L, +1.821350127295808288614e-12L, -1.228919312990171362342e-13L,
      +2.227139250593818235212e-15L, +5.734729405928016301596e-16L, -8.284965573075354177016e-17L,
      +6.067422701530157308321e-18L, -1.994908519477689596319e-19L, -1.173365630675305693390e-20L};
  constexpr int degree = sizeof(coeffs) / sizeof(long double) - 1;
  constexpr long double midpoint = 0.5L;
  constexpr long double scale = 0.5L;

  return chebyshevTnSeries((x - midpoint) / scale, coeffs, degree);
}

long double cosineChebyshevExpansion1To3(long double x)
{
  static const long double coeffs[] = {
      +2.098677278318224971989e-1L,  -9.314234883154103266195e-2L,  +1.739905936938124979297e-2L,
      -2.454274824644285136137e-3L,  +1.589872606981337312438e-4L,  +4.203943842506079780413e-5L,
      -2.018022256093216535093e-5L,  +5.125709636776428285284e-6L,  -9.601813551752718650057e-7L,
      +1.373989484857155846826e-7L,  -1.348105546577211255591e-8L,  +2.745868700337953872632e-10L,
      +2.401655517097260106976e-10L, -6.678059547527685587692e-11L, +1.140562171732840809159e-11L,
      -1.401526517205212219089e-12L, +1.105498827380224475667e-13L, +2.040731455126809208066e-16L,
      -1.946040679213045143184e-15L, +4.151821375667161733612e-16L, -5.642257647205149369594e-17L,
      +5.266176626521504829010e-18L, -2.299025577897146333791e-19L, -2.952226367506641078731e-20L,
      +8.760405943193778149078e-21L};
  constexpr int degree = sizeof(coeffs) / sizeof(long double) - 1;
  constexpr long double midpoint = 2.0L;

  return chebyshevTnSeries(x - midpoint, coeffs, degree);
}

long double cosineChebyshevExpansion3To5(long double x)
{
  static const long double coeffs[] = {
      +1.025703371090289562388e-1L,  -2.569833023232301400495e-2L,  +3.160592981728234288078e-3L,
      -3.776110718882714758799e-4L,  +4.325593433537248833341e-5L,  -4.668447489229591855730e-6L,
      +4.619254757356785108280e-7L,  -3.970436510433553795244e-8L,  +2.535664754977344448598e-9L,
      -2.108170964644819803367e-11L, -2.959172018518707683013e-11L, +6.727219944906606516055e-12L,
      -1.062829587519902899001e-12L, +1.402071724705287701110e-13L, -1.619154679722651005075e-14L,
      +1.651319588396970446858e-15L, -1.461704569438083772889e-16L, +1.053521559559583268504e-17L,
      -4.760946403462515858756e-19L, -1.803784084922403924313e-20L, +7.873130866418738207547e-21L};
  constexpr int degree = sizeof(coeffs) / sizeof(long double) - 1;
  constexpr long double midpoint = 4.0L;

  return chebyshevTnSeries(x - midpoint, coeffs, degree);
}

long double cosineChebyshevExpansion5To7(long double x)
{
  static const long double coeffs[] = {
      +6.738667333400589274018e-2L,  -1.128146832637904868638e-2L,  +9.408843234170404670278e-4L,
      -7.800074103496165011747e-5L,  +6.409101169623350885527e-6L,  -5.201350558247239981834e-7L,
      +4.151668914650221476906e-8L,  -3.242202015335530552721e-9L,  +2.460339340900396789789e-10L,
      -1.796823324763304661865e-11L, +1.244108496436438952425e-12L, -7.950417122987063540635e-14L,
      +4.419142625999150971878e-15L, -1.759082736751040110146e-16L, -1.307443936270786700760e-18L,
      +1.362484141039320395814e-18L, -2.055236564763877250559e-19L, +2.329142055084791308691e-20L,
      -2.282438671525884861970e-21L};
  constexpr int degree = sizeof(coeffs) / sizeof(long double) - 1;
  constexpr long double midpoint = 6.0L;

  return chebyshevTnSeries(x - midpoint, coeffs, degree);
}

/**
 * @brief Asymptotic series for auxiliary Fresnel cosine integral
 *
 * For large x > 7, uses:
 * f(x) ~ 1/(x * sqrt(2*pi)) * [1 - 3/(4*x^4) + 105/(16*x^8) + ...]
 *
 * @param x Argument (must be > 7)
 * @return Value of f(x)
 */
long double cosineAsymptoticSeries(long double x)
{
  const long double x2 = x * x;
  const long double x4 = -4.0L * x2 * x2;

  long double xPower = 1.0L;
  long double factorial = 1.0L;
  long double terms[kNumAsymptoticTerms + 1];
  const long double epsilon = LDBL_EPSILON / 4.0L;

  int j = 3;
  int numTerms = 0;

  terms[0] = 1.0L;
  terms[kNumAsymptoticTerms] = 0.0L;

  for (int i = 1; i < kNumAsymptoticTerms; i++)
  {
    factorial *= static_cast<long double>(j) * static_cast<long double>(j - 2);
    xPower *= x4;
    terms[i] = factorial / xPower;
    j += 4;

    if (std::abs(terms[i]) >= std::abs(terms[i - 1]))
    {
      numTerms = i - 1;
      break;
    }
    if (std::abs(terms[i]) <= epsilon)
    {
      numTerms = i;
      break;
    }
  }

  long double sum = 0.0L;
  for (int i = numTerms; i >= 0; i--)
  {
    sum += terms[i];
  }

  return sum / (x * kSqrt2Pi);
}

/**
 * @brief Fresnel auxiliary cosine integral f(x)
 *
 * Computes the integral from 0 to infinity of:
 * sqrt(2/pi) * exp(-2*x*t) * cos(t^2) dt
 *
 * @param x Argument (must be >= 0)
 * @return Value of f(x)
 */
long double fresnelAuxiliaryCosineIntegral(long double x)
{
  if (x == 0.0L)
  {
    return 0.5L;
  }
  if (x <= 1.0L)
  {
    return cosineChebyshevExpansion0To1(x);
  }
  if (x <= 3.0L)
  {
    return cosineChebyshevExpansion1To3(x);
  }
  if (x <= 5.0L)
  {
    return cosineChebyshevExpansion3To5(x);
  }
  if (x <= 7.0L)
  {
    return cosineChebyshevExpansion5To7(x);
  }
  return cosineAsymptoticSeries(x);
}

/**
 * @brief Fresnel sine integral (alternative normalization)
 *
 * Computes the integral from 0 to x of sqrt(2/pi) * sin(t^2) dt
 *
 * @param x Upper limit of integration
 * @return Value of the integral
 */
long double fresnelSinAlt(long double x)
{
  if (std::abs(x) < 0.5L)
  {
    return powerSeriesS(x);
  }

  const long double absX = std::abs(x);
  const long double f = fresnelAuxiliaryCosineIntegral(absX);
  const long double g = fresnelAuxiliarySineIntegral(absX);
  const long double x2 = x * x;
  const long double result = 0.5L - std::cos(x2) * f - std::sin(x2) * g;

  return (x < 0.0L) ? -result : result;
}

/**
 * @brief Fresnel cosine integral (alternative normalization)
 *
 * Computes the integral from 0 to x of sqrt(2/pi) * cos(t^2) dt
 *
 * @param x Upper limit of integration
 * @return Value of the integral
 */
long double fresnelCosAlt(long double x)
{
  if (std::abs(x) < 0.5L)
  {
    return powerSeriesC(x);
  }

  const long double absX = std::abs(x);
  const long double f = fresnelAuxiliaryCosineIntegral(absX);
  const long double g = fresnelAuxiliarySineIntegral(absX);
  const long double x2 = x * x;
  const long double result = 0.5L + std::sin(x2) * f - std::cos(x2) * g;

  return (x < 0.0L) ? -result : result;
}

}  // anonymous namespace

// Public API implementations
namespace mrpt::math
{
long double lfresnel_sin_integral(long double x) noexcept
{
  return fresnelSinAlt(x / kSqrt2OverPi);
}

long double lfresnel_cos_integral(long double x) noexcept
{
  return fresnelCosAlt(x / kSqrt2OverPi);
}

double fresnel_sin_integral(double x) noexcept
{
  return static_cast<double>(lfresnel_sin_integral(static_cast<long double>(x)));
}

double fresnel_cos_integral(double x) noexcept
{
  return static_cast<double>(lfresnel_cos_integral(static_cast<long double>(x)));
}

}  // namespace mrpt::math
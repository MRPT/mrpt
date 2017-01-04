/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/math/fresnel.h>
#include <cmath>
#include <cfloat>

// Based on code (public domain) in http://www.mymathlib.com/functions/

static long double Power_Series_S(long double x);
static long double xFresnel_Auxiliary_Cosine_Integral(long double x);
static long double xFresnel_Auxiliary_Sine_Integral(long double x);
static long double Power_Series_C(long double x);
static long double xChebyshev_Tn_Series(long double x, const long double a[], int degree);

//extern long double xFresnel_Sine_Integral(long double x);
//extern long double xFresnel_Cosine_Integral(long double x);


// the integral from 0 to x of sqrt(2 / pi) sin(t ^ 2) dt.
long double lfresnel_sin_alt(long double x)
{
	long double f, g, x2, s;

	if (std::abs(x) < 0.5L) return Power_Series_S(x);

	f = xFresnel_Auxiliary_Cosine_Integral(fabsl(x));
	g = xFresnel_Auxiliary_Sine_Integral(fabsl(x));
	x2 = x * x;
	s = 0.5L - cosl(x2) * f - sinl(x2) * g;
	return (x < 0.0L) ? -s : s;
}

// the integral from 0 to x of sqrt(2 / pi) cos(t ^ 2) dt.
long double lfresnel_cos_alt(long double x)
{
	long double f, g, x2, c;

	if (fabsl(x) < 0.5L) return Power_Series_C(x);

	f = xFresnel_Auxiliary_Cosine_Integral(fabsl(x));
	g = xFresnel_Auxiliary_Sine_Integral(fabsl(x));
	x2 = x * x;
	c = 0.5L + sinl(x2) * f - cosl(x2) * g;
	return (x < 0.0L) ? -c : c;
}

long double mrpt::math::lfresnel_sin_integral(long double x) MRPT_NO_THROWS
{
	long double sqrt_2_o_pi = 7.978845608028653558798921198687637369517e-1L;
	return lfresnel_sin_alt(x/ sqrt_2_o_pi);
}

long double mrpt::math::lfresnel_cos_integral(long double x) MRPT_NO_THROWS
{
	long double sqrt_2_o_pi = 7.978845608028653558798921198687637369517e-1L;
	return lfresnel_cos_alt(x / sqrt_2_o_pi);
}

double mrpt::math::fresnel_sin_integral(double x) MRPT_NO_THROWS
{
	return static_cast<double>(lfresnel_sin_integral(static_cast<long double>(x)));
}

double mrpt::math::fresnel_cos_integral(double x) MRPT_NO_THROWS
{
	return static_cast<double>(lfresnel_cos_integral(static_cast<long double>(x)));
}

//                         Internally Defined Routines                        //
double      Fresnel_Sine_Integral(double x);
long double xFresnel_Sine_Integral(long double x);


////////////////////////////////////////////////////////////////////////////////
// double Fresnel_Sine_Integral( double x )                                   //
//                                                                            //
//  Description:                                                              //
//     The Fresnel sine integral, S(x), is the integral with integrand        //
//                          sqrt(2/pi) sin(t^2) dt                            //
//     where the integral extends from 0 to x.                                //
//                                                                            //
//  Arguments:                                                                //
//     double  x  The argument of the Fresnel sine integral S().              //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel sine integral S evaluated at x.               //
//                                                                            //
//  Example:                                                                  //
//     double y, x;                                                           //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = Fresnel_Sine_Integral( x );                                        //
////////////////////////////////////////////////////////////////////////////////
double Fresnel_Sine_Integral(double x)
{
	return (double)xFresnel_Sine_Integral((long double)x);
}


////////////////////////////////////////////////////////////////////////////////
// long double xFresnel_Sine_Integral( long double x )                        //
//                                                                            //
//  Description:                                                              //
//     The Fresnel sine integral, S(x), is the integral with integrand        //
//                          sqrt(2/pi) sin(t^2) dt                            //
//     where the integral extends from 0 to x.                                //
//                                                                            //
//  Arguments:                                                                //
//     long double  x  The argument of the Fresnel sine integral S().         //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel sine integral S evaluated at x.               //
//                                                                            //
//  Example:                                                                  //
//     long double y, x;                                                      //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = xFresnel_Sine_Integral( x );                                       //
////////////////////////////////////////////////////////////////////////////////

long double xFresnel_Sine_Integral(long double x)
{
	long double f;
	long double g;
	long double x2;
	long double s;

	if (fabsl(x) < 0.5L) return Power_Series_S(x);

	f = xFresnel_Auxiliary_Cosine_Integral(fabsl(x));
	g = xFresnel_Auxiliary_Sine_Integral(fabsl(x));
	x2 = x * x;
	s = 0.5L - cosl(x2) * f - sinl(x2) * g;
	return (x < 0.0L) ? -s : s;
}

////////////////////////////////////////////////////////////////////////////////
// static long double Power_Series_S( long double x )                         //
//                                                                            //
//  Description:                                                              //
//     The power series representation for the Fresnel sine integral, S(x),   //
//      is                                                                    //
//               x^3 sqrt(2/pi) Sum (-x^4)^j / [(4j+3) (2j+1)!]               //
//     where the sum extends over j = 0, ,,,.                                 //
//                                                                            //
//  Arguments:                                                                //
//     long double  x                                                         //
//                The argument of the Fresnel sine integral S().              //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel sine integral S evaluated at x.               //
//                                                                            //
//  Example:                                                                  //
//     long double y, x;                                                      //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = Power_Series_S( x );                                               //
////////////////////////////////////////////////////////////////////////////////

static long double Power_Series_S(long double x)
{
	long double x2 = x * x;
	long double x3 = x * x2;
	long double x4 = -x2 * x2;
	long double xn = 1.0L;
	long double Sn = 1.0L;
	long double Sm1 = 0.0L;
	long double term;
	long double factorial = 1.0L;
	long double sqrt_2_o_pi = 7.978845608028653558798921198687637369517e-1L;
	int y = 0;

	if (x == 0.0L) return 0.0L;
	Sn /= 3.0L;
	while (fabsl(Sn - Sm1) > LDBL_EPSILON * fabsl(Sm1)) {
		Sm1 = Sn;
		y += 1;
		factorial *= (long double)(y + y);
		factorial *= (long double)(y + y + 1);
		xn *= x4;
		term = xn / factorial;
		term /= (long double)(y + y + y + y + 3);
		Sn += term;
	}
	return x3 * sqrt_2_o_pi * Sn;
}


//                         Internally Defined Routines                        //
double      Fresnel_Auxiliary_Sine_Integral(double x);
long double xFresnel_Auxiliary_Sine_Integral(long double x);


//                         Internally Defined Constants                       //
static long double const sqrt_2pi = 2.506628274631000502415765284811045253006L;


////////////////////////////////////////////////////////////////////////////////
// double Fresnel_Auxiliary_Sine_Integral( double x )                         //
//                                                                            //
//  Description:                                                              //
//     The Fresnel auxiliary sine integral, g(x), is the integral from 0 to   //
//     infinity of the integrand                                              //
//                     sqrt(2/pi) exp(-2xt) sin(t^2) dt                       //
//     where x >= 0.                                                          //
//                                                                            //
//  Arguments:                                                                //
//     double  x  The argument of the Fresnel auxiliary sine integral g()     //
//                where x >= 0.                                               //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel auxiliary sine integral g evaluated at        //
//     x >= 0.                                                                //
//                                                                            //
//  Example:                                                                  //
//     double y, x;                                                           //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = Fresnel_Auxiliary_Sine_Integral( x );                              //
////////////////////////////////////////////////////////////////////////////////
double Fresnel_Auxiliary_Sine_Integral(double x)
{
	return (double)xFresnel_Auxiliary_Sine_Integral((long double)x);
}


////////////////////////////////////////////////////////////////////////////////
// long double xFresnel_Auxiliary_Sine_Integral( double x )                   //
//                                                                            //
//  Description:                                                              //
//     The Fresnel auxiliary sine integral, g(x), is the integral from 0 to   //
//     infinity of the integrand                                              //
//                     sqrt(2/pi) exp(-2xt) sin(t^2) dt                       //
//     where x >= 0.                                                          //
//                                                                            //
//  Arguments:                                                                //
//     long double  x  The argument of the Fresnel auxiliary sine integral    //
//                     g() where x >= 0.                                      //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel auxiliary sine integral g evaluated at        //
//     x >= 0.                                                                //
//                                                                            //
//  Example:                                                                  //
//     double y, x;                                                           //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = xFresnel_Auxiliary_Sine_Integral( x );                             //
////////////////////////////////////////////////////////////////////////////////

static long double sin_Chebyshev_Expansion_0_1(long double x);
static long double sin_Chebyshev_Expansion_1_3(long double x);
static long double sin_Chebyshev_Expansion_3_5(long double x);
static long double sin_Chebyshev_Expansion_5_7(long double x);
static long double sin_Asymptotic_Series(long double x);

long double xFresnel_Auxiliary_Sine_Integral(long double x)
{
	if (x == 0.0L) return 0.5L;
	if (x <= 1.0L) return sin_Chebyshev_Expansion_0_1(x);
	if (x <= 3.0L) return sin_Chebyshev_Expansion_1_3(x);
	if (x <= 5.0L) return sin_Chebyshev_Expansion_3_5(x);
	if (x <= 7.0L) return sin_Chebyshev_Expansion_5_7(x);
	return sin_Asymptotic_Series(x);
}



////////////////////////////////////////////////////////////////////////////////
// static long double Chebyshev_Expansion_0_1( long double x )                //
//                                                                            //
//  Description:                                                              //
//     Evaluate the Fresnel auxiliary sine integral, g(x), on the interval    //
//     0 < x <= 1 using the Chebyshev interpolation formula.                  //
//                                                                            //
//  Arguments:                                                                //
//     long double  x  The argument of the Fresnel auxiliary sine integral    //
//                     where 0 < x <= 1.                                      //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel auxiliary sine integral g evaluated at        //
//     x where 0 < x <= 1.                                                    //
//                                                                            //
//  Example:                                                                  //
//     long double y, x;                                                      //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = Chebyshev_Expansion_0_1(x);                                        //
////////////////////////////////////////////////////////////////////////////////

static long double sin_Chebyshev_Expansion_0_1(long double x)
{
	static long double const c[] = {
		+2.560134650043040830997e-1L,  -1.993005146464943284549e-1L,
		+4.025503636721387266117e-2L,  -4.459600454502960250729e-3L,
		+6.447097305145147224459e-5L,  +7.544218493763717599380e-5L,
		-1.580422720690700333493e-5L,  +1.755845848573471891519e-6L,
		-9.289769688468301734718e-8L,  -5.624033192624251079833e-9L,
		+1.854740406702369495830e-9L,  -2.174644768724492443378e-10L,
		+1.392899828133395918767e-11L, -6.989216003725983789869e-14L,
		-9.959396121060010838331e-14L, +1.312085140393647257714e-14L,
		-9.240470383522792593305e-16L, +2.472168944148817385152e-17L,
		+2.834615576069400293894e-18L, -4.650983461314449088349e-19L,
		+3.544083040732391556797e-20L
	};

	static const int degree = sizeof(c) / sizeof(long double) - 1;
	static const long double midpoint = 0.5L;
	static const long double scale = 0.5L;

	return xChebyshev_Tn_Series((x - midpoint) / scale, c, degree);
}


////////////////////////////////////////////////////////////////////////////////
// static long double Chebyshev_Expansion_1_3( long double x )                //
//                                                                            //
//  Description:                                                              //
//     Evaluate the Fresnel auxiliary sine integral, g(x), on the interval    //
//     1 < x <= 3 using the Chebyshev interpolation formula.                  //
//                                                                            //
//  Arguments:                                                                //
//     long double  x  The argument of the Fresnel auxiliary sine integral    //
//                     where 1 < x <= 3.                                      //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel auxiliary sine integral g evaluated at        //
//     x where 1 < x <= 3.                                                    //
//                                                                            //
//  Example:                                                                  //
//     long double y, x;                                                      //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = Chebyshev_Expansion_1_3(x);                                        //
////////////////////////////////////////////////////////////////////////////////

static long double sin_Chebyshev_Expansion_1_3(long double x)
{
	static long double const c[] = {
		+3.470341566046115476477e-2L,  -3.855580521778624043304e-2L,
		+1.420604309383996764083e-2L,  -4.037349972538938202143e-3L,
		+9.292478174580997778194e-4L,  -1.742730601244797978044e-4L,
		+2.563352976720387343201e-5L,  -2.498437524746606551732e-6L,
		-1.334367201897140224779e-8L,  +7.436854728157752667212e-8L,
		-2.059620371321272169176e-8L,  +3.753674773239250330547e-9L,
		-5.052913010605479996432e-10L, +4.580877371233042345794e-11L,
		-7.664740716178066564952e-13L, -7.200170736686941995387e-13L,
		+1.812701686438975518372e-13L, -2.799876487275995466163e-14L,
		+3.048940815174731772007e-15L, -1.936754063718089166725e-16L,
		-7.653673328908379651914e-18L, +4.534308864750374603371e-18L,
		-8.011054486030591219007e-19L, +9.374587915222218230337e-20L,
		-7.144943099280650363024e-21L, +1.105276695821552769144e-22L,
		+6.989334213887669628647e-23L
	};

	static const int degree = sizeof(c) / sizeof(long double) - 1;
	static const long double midpoint = 2.0L;

	return xChebyshev_Tn_Series((x - midpoint), c, degree);
}


////////////////////////////////////////////////////////////////////////////////
// static long double Chebyshev_Expansion_3_5( long double x )                //
//                                                                            //
//  Description:                                                              //
//     Evaluate the Fresnel auxiliary sine integral, g(x), on the interval    //
//     3 < x <= 5 using the Chebyshev interpolation formula.                  //
//                                                                            //
//  Arguments:                                                                //
//     long double  x  The argument of the Fresnel auxiliary sine integral    //
//                     where 3 < x <= 5.                                      //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel auxiliary sine integral g evaluated at        //
//     x where 3 < x <= 5.                                                    //
//                                                                            //
//  Example:                                                                  //
//     long double y, x;                                                      //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = Chebyshev_Expansion_3_5(x);                                        //
////////////////////////////////////////////////////////////////////////////////

static long double sin_Chebyshev_Expansion_3_5(long double x)
{
	static long double const c[] = {
		+3.684922395955255848372e-3L,  -2.624595437764014386717e-3L,
		+6.329162500611499391493e-4L,  -1.258275676151483358569e-4L,
		+2.207375763252044217165e-5L,  -3.521929664607266176132e-6L,
		+5.186211398012883705616e-7L,  -7.095056569102400546407e-8L,
		+9.030550018646936241849e-9L,  -1.066057806832232908641e-9L,
		+1.157128073917012957550e-10L, -1.133877461819345992066e-11L,
		+9.633572308791154852278e-13L, -6.336675771012312827721e-14L,
		+1.634407356931822107368e-15L, +3.944542177576016972249e-16L,
		-9.577486627424256130607e-17L, +1.428772744117447206807e-17L,
		-1.715342656474756703926e-18L, +1.753564314320837957805e-19L,
		-1.526125102356904908532e-20L, +1.070275366865736879194e-21L,
		-4.783978662888842165071e-23L
	};

	static const int degree = sizeof(c) / sizeof(long double) - 1;
	static const long double midpoint = 4.0L;

	return xChebyshev_Tn_Series((x - midpoint), c, degree);
}


////////////////////////////////////////////////////////////////////////////////
// static long double Chebyshev_Expansion_5_7( long double x )                //
//                                                                            //
//  Description:                                                              //
//     Evaluate the Fresnel auxiliary sine integral, g(x), on the interval    //
//     5 < x <= 7 using the Chebyshev interpolation formula.                  //
//                                                                            //
//  Arguments:                                                                //
//     long double  x  The argument of the Fresnel auxiliary sine integral    //
//                     where 5 < x <= 7.                                      //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel auxiliary sine integral g evaluated at        //
//     x where 5 < x <= 7.                                                    //
//                                                                            //
//  Example:                                                                  //
//     long double y, x;                                                      //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = Chebyshev_Expansion_5_7(x);                                        //
////////////////////////////////////////////////////////////////////////////////

static long double sin_Chebyshev_Expansion_5_7(long double x)
{
	static long double const c[] = {
		+1.000801217561417083840e-3L,  -4.915205279689293180607e-4L,
		+8.133163567827942356534e-5L,  -1.120758739236976144656e-5L,
		+1.384441872281356422699e-6L,  -1.586485067224130537823e-7L,
		+1.717840749804993618997e-8L,  -1.776373217323590289701e-9L,
		+1.765399783094380160549e-10L, -1.692470022450343343158e-11L,
		+1.568238301528778401489e-12L, -1.405356860742769958771e-13L,
		+1.217377701691787512346e-14L, -1.017697418261094517680e-15L,
		+8.186068056719295045596e-17L, -6.305153620995673221364e-18L,
		+4.614110100197028845266e-19L, -3.165914620159266813849e-20L,
		+1.986716456911232767045e-21L, -1.078418278174434671506e-22L,
		+4.255983404468350776788e-24L
	};

	static const int degree = sizeof(c) / sizeof(long double) - 1;
	static const long double midpoint = 6.0L;

	return xChebyshev_Tn_Series((x - midpoint), c, degree);

}


////////////////////////////////////////////////////////////////////////////////
// static long double Asymptotic_Series( long double x )                      //
//                                                                            //
//  Description:                                                              //
//     For a large argument x, the auxiliary Fresnel sine integral, g(x),     //
//     can be expressed as the asymptotic series                              //
//      g(x) ~ 1/(x^3 * sqrt(8pi))[1 - 15/4x^4 + 945/16x^8 + ... +            //
//                                                (4j+1)!!/(-4x^4)^j + ... ]  //
//                                                                            //
//  Arguments:                                                                //
//     long double  x  The argument of the Fresnel auxiliary sine integral    //
//                     where x > 7.                                           //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel auxiliary sine integral g evaluated at        //
//     x where x > 7.                                                         //
//                                                                            //
//  Example:                                                                  //
//     long double y, x;                                                      //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = Asymptotic_Series( x );                                            //
////////////////////////////////////////////////////////////////////////////////
#define NUM_ASYMPTOTIC_TERMS 35
static long double sin_Asymptotic_Series(long double x)
{
	long double x2 = x * x;
	long double x4 = -4.0L * x2 * x2;
	long double xn = 1.0L;
	long double factorial = 1.0L;
	long double g = 0.0L;
	long double term[NUM_ASYMPTOTIC_TERMS + 1];
	long double epsilon = LDBL_EPSILON / 4.0L;
	int j = 5;
	int i = 0;

	term[0] = 1.0L;
	term[NUM_ASYMPTOTIC_TERMS] = 0.0L;
	for (i = 1; i < NUM_ASYMPTOTIC_TERMS; i++) {
		factorial *= ((long double)j * (long double)(j - 2));
		xn *= x4;
		term[i] = factorial / xn;
		j += 4;
		if (fabsl(term[i]) >= fabsl(term[i - 1])) {
			i--;
			break;
		}
		if (fabsl(term[i]) <= epsilon) break;
	}
	for (; i >= 0; i--) g += term[i];

	g /= (x * sqrt_2pi);
	return g / (x2 + x2);
}

//                         Internally Defined Routines                        //
double      Fresnel_Auxiliary_Cosine_Integral(double x);
long double xFresnel_Auxiliary_Cosine_Integral(long double x);


////////////////////////////////////////////////////////////////////////////////
// double Fresnel_Auxiliary_Cosine_Integral( double x )                       //
//                                                                            //
//  Description:                                                              //
//     The Fresnel auxiliary cosine integral, f(x), is the integral from 0 to //
//     infinity of the integrand                                              //
//                     sqrt(2/pi) exp(-2xt) cos(t^2) dt                       //
//     where x >= 0.                                                          //
//                                                                            //
//  Arguments:                                                                //
//     double  x  The argument of the Fresnel auxiliary cosine integral f()   //
//                where x >= 0.                                               //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel auxiliary cosine integral f evaluated at      //
//     x >= 0.                                                                //
//                                                                            //
//  Example:                                                                  //
//     double y, x;                                                           //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = Fresnel_Auxiliary_Cosine_Integral( x );                            //
////////////////////////////////////////////////////////////////////////////////
double Fresnel_Auxiliary_Cosine_Integral(double x)
{
	return (double)xFresnel_Auxiliary_Cosine_Integral((long double)x);
}

static long double cos_Chebyshev_Expansion_0_1(long double x);
static long double cos_Chebyshev_Expansion_1_3(long double x);
static long double cos_Chebyshev_Expansion_3_5(long double x);
static long double cos_Chebyshev_Expansion_5_7(long double x);
static long double cos_Asymptotic_Series(long double x);

////////////////////////////////////////////////////////////////////////////////
// long double xFresnel_Auxiliary_Cosine_Integral( double x )                 //
//                                                                            //
//  Description:                                                              //
//     The Fresnel auxiliary cosine integral, f(x), is the integral from 0 to //
//     infinity of the integrand                                              //
//                     sqrt(2/pi) exp(-2xt) cos(t^2) dt                       //
//     where x >= 0.                                                          //
//                                                                            //
//  Arguments:                                                                //
//     long double  x  The argument of the Fresnel auxiliary cosine integral  //
//                     f() where x >= 0.                                      //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel auxiliary cosine integral f evaluated at      //
//     x >= 0.                                                                //
//                                                                            //
//  Example:                                                                  //
//     double y, x;                                                           //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = xFresnel_Auxiliary_Cosine_Integral( x );                           //
////////////////////////////////////////////////////////////////////////////////
long double xFresnel_Auxiliary_Cosine_Integral(long double x)
{
	if (x == 0.0L) return 0.5L;
	if (x <= 1.0L) return cos_Chebyshev_Expansion_0_1(x);
	if (x <= 3.0L) return cos_Chebyshev_Expansion_1_3(x);
	if (x <= 5.0L) return cos_Chebyshev_Expansion_3_5(x);
	if (x <= 7.0L) return cos_Chebyshev_Expansion_5_7(x);
	return cos_Asymptotic_Series(x);
}

//                         Externally Defined Routines                        //
extern long double xFresnel_Auxiliary_Cosine_Integral(long double x);
extern long double xFresnel_Auxiliary_Sine_Integral(long double x);


//                         Internally Defined Routines                        //
double      Fresnel_Cosine_Integral(double x);
long double xFresnel_Cosine_Integral(long double x);

////////////////////////////////////////////////////////////////////////////////
// double Fresnel_Cosine_Integral( double x )                                 //
//                                                                            //
//  Description:                                                              //
//     The Fresnel cosine integral, C(x), is the integral with integrand      //
//                          sqrt(2/pi) cos(t^2) dt                            //
//     where the integral extends from 0 to x.                                //
//                                                                            //
//  Arguments:                                                                //
//     double  x  The argument of the Fresnel cosine integral C().            //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel cosine integral C evaluated at x.             //
//                                                                            //
//  Example:                                                                  //
//     double y, x;                                                           //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = Fresnel_Cosine_Integral( x );                                      //
////////////////////////////////////////////////////////////////////////////////
double Fresnel_Cosine_Integral(double x)
{
	return (double)xFresnel_Cosine_Integral((long double)x);
}


////////////////////////////////////////////////////////////////////////////////
// long double xFresnel_Cosine_Integral( long double x )                      //
//                                                                            //
//  Description:                                                              //
//     The Fresnel cosine integral, C(x), is the integral with integrand      //
//                          sqrt(2/pi) cos(t^2) dt                            //
//     where the integral extends from 0 to x.                                //
//                                                                            //
//  Arguments:                                                                //
//     long double  x  The argument of the Fresnel cosine integral C().       //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel cosine integral C evaluated at x.             //
//                                                                            //
//  Example:                                                                  //
//     long double y, x;                                                      //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = xFresnel_Cosine_Integral( x );                                     //
////////////////////////////////////////////////////////////////////////////////

long double xFresnel_Cosine_Integral(long double x)
{
	long double f;
	long double g;
	long double x2;
	long double c;

	if (fabsl(x) < 0.5L) return Power_Series_C(x);

	f = xFresnel_Auxiliary_Cosine_Integral(fabsl(x));
	g = xFresnel_Auxiliary_Sine_Integral(fabsl(x));
	x2 = x * x;
	c = 0.5L + sinl(x2) * f - cosl(x2) * g;
	return (x < 0.0L) ? -c : c;
}


////////////////////////////////////////////////////////////////////////////////
// static long double Power_Series_C( long double x )                         //
//                                                                            //
//  Description:                                                              //
//     The power series representation for the Fresnel cosine integral, C(x), //
//      is                                                                    //
//                 x sqrt(2/pi) Sum (-x^4)^j / [(4j+1) (2j)!]                 //
//     where the sum extends over j = 0, ,,,.                                 //
//                                                                            //
//  Arguments:                                                                //
//     long double  x                                                         //
//                The argument of the Fresnel cosine integral C().            //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel cosine integral C evaluated at x.             //
//                                                                            //
//  Example:                                                                  //
//     long double y, x;                                                      //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = Power_Series_C( x );                                               //
////////////////////////////////////////////////////////////////////////////////

static long double Power_Series_C(long double x)
{
	long double x2 = x * x;
//	long double x3 = x * x2;
	long double x4 = -x2 * x2;
	long double xn = 1.0L;
	long double Sn = 1.0L;
	long double Sm1 = 0.0L;
	long double term;
	long double factorial = 1.0L;
	long double sqrt_2_o_pi = 7.978845608028653558798921198687637369517e-1L;
	int y = 0;

	if (x == 0.0L) return 0.0L;
	while (fabsl(Sn - Sm1) > LDBL_EPSILON * fabsl(Sm1)) {
		Sm1 = Sn;
		y += 1;
		factorial *= (long double)(y + y);
		factorial *= (long double)(y + y - 1);
		xn *= x4;
		term = xn / factorial;
		term /= (long double)(y + y + y + y + 1);
		Sn += term;
	}
	return x * sqrt_2_o_pi * Sn;
}

////////////////////////////////////////////////////////////////////////////////
// File: xchebyshev_Tn_series.c                                               //
// Routine(s):                                                                //
//    xChebyshev_Tn_Series                                                    //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// long double xChebyshev_Tn_Series(long double x, long double a[],int degree)//
//                                                                            //
//  Description:                                                              //
//     This routine uses Clenshaw's recursion algorithm to evaluate a given   //
//     polynomial p(x) expressed as a linear combination of Chebyshev         //
//     polynomials of the first kind, Tn, at a point x,                       //
//       p(x) = a[0] + a[1]*T[1](x) + a[2]*T[2](x) + ... + a[deg]*T[deg](x).  //
//                                                                            //
//     Clenshaw's recursion formula applied to Chebyshev polynomials of the   //
//     first kind is:                                                         //
//     Set y[degree + 2] = 0, y[degree + 1] = 0, then for k = degree, ..., 1  //
//     set y[k] = 2 * x * y[k+1] - y[k+2] + a[k].  Finally                    //
//     set y[0] = x * y[1] - y[2] + a[0].  Then p(x) = y[0].                  //
//                                                                            //
//  Arguments:                                                                //
//     long double x                                                          //
//        The point at which to evaluate the polynomial.                      //
//     long double a[]                                                        //
//        The coefficients of the expansion in terms of Chebyshev polynomials,//
//        i.e. a[k] is the coefficient of T[k](x).  Note that in the calling  //
//        routine a must be defined double a[N] where N >= degree + 1.        //
//     int    degree                                                          //
//        The degree of the polynomial p(x).                                  //
//                                                                            //
//  Return Value:                                                             //
//     The value of the polynomial at x.                                      //
//     If degree is negative, then 0.0 is returned.                           //
//                                                                            //
//  Example:                                                                  //
//     long double x, a[N], p;                                                //
//     int    deg = N - 1;                                                    //
//                                                                            //
//     ( code to initialize x, and a[i] i = 0, ... , a[deg] )                 //
//                                                                            //
//     p = xChebyshev_Tn_Series(x, a, deg);                                   //
////////////////////////////////////////////////////////////////////////////////
static long double xChebyshev_Tn_Series(long double x, const long double a[], int degree)
{
	long double yp2 = 0.0L;
	long double yp1 = 0.0L;
	long double y = 0.0L;
	long double two_x = x + x;
	int k;

	// Check that degree >= 0.  If not, then return 0. //

	if (degree < 0) return 0.0L;

	// Apply Clenshaw's recursion save the last iteration. //

	for (k = degree; k >= 1; k--, yp2 = yp1, yp1 = y)
		y = two_x * yp1 - yp2 + a[k];

	// Now apply the last iteration and return the result. //

	return x * yp1 - yp2 + a[0];
}



////////////////////////////////////////////////////////////////////////////////
// static long double Chebyshev_Expansion_0_1( long double x )                //
//                                                                            //
//  Description:                                                              //
//     Evaluate the Fresnel auxiliary cosine integral, f(x), on the interval  //
//     0 < x <= 1 using the Chebyshev interpolation formula.                  //
//                                                                            //
//  Arguments:                                                                //
//     long double  x  The argument of the Fresnel auxiliary cosine integral  //
//                     where 0 < x <= 1.                                      //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel auxiliary cosine integral f evaluated at      //
//     x where 0 < x <= 1.                                                    //
//                                                                            //
//  Example:                                                                  //
//     long double y, x;                                                      //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = Chebyshev_Expansion_0_1(x);                                        //
////////////////////////////////////////////////////////////////////////////////

static long double cos_Chebyshev_Expansion_0_1(long double x)
{
	static long double const c[] = {
		+4.200987560240514577713e-1L,  -9.358785913634965235904e-2L,
		-7.642539415723373644927e-3L,  +4.958117751796130135544e-3L,
		-9.750236036106120253456e-4L,  +1.075201474958704192865e-4L,
		-4.415344769301324238886e-6L,  -7.861633919783064216022e-7L,
		+1.919240966215861471754e-7L,  -2.175775608982741065385e-8L,
		+1.296559541430849437217e-9L,  +2.207205095025162212169e-11L,
		-1.479219615873704298874e-11L, +1.821350127295808288614e-12L,
		-1.228919312990171362342e-13L, +2.227139250593818235212e-15L,
		+5.734729405928016301596e-16L, -8.284965573075354177016e-17L,
		+6.067422701530157308321e-18L, -1.994908519477689596319e-19L,
		-1.173365630675305693390e-20L
	};

	static const int degree = sizeof(c) / sizeof(long double) - 1;
	static const long double midpoint = 0.5L;
	static const long double scale = 0.5L;

	return xChebyshev_Tn_Series((x - midpoint) / scale, c, degree);
}


////////////////////////////////////////////////////////////////////////////////
// static long double Chebyshev_Expansion_1_3( long double x )                //
//                                                                            //
//  Description:                                                              //
//     Evaluate the Fresnel auxiliary cosine integral, f(x), on the interval  //
//     1 < x <= 3 using the Chebyshev interpolation formula.                  //
//                                                                            //
//  Arguments:                                                                //
//     long double  x  The argument of the Fresnel auxiliary cosine integral  //
//                     where 1 < x <= 3.                                      //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel auxiliary cosine integral f evaluated at      //
//     x where 1 < x <= 3.                                                    //
//                                                                            //
//  Example:                                                                  //
//     long double y, x;                                                      //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = Chebyshev_Expansion_1_3(x);                                        //
////////////////////////////////////////////////////////////////////////////////

static long double cos_Chebyshev_Expansion_1_3(long double x)
{
	static long double const c[] = {
		+2.098677278318224971989e-1L,  -9.314234883154103266195e-2L,
		+1.739905936938124979297e-2L,  -2.454274824644285136137e-3L,
		+1.589872606981337312438e-4L,  +4.203943842506079780413e-5L,
		-2.018022256093216535093e-5L,  +5.125709636776428285284e-6L,
		-9.601813551752718650057e-7L,  +1.373989484857155846826e-7L,
		-1.348105546577211255591e-8L,  +2.745868700337953872632e-10L,
		+2.401655517097260106976e-10L, -6.678059547527685587692e-11L,
		+1.140562171732840809159e-11L, -1.401526517205212219089e-12L,
		+1.105498827380224475667e-13L, +2.040731455126809208066e-16L,
		-1.946040679213045143184e-15L, +4.151821375667161733612e-16L,
		-5.642257647205149369594e-17L, +5.266176626521504829010e-18L,
		-2.299025577897146333791e-19L, -2.952226367506641078731e-20L,
		+8.760405943193778149078e-21L
	};

	static const int degree = sizeof(c) / sizeof(long double) - 1;
	static const long double midpoint = 2.0L;

	return xChebyshev_Tn_Series((x - midpoint), c, degree);

}


////////////////////////////////////////////////////////////////////////////////
// static long double Chebyshev_Expansion_3_5( long double x )                //
//                                                                            //
//  Description:                                                              //
//     Evaluate the Fresnel auxiliary cosine integral, g(x), on the interval  //
//     3 < x <= 5 using the Chebyshev interpolation formula.                  //
//                                                                            //
//  Arguments:                                                                //
//     long double  x  The argument of the Fresnel auxiliary cosine integral  //
//                     where 3 < x <= 5.                                      //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel auxiliary cosine integral f evaluated at      //
//     x where 3 < x <= 5.                                                    //
//                                                                            //
//  Example:                                                                  //
//     long double y, x;                                                      //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = Chebyshev_Expansion_3_5(x);                                        //
////////////////////////////////////////////////////////////////////////////////

static long double cos_Chebyshev_Expansion_3_5(long double x)
{
	static long double const c[] = {
		+1.025703371090289562388e-1L,  -2.569833023232301400495e-2L,
		+3.160592981728234288078e-3L,  -3.776110718882714758799e-4L,
		+4.325593433537248833341e-5L,  -4.668447489229591855730e-6L,
		+4.619254757356785108280e-7L,  -3.970436510433553795244e-8L,
		+2.535664754977344448598e-9L,  -2.108170964644819803367e-11L,
		-2.959172018518707683013e-11L, +6.727219944906606516055e-12L,
		-1.062829587519902899001e-12L, +1.402071724705287701110e-13L,
		-1.619154679722651005075e-14L, +1.651319588396970446858e-15L,
		-1.461704569438083772889e-16L, +1.053521559559583268504e-17L,
		-4.760946403462515858756e-19L, -1.803784084922403924313e-20L,
		+7.873130866418738207547e-21L
	};

	static const int degree = sizeof(c) / sizeof(long double) - 1;
	static const long double midpoint = 4.0L;

	return xChebyshev_Tn_Series((x - midpoint), c, degree);
}


////////////////////////////////////////////////////////////////////////////////
// static long double Chebyshev_Expansion_5_7( long double x )                //
//                                                                            //
//  Description:                                                              //
//     Evaluate the Fresnel auxiliary cosine integral, g(x), on the interval  //
//     5 < x <= 7 using the Chebyshev interpolation formula.                  //
//                                                                            //
//  Arguments:                                                                //
//     long double  x  The argument of the Fresnel auxiliary cosine integral  //
//                     where 5 < x <= 7.                                      //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel auxiliary cosine integral f evaluated at      //
//     x where 5 < x <= 7.                                                    //
//                                                                            //
//  Example:                                                                  //
//     long double y, x;                                                      //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = Chebyshev_Expansion_5_7(x);                                        //
////////////////////////////////////////////////////////////////////////////////

static long double cos_Chebyshev_Expansion_5_7(long double x)
{
	static long double const c[] = {
		+6.738667333400589274018e-2L,  -1.128146832637904868638e-2L,
		+9.408843234170404670278e-4L,  -7.800074103496165011747e-5L,
		+6.409101169623350885527e-6L,  -5.201350558247239981834e-7L,
		+4.151668914650221476906e-8L,  -3.242202015335530552721e-9L,
		+2.460339340900396789789e-10L, -1.796823324763304661865e-11L,
		+1.244108496436438952425e-12L, -7.950417122987063540635e-14L,
		+4.419142625999150971878e-15L, -1.759082736751040110146e-16L,
		-1.307443936270786700760e-18L, +1.362484141039320395814e-18L,
		-2.055236564763877250559e-19L, +2.329142055084791308691e-20L,
		-2.282438671525884861970e-21L
	};

	static const int degree = sizeof(c) / sizeof(long double) - 1;
	static const long double midpoint = 6.0L;

	return xChebyshev_Tn_Series((x - midpoint), c, degree);

}


////////////////////////////////////////////////////////////////////////////////
// static long double Asymptotic_Series( long double x )                      //
//                                                                            //
//  Description:                                                              //
//     For a large argument x, the auxiliary Fresnel cosine integral, f(x),   //
//     can be expressed as the asymptotic series                              //
//      f(x) ~ 1/(x*sqrt(2pi))[1 - 3/4x^4 + 105/16x^8 + ... +                 //
//                                                (4j-1)!!/(-4x^4)^j + ... ]  //
//                                                                            //
//  Arguments:                                                                //
//     long double  x  The argument of the Fresnel auxiliary cosine integral  //
//                     where x > 7.                                           //
//                                                                            //
//  Return Value:                                                             //
//     The value of the Fresnel auxiliary cosine integral f evaluated at      //
//     x where x > 7.                                                         //
//                                                                            //
//  Example:                                                                  //
//     long double y, x;                                                      //
//                                                                            //
//     ( code to initialize x )                                               //
//                                                                            //
//     y = Asymptotic_Series( x );                                            //
////////////////////////////////////////////////////////////////////////////////
#define NUM_ASYMPTOTIC_TERMS 35
static long double cos_Asymptotic_Series(long double x)
{
	long double x2 = x * x;
	long double x4 = -4.0L * x2 * x2;
	long double xn = 1.0L;
	long double factorial = 1.0L;
	long double f = 0.0L;
	long double term[NUM_ASYMPTOTIC_TERMS + 1];
	long double epsilon = LDBL_EPSILON / 4.0L;
	int j = 3;
	int i = 0;

	term[0] = 1.0L;
	term[NUM_ASYMPTOTIC_TERMS] = 0.0L;
	for (i = 1; i < NUM_ASYMPTOTIC_TERMS; i++) {
		factorial *= ((long double)j * (long double)(j - 2));
		xn *= x4;
		term[i] = factorial / xn;
		j += 4;
		if (fabsl(term[i]) >= fabsl(term[i - 1])) {
			i--;
			break;
		}
		if (fabsl(term[i]) <= epsilon) break;
	}

	for (; i >= 0; i--) f += term[i];

	return f / (x * sqrt_2pi);
}


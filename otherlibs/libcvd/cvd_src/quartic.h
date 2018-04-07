#ifndef QUARTIC_H
#define QUARTIC_H

namespace CVD {

/// A function to evaluate x^4 + Bx^3 + Cx^2 + Dx + E
inline double eval_quartic(double B, double C, double D, double E, double x)
{
    return E + x*(D + x*(C + x*(B + x)));
}

/// A function that performs one iteration of Newton's method on the quartic x^4 + Bx^3 + Cx^2 + Dx + E
inline double newton_quartic(double B, double C, double D, double E, double x)
{
    double fx = E + x*(D + x*(C + x*(B + x)));
    double dx = D + x*(2*C + x*(3*B + x*4));
    return x - fx/dx;
}

/// A function to find the real roots of a quartic polynomial x^4 + Bx^3 + Cx^2 + Dx + E.
/// It efficiently implements the quartic formula as given by Cardano, Harriot, et al.
/// The precision of the resulting roots depends on the nature of the coefficients. 
/// Sufficient precision can be ensured by refining the resulting roots using Newton's method.
/// @param[in] B the coefficient of the cubic term
/// @param[in] C the coefficient of the quadratic term
/// @param[in] D the coefficient of the linear term
/// @param[in] E the coefficient of the constant term
/// @param[out] r an array in which 0, 2, or 4 real roots will be stored
/// @return the number of real roots
/// @ingroup helpersgroup
int find_quartic_real_roots(double B, double C, double D, double E, double r[]);

}

#endif

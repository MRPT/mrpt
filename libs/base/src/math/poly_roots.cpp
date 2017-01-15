/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/math/poly_roots.h>
#include <cmath>

// Based on:
// poly.cpp : solution of cubic and quartic equation
// (c) Khashin S.I. http://math.ivanovo.ac.ru/dalgebra/Khashin/index.html
// khash2 (at) gmail.com
//

#define	TwoPi  6.28318530717958648
const double eps=1e-14;

//---------------------------------------------------------------------------
// x - array of size 3
// In case 3 real roots: => x[0], x[1], x[2], return 3
//         2 real roots: x[0], x[1],          return 2
//         1 real root : x[0], x[1] +- i*x[2], return 1
int mrpt::math::solve_poly3(double *x,double a,double b,double c)  MRPT_NO_THROWS {	// solve cubic equation x^3 + a*x^2 + b*x + c
	double a2 = a*a;
	double q  = (a2 - 3*b)/9;
	double r  = (a*(2*a2-9*b) + 27*c)/54;
	double r2 = r*r;
	double q3 = q*q*q;
	double A,B;
	if(r2<q3) {
		double t=r/sqrt(q3);
		if( t<-1) t=-1;
		if( t> 1) t= 1;
		t=acos(t);
		a/=3; q=-2*sqrt(q);
		x[0]=q*cos(t/3)-a;
		x[1]=q*cos((t+TwoPi)/3)-a;
		x[2]=q*cos((t-TwoPi)/3)-a;
		return(3);
	} else {
		A =-pow(std::abs(r)+sqrt(r2-q3),1./3);
		if( r<0 ) A=-A;
		B = A==0? 0 : q/A;

		a/=3;
		x[0] =(A+B)-a;
		x[1] =-0.5*(A+B)-a;
		x[2] = 0.5*sqrt(3.)*(A-B);
		if(std::abs(x[2])<eps) { x[2]=x[1]; return(2); }
		return(1);
	}
}// SolveP3(double *x,double a,double b,double c) {
//---------------------------------------------------------------------------
// a>=0!
void  CSqrt( double x, double y, double &a, double &b) // returns:  a+i*s = sqrt(x+i*y)
{
	double r  = sqrt(x*x+y*y);
	if( y==0 ) {
		r = sqrt(r);
		if(x>=0) { a=r; b=0; } else { a=0; b=r; }
	} else {		// y != 0
		a = sqrt(0.5*(x+r));
		b = 0.5*y/a;
	}
}
//---------------------------------------------------------------------------
int   SolveP4Bi(double *x, double b, double d)	// solve equation x^4 + b*x^2 + d = 0
{
	double D = b*b-4*d;
	if( D>=0 )
	{
		double sD = sqrt(D);
		double x1 = (-b+sD)/2;
		double x2 = (-b-sD)/2;	// x2 <= x1
		if( x2>=0 )				// 0 <= x2 <= x1, 4 real roots
		{
			double sx1 = sqrt(x1);
			double sx2 = sqrt(x2);
			x[0] = -sx1;
			x[1] =  sx1;
			x[2] = -sx2;
			x[3] =  sx2;
			return 4;
		}
		if( x1 < 0 )				// x2 <= x1 < 0, two pair of imaginary roots
		{
			double sx1 = sqrt(-x1);
			double sx2 = sqrt(-x2);
			x[0] =    0;
			x[1] =  sx1;
			x[2] =    0;
			x[3] =  sx2;
			return 0;
		}
		// now x2 < 0 <= x1 , two real roots and one pair of imginary root
		double sx1 = sqrt( x1);
		double sx2 = sqrt(-x2);
		x[0] = -sx1;
		x[1] =  sx1;
		x[2] =    0;
		x[3] =  sx2;
		return 2;
	} else { // if( D < 0 ), two pair of compex roots
		double sD2 = 0.5*sqrt(-D);
		CSqrt(-0.5*b, sD2, x[0],x[1]);
		CSqrt(-0.5*b,-sD2, x[2],x[3]);
		return 0;
	} // if( D>=0 )
} // SolveP4Bi(double *x, double b, double d)	// solve equation x^4 + b*x^2 d
//---------------------------------------------------------------------------
#define SWAP(a,b) { t=b; b=a; a=t; }
static void  dblSort3( double &a, double &b, double &c) // make: a <= b <= c
{
	double t;
	if( a>b ) SWAP(a,b);	// now a<=b
	if( c<b ) {
		SWAP(b,c);			// now a<=b, b<=c
		if( a>b ) SWAP(a,b);// now a<=b
	}
}
//---------------------------------------------------------------------------
int   SolveP4De(double *x, double b, double c, double d)	// solve equation x^4 + b*x^2 + c*x + d
{
	//if( c==0 ) return SolveP4Bi(x,b,d); // After that, c!=0
	if( std::abs(c)<1e-14*(std::abs(b)+std::abs(d)) ) return SolveP4Bi(x,b,d); // After that, c!=0

	int res3 = mrpt::math::solve_poly3( x, 2*b, b*b-4*d, -c*c);	// solve resolvent
	// by Viet theorem:  x1*x2*x3=-c*c not equals to 0, so x1!=0, x2!=0, x3!=0
	if( res3>1 )	// 3 real roots,
	{
		dblSort3(x[0], x[1], x[2]);	// sort roots to x[0] <= x[1] <= x[2]
		// Note: x[0]*x[1]*x[2]= c*c > 0
		if( x[0] > 0) // all roots are positive
		{
			double sz1 = sqrt(x[0]);
			double sz2 = sqrt(x[1]);
			double sz3 = sqrt(x[2]);
			// Note: sz1*sz2*sz3= -c (and not equal to 0)
			if( c>0 )
			{
				x[0] = (-sz1 -sz2 -sz3)/2;
				x[1] = (-sz1 +sz2 +sz3)/2;
				x[2] = (+sz1 -sz2 +sz3)/2;
				x[3] = (+sz1 +sz2 -sz3)/2;
				return 4;
			}
			// now: c<0
			x[0] = (-sz1 -sz2 +sz3)/2;
			x[1] = (-sz1 +sz2 -sz3)/2;
			x[2] = (+sz1 -sz2 -sz3)/2;
			x[3] = (+sz1 +sz2 +sz3)/2;
			return 4;
		} // if( x[0] > 0) // all roots are positive
		// now x[0] <= x[1] < 0, x[2] > 0
		// two pair of comlex roots
		double sz1 = sqrt(-x[0]);
		double sz2 = sqrt(-x[1]);
		double sz3 = sqrt( x[2]);

		if( c>0 )	// sign = -1
		{
			x[0] = -sz3/2;
			x[1] = ( sz1 -sz2)/2;		// x[0]±i*x[1]
			x[2] =  sz3/2;
			x[3] = (-sz1 -sz2)/2;		// x[2]±i*x[3]
			return 0;
		}
		// now: c<0 , sign = +1
		x[0] =   sz3/2;
		x[1] = (-sz1 +sz2)/2;
		x[2] =  -sz3/2;
		x[3] = ( sz1 +sz2)/2;
		return 0;
	} // if( res3>1 )	// 3 real roots,
	// now resoventa have 1 real and pair of compex roots
	// x[0] - real root, and x[0]>0,
	// x[1]±i*x[2] - complex roots,
	double sz1 = sqrt(x[0]);
	double szr, szi;
	CSqrt(x[1], x[2], szr, szi);  // (szr+i*szi)^2 = x[1]+i*x[2]
	if( c>0 )	// sign = -1
	{
		x[0] = -sz1/2-szr;			// 1st real root
		x[1] = -sz1/2+szr;			// 2nd real root
		x[2] = sz1/2;
		x[3] = szi;
		return 2;
	}
	// now: c<0 , sign = +1
	x[0] = sz1/2-szr;			// 1st real root
	x[1] = sz1/2+szr;			// 2nd real root
	x[2] = -sz1/2;
	x[3] = szi;
	return 2;
} // SolveP4De(double *x, double b, double c, double d)	// solve equation x^4 + b*x^2 + c*x + d
//-----------------------------------------------------------------------------
double N4Step(double x, double a,double b,double c,double d)	// one Newton step for x^4 + a*x^3 + b*x^2 + c*x + d
{
	double fxs= ((4*x+3*a)*x+2*b)*x+c;	// f'(x)
	if( fxs==0 ) return 1e99;
	double fx = (((x+a)*x+b)*x+c)*x+d;	// f(x)
	return x - fx/fxs;
}
//-----------------------------------------------------------------------------
// x - array of size 4
// return 4: 4 real roots x[0], x[1], x[2], x[3], possible multiple roots
// return 2: 2 real roots x[0], x[1] and complex x[2]±i*x[3],
// return 0: two pair of complex roots: x[0]+-i*x[1],  x[2]+-i*x[3],
int  mrpt::math::solve_poly4(double *x,double a,double b,double c,double d)  MRPT_NO_THROWS {	// solve equation x^4 + a*x^3 + b*x^2 + c*x + d by Dekart-Euler method
	// move to a=0:
	double d1 = d + 0.25*a*( 0.25*b*a - 3./64*a*a*a - c);
	double c1 = c + 0.5*a*(0.25*a*a - b);
	double b1 = b - 0.375*a*a;
	int res = SolveP4De( x, b1, c1, d1);
	if( res==4) { x[0]-= a/4; x[1]-= a/4; x[2]-= a/4; x[3]-= a/4; }
	else if (res==2) { x[0]-= a/4; x[1]-= a/4; x[2]-= a/4; }
	else             { x[0]-= a/4; x[2]-= a/4; }
	// one Newton step for each real root:
	if( res>0 )
	{
		x[0] = N4Step(x[0], a,b,c,d);
		x[1] = N4Step(x[1], a,b,c,d);
	}
	if( res>2 )
	{
		x[2] = N4Step(x[2], a,b,c,d);
		x[3] = N4Step(x[3], a,b,c,d);
	}
	return res;
}
//-----------------------------------------------------------------------------
#define F5(t) (((((t+a)*t+b)*t+c)*t+d)*t+e)
//-----------------------------------------------------------------------------
static double SolveP5_1(double a,double b,double c,double d,double e)	// return real root of x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0
{
	int cnt;
	if( std::abs(e)<eps ) return 0;

	double brd =  std::abs(a);			// brd - border of real roots
	if( std::abs(b)>brd ) brd = std::abs(b);
	if( std::abs(c)>brd ) brd = std::abs(c);
	if( std::abs(d)>brd ) brd = std::abs(d);
	if( std::abs(e)>brd ) brd = std::abs(e);
	brd++;							// brd - border of real roots

	double x0, f0;					// less, than root
	double x1, f1;					// greater, than root
	double x2, f2, f2s;				// next values, f(x2), f'(x2)
	double dx=1e8;

	if( e<0 ) { x0 =   0; x1 = brd; f0=e; f1=F5(x1); x2 = 0.01*brd; }
	else	  { x0 =-brd; x1 =   0; f0=F5(x0); f1=e; x2 =-0.01*brd; }

	if( std::abs(f0)<eps ) return x0;
	if( std::abs(f1)<eps ) return x1;

	// now x0<x1, f(x0)<0, f(x1)>0
	// Firstly 5 bisections
	for( cnt=0; cnt<5; cnt++)
	{
		x2 = (x0 + x1)/2;			// next point
		f2 = F5(x2);				// f(x2)
		if( std::abs(f2)<eps ) return x2;
		if( f2>0 ) { x1=x2; f1=f2; }
		else       { x0=x2; f0=f2; }
	}

	// At each step:
	// x0<x1, f(x0)<0, f(x1)>0.
	// x2 - next value
	// we hope that x0 < x2 < x1, but not necessarily
	do {
		cnt++;
		if( x2<=x0 || x2>= x1 ) x2 = (x0 + x1)/2;	// now  x0 < x2 < x1
		f2 = F5(x2);								// f(x2)
		if( std::abs(f2)<eps ) return x2;
		if( f2>0 ) { x1=x2; f1=f2; }
		else       { x0=x2; f0=f2; }
		f2s= (((5*x2+4*a)*x2+3*b)*x2+2*c)*x2+d;		// f'(x2)
		if( std::abs(f2s)<eps ) { x2=1e99; continue; }
		dx = f2/f2s;
		x2 -= dx;
	} while(std::abs(dx)>eps);
	return x2;
} // SolveP5_1(double a,double b,double c,double d,double e)	// return real root of x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0
//-----------------------------------------------------------------------------
int  mrpt::math::solve_poly5(double *x,double a,double b,double c,double d,double e)  MRPT_NO_THROWS	// solve equation x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0
{
	double r = x[0] = SolveP5_1(a,b,c,d,e);
	double a1 = a+r, b1=b+r*a1, c1=c+r*b1, d1=d+r*c1;
	return 1+solve_poly4(x+1, a1,b1,c1,d1);
} // SolveP5(double *x,double a,double b,double c,double d,double e)	// solve equation x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0
//-----------------------------------------------------------------------------

// a*x^2 + b*x + c = 0
int mrpt::math::solve_poly2(double a, double b, double c, double &r1, double &r2)  MRPT_NO_THROWS
{
	if (std::abs(a)<eps)
	{
		// b*x+c=0
		if (std::abs(b)<eps) 
			return 0;
		r1 = -c/b;
		r2 =1e99;
		return 1;
	}
	else 
	{
		double Di =  b*b - 4*a*c;
		if( Di<0 ) { r1=r2=1e99; return 0; }
		Di =  sqrt(Di);
		r1 =  (-b + Di)/(2*a);
		r2 =  (-b - Di)/(2*a);
		// We ensure at output that r1 <= r2
		double t; // temp:
		if (r2<r1) SWAP(r1,r2);
		return 2;
	}
}

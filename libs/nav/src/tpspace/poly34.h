// poly34.h : solution of cubic and quartic equation
// (c) Khashin S.I. http://math.ivanovo.ac.ru/dalgebra/Khashin/index.html
// khash2 (at) gmail.com


int   SolveP3(double *x,double a,double b,double c);			// solve cubic equation x^3 + a*x^2 + b*x + c = 0
int   SolveP4(double *x,double a,double b,double c,double d);	// solve equation x^4 + a*x^3 + b*x^2 + c*x + d = 0 by Dekart-Euler method
// x - array of size 4
// return 4: 4 real roots x[0], x[1], x[2], x[3], possible multiple roots
// return 2: 2 real roots x[0], x[1] and complex x[2]±i*x[3], 
// return 0: two pair of complex roots: x[0]±i*x[1],  x[2]±i*x[3], 
int   SolveP5(double *x,double a,double b,double c,double d,double e);	// solve equation x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0

int   SolveP4Bi(double *x, double b, double d);				// solve equation x^4 + b*x^2 + d = 0
int   SolveP4De(double *x, double b, double c, double d);	// solve equation x^4 + b*x^2 + c*x + d = 0
void  CSqrt( double x, double y, double &a, double &b);		// returns as a+i*s,  sqrt(x+i*y)
double N4Step(double x, double a,double b,double c,double d);// one Newton step for x^4 + a*x^3 + b*x^2 + c*x + d

	double SolveP5_1(double a,double b,double c,double d,double e);	// return real root of x^5 + a*x^4 + b*x^3 + c*x^2 + d*x + e = 0

// Solve2: let f(x ) = a*x^2 + b*x + c and 
//     f(x0) = f0,
//     f(x1) = f1,
//     f(x2) = f3
// Then r1, r2 - root of f(x)=0.
// Returns 0, if there are no roots, else return 2.
int Solve2( double x0, double x1, double x2, double f0, double f1, double f2, double &r1, double &r2); 


// This file added by arh, for compatibility with MSVC++ 6.0

#ifndef __VALUES_H
#define __VALUES_H


#ifdef WIN32
#ifndef MAXDOUBLE
#include <float.h>
#define MAXDOUBLE DBL_MAX
#endif
#else
#include <values.h>
#endif



#endif // __VALUES_H

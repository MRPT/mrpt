/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef _PSTDINT_H_INCLUDED
#include <stddef.h>
#include <limits.h>

/*
 *  For gcc with _STDINT_H, fill in the PRINTF_INT*_MODIFIER macros, and
 *  do nothing else.  On the Mac OS X version of gcc this is _STDINT_H_.
 */

#if (                                                                  \
	defined(__STDC__) && __STDC__ && defined(__STDC_VERSION__) &&      \
	__STDC_VERSION__ >= 199901L) ||                                    \
	(defined(__WATCOMC__) && __WATCOMC__ >= 1250) ||                   \
	(defined(__GNUC__) && __GNUC__ > 2) ||                             \
	(defined(_MSC_VER) && _MSC_VER >= 1600) || defined(IAR_ARM_CM3) || \
	(defined(__ICCARM__) && __ICCARM__) ||                             \
	(defined(_ADI_COMPILER) && _ADI_COMPILER) ||                       \
	(defined(__ICC8051__) && __ICC8051__)
#include <stdint.h>
#endif

#if (                                                                        \
	defined(_STDINT_H_INCLUDED) || defined(_STDINT) || defined(_STDINT_H) || \
	defined(_STDINT_H_) || defined(BOOST_CSTDINT_HPP)) &&                    \
	!defined(_PSTDINT_H_INCLUDED)
#define _PSTDINT_H_INCLUDED
#ifndef PRINTF_INT64_MODIFIER
#if (__WORDSIZE == 64) && !defined(__APPLE__)  // lint !e553 __WORDSIZE should
// always be defined at this
// point
#define PRINTF_INT64_MODIFIER "l"
#else
#define PRINTF_INT64_MODIFIER "ll"
#endif
#endif
#ifndef PRINTF_INT32_MODIFIER
/* There might be cases where this needs to be "l". */
#define PRINTF_INT32_MODIFIER ""
#endif
#ifndef PRINTF_INT16_MODIFIER
#define PRINTF_INT16_MODIFIER "h"
#endif
#ifndef PRINTF_INTMAX_MODIFIER
#define PRINTF_INTMAX_MODIFIER PRINTF_INT64_MODIFIER
#endif
#ifndef PRINTF_INT64_HEX_WIDTH
#define PRINTF_INT64_HEX_WIDTH "16"
#endif
#ifndef PRINTF_INT32_HEX_WIDTH
#define PRINTF_INT32_HEX_WIDTH "8"
#endif
#ifndef PRINTF_INT16_HEX_WIDTH
#define PRINTF_INT16_HEX_WIDTH "4"
#endif
#ifndef PRINTF_INT8_HEX_WIDTH
#define PRINTF_INT8_HEX_WIDTH "2"
#endif
#ifndef PRINTF_INT64_DEC_WIDTH
#define PRINTF_INT64_DEC_WIDTH "20"
#endif
#ifndef PRINTF_INT32_DEC_WIDTH
#define PRINTF_INT32_DEC_WIDTH "10"
#endif
#ifndef PRINTF_INT16_DEC_WIDTH
#define PRINTF_INT16_DEC_WIDTH "5"
#endif
#ifndef PRINTF_INT8_DEC_WIDTH
#define PRINTF_INT8_DEC_WIDTH "3"
#endif
#ifndef PRINTF_INTMAX_HEX_WIDTH
#define PRINTF_INTMAX_HEX_WIDTH PRINTF_INT64_HEX_WIDTH
#endif
#ifndef PRINTF_INTMAX_DEC_WIDTH
#define PRINTF_INTMAX_DEC_WIDTH PRINTF_INT64_DEC_WIDTH
#endif

/*
 *  Something really weird is going on with Open Watcom.  Just pull some of
 *  these duplicated definitions from Open Watcom's stdint.h file for now.
 */

#if defined(__WATCOMC__) && __WATCOMC__ >= 1250
#if !defined(INT64_C)
#define INT64_C(x) (x + (INT64_MAX - INT64_MAX))
#endif
#if !defined(UINT64_C)
#define UINT64_C(x) (x + (UINT64_MAX - UINT64_MAX))
#endif
#if !defined(INT32_C)
#define INT32_C(x) (x + (INT32_MAX - INT32_MAX))
#endif
#if !defined(UINT32_C)
#define UINT32_C(x) (x + (UINT32_MAX - UINT32_MAX))
#endif
#if !defined(INT16_C)
#define INT16_C(x) (x)
#endif
#if !defined(UINT16_C)
#define UINT16_C(x) (x)
#endif
#if !defined(INT8_C)
#define INT8_C(x) (x)
#endif
#if !defined(UINT8_C)
#define UINT8_C(x) (x)
#endif
#if !defined(UINT64_MAX)
#define UINT64_MAX 18446744073709551615ULL
#endif
#if !defined(INT64_MAX)
#define INT64_MAX 9223372036854775807LL
#endif
#if !defined(UINT32_MAX)
#define UINT32_MAX 4294967295UL
#endif
#if !defined(INT32_MAX)
#define INT32_MAX 2147483647L
#endif
#if !defined(INTMAX_MAX)
#define INTMAX_MAX INT64_MAX
#endif
#if !defined(INTMAX_MIN)
#define INTMAX_MIN INT64_MIN
#endif
#endif

/*  Some of the targets compiled by Visual Dsp++ (like the Sharc) do not have a
   notion of 8bit of 16bit value
	We define these here although the actual storage size is 32 bits
*/
#if (defined(_ADI_COMPILER) && _ADI_COMPILER)
#ifndef int8_t
typedef signed char int8_t;
#endif
#ifndef uint8_t
typedef unsigned char uint8_t;
#endif
#ifndef int16_t
typedef signed short int16_t;
#endif
#ifndef uint16_t
typedef unsigned short uint16_t;
#endif
#endif  //(defined(_ADI_COMPILER) && _ADI_COMPILER)

#endif

#ifndef _PSTDINT_H_INCLUDED
#define _PSTDINT_H_INCLUDED

#ifndef SIZE_MAX
#ifdef DOXYGEN
#define SIZE_MAX (~0)
#else
#define SIZE_MAX (~(size_t)0)
#endif
#endif

/*
 *  Deduce the type assignments from limits.h under the assumption that
 *  integer sizes in bits are powers of 2, and follow the ANSI
 *  definitions.
 */

#ifndef UINT8_MAX
#define UINT8_MAX 0xff
#endif
#ifndef uint8_t
#if (UCHAR_MAX == UINT8_MAX) || defined(S_SPLINT_S)
typedef unsigned char uint8_t;
#define UINT8_C(v) ((uint8_t)v)
#else
#error "Platform not supported"
#endif
#endif

#ifndef INT8_MAX
#define INT8_MAX 0x7f
#endif
#ifndef INT8_MIN
#define INT8_MIN INT8_C(0x80)
#endif
#ifndef int8_t
#if (SCHAR_MAX == INT8_MAX) || defined(S_SPLINT_S)
typedef signed char int8_t;
#define INT8_C(v) ((int8_t)v)
#else
#error "Platform not supported"
#endif
#endif

#ifndef UINT16_MAX
#define UINT16_MAX 0xffff
#endif
#ifndef uint16_t
#if (UINT_MAX == UINT16_MAX) || defined(S_SPLINT_S)
typedef unsigned int uint16_t;
#ifndef PRINTF_INT16_MODIFIER
#define PRINTF_INT16_MODIFIER ""
#endif
#define UINT16_C(v) ((uint16_t)(v))
#elif (USHRT_MAX == UINT16_MAX)
typedef unsigned short uint16_t;
#define UINT16_C(v) ((uint16_t)(v))
#ifndef PRINTF_INT16_MODIFIER
#define PRINTF_INT16_MODIFIER "h"
#endif
#else
#error "Platform not supported"
#endif
#endif

#ifndef INT16_MAX
#define INT16_MAX 0x7fff
#endif
#ifndef INT16_MIN
#define INT16_MIN INT16_C(0x8000)
#endif
#ifndef int16_t
#if (INT_MAX == INT16_MAX) || defined(S_SPLINT_S)
typedef signed int int16_t;
#define INT16_C(v) ((int16_t)(v))
#ifndef PRINTF_INT16_MODIFIER
#define PRINTF_INT16_MODIFIER ""
#endif
#elif (SHRT_MAX == INT16_MAX)
typedef signed short int16_t;
#define INT16_C(v) ((int16_t)(v))
#ifndef PRINTF_INT16_MODIFIER
#define PRINTF_INT16_MODIFIER "h"
#endif
#else
#error "Platform not supported"
#endif
#endif

#ifndef UINT32_MAX
#define UINT32_MAX (0xffffffffUL)
#endif
#ifndef uint32_t
#if (SIZE_MAX == UINT32_MAX) && !defined(S_SPLINT_S) && !defined(_lint)
typedef unsigned int uint32_t;
#ifndef PRINTF_INT32_MODIFIER
#define PRINTF_INT32_MODIFIER ""
#endif
#define UINT32_C(v) v##U
#elif (ULONG_MAX == UINT32_MAX) || defined(S_SPLINT_S)
typedef unsigned long uint32_t;
#define UINT32_C(v) v##UL
#ifndef PRINTF_INT32_MODIFIER
#define PRINTF_INT32_MODIFIER "l"
#endif
#elif (USHRT_MAX == UINT32_MAX)
typedef unsigned short uint32_t;
#define UINT32_C(v) ((unsigned short)(v))
#ifndef PRINTF_INT32_MODIFIER
#define PRINTF_INT32_MODIFIER ""
#endif
#else
#error "Platform not supported"
#endif
#endif

#ifndef INT32_MAX
#define INT32_MAX (0x7fffffffL)
#endif
#ifndef INT32_MIN
#define INT32_MIN INT32_C(0x80000000)
#endif
#ifndef int32_t
#if (SIZE_MAX / 2 == INT32_MAX) && defined(__GNUC__) && (__GNUC__ > 3)
typedef signed int int32_t;
#define INT32_C(v) v
#ifndef PRINTF_INT32_MODIFIER
#define PRINTF_INT32_MODIFIER ""
#endif
#elif (LONG_MAX == INT32_MAX) || defined(S_SPLINT_S)
typedef signed long int32_t;
#define INT32_C(v) v##L
#ifndef PRINTF_INT32_MODIFIER
#define PRINTF_INT32_MODIFIER "l"
#endif
#elif (SIZE_MAX / 2 == INT32_MAX)
typedef signed int int32_t;
#define INT32_C(v) v
#ifndef PRINTF_INT32_MODIFIER
#define PRINTF_INT32_MODIFIER ""
#endif
#elif (SHRT_MAX == INT32_MAX)
typedef signed short int32_t;
#define INT32_C(v) ((short)(v))
#ifndef PRINTF_INT32_MODIFIER
#define PRINTF_INT32_MODIFIER ""
#endif
#else
#error "Platform not supported"
#endif
#endif

/*
 *  The macro stdint_int64_defined is temporarily used to record
 *  whether or not 64 integer support is available.  It must be
 *  defined for any 64 integer extensions for new platforms that are
 *  added.
 */

#undef stdint_int64_defined
#if (defined(__STDC__) && defined(__STDC_VERSION__)) || defined(S_SPLINT_S)
#if (__STDC__ && __STDC_VERSION >= 199901L) || defined(S_SPLINT_S)
#define stdint_int64_defined
typedef long long int64_t;
typedef unsigned long long uint64_t;
#define UINT64_C(v) v##ULL
#define INT64_C(v) v##LL
#ifndef PRINTF_INT64_MODIFIER
#define PRINTF_INT64_MODIFIER "ll"
#endif
#endif
#endif

#if !defined(stdint_int64_defined)
#if defined(__GNUC__) && __WORDSIZE == 64
#define stdint_int64_defined
__extension__ typedef long int64_t;
__extension__ typedef unsigned long uint64_t;
#define UINT64_C(v) v##UL
#define INT64_C(v) v##L
#ifndef PRINTF_INT64_MODIFIER
#define PRINTF_INT64_MODIFIER "l"
#endif
#elif defined(__GNUC__)
#define stdint_int64_defined
__extension__ typedef long long int64_t;
__extension__ typedef unsigned long long uint64_t;
#define UINT64_C(v) v##ULL
#define INT64_C(v) v##LL
#ifndef PRINTF_INT64_MODIFIER
#define PRINTF_INT64_MODIFIER "ll"
#endif
#elif defined(__MWERKS__) || defined(__SUNPRO_C) || defined(__SUNPRO_CC) || \
	defined(__APPLE_CC__) || defined(_LONG_LONG) || defined(_CRAYC) ||      \
	defined(S_SPLINT_S)
#define stdint_int64_defined
typedef long long int64_t;
typedef unsigned long long uint64_t;
#define UINT64_C(v) v##ULL
#define INT64_C(v) v##LL
#ifndef PRINTF_INT64_MODIFIER
#define PRINTF_INT64_MODIFIER "ll"
#endif
#elif (defined(__WATCOMC__) && defined(__WATCOM_INT64__)) ||               \
	(defined(_MSC_VER) && _INTEGRAL_MAX_BITS >= 64) ||                     \
	(defined(__BORLANDC__) && __BORLANDC__ > 0x460) || defined(__alpha) || \
	defined(__DECC)
#define stdint_int64_defined
typedef __int64 int64_t;
typedef unsigned __int64 uint64_t;
#define UINT64_C(v) v##UI64
#define INT64_C(v) v##I64
#ifndef PRINTF_INT64_MODIFIER
#define PRINTF_INT64_MODIFIER "I64"
#endif
#endif
#endif

#if !defined(LONG_LONG_MAX) && defined(INT64_C)
#define LONG_LONG_MAX INT64_C(9223372036854775807LL)
#endif
#ifndef ULONG_LONG_MAX
#define ULONG_LONG_MAX UINT64_C(18446744073709551615ULL)
#endif

#if !defined(INT64_MAX) && defined(INT64_C)
#define INT64_MAX INT64_C(9223372036854775807LL)
#endif
#if !defined(INT64_MIN) && defined(INT64_C)
// lint -e(417)
#define INT64_MIN INT64_C(-9223372036854775808LL)
#endif
#if !defined(UINT64_MAX) && defined(INT64_C)
#define UINT64_MAX UINT64_C(18446744073709551615ULL)
#endif

/*
 *  Width of hexadecimal for number field.
 */

#ifndef PRINTF_INT64_HEX_WIDTH
#define PRINTF_INT64_HEX_WIDTH "16"
#endif
#ifndef PRINTF_INT32_HEX_WIDTH
#define PRINTF_INT32_HEX_WIDTH "8"
#endif
#ifndef PRINTF_INT16_HEX_WIDTH
#define PRINTF_INT16_HEX_WIDTH "4"
#endif
#ifndef PRINTF_INT8_HEX_WIDTH
#define PRINTF_INT8_HEX_WIDTH "2"
#endif

#ifndef PRINTF_INT64_DEC_WIDTH
#define PRINTF_INT64_DEC_WIDTH "20"
#endif
#ifndef PRINTF_INT32_DEC_WIDTH
#define PRINTF_INT32_DEC_WIDTH "10"
#endif
#ifndef PRINTF_INT16_DEC_WIDTH
#define PRINTF_INT16_DEC_WIDTH "5"
#endif
#ifndef PRINTF_INT8_DEC_WIDTH
#define PRINTF_INT8_DEC_WIDTH "3"
#endif

/*
 *  Ok, lets not worry about 128 bit integers for now.  Moore's law says
 *  we don't need to worry about that until about 2040 at which point
 *  we'll have bigger things to worry about.
 */

#ifdef stdint_int64_defined
typedef int64_t intmax_t;
typedef uint64_t uintmax_t;
#define INTMAX_MAX INT64_MAX
#define INTMAX_MIN INT64_MIN
#define UINTMAX_MAX UINT64_MAX
#define UINTMAX_C(v) UINT64_C(v)
#define INTMAX_C(v) INT64_C(v)
#ifndef PRINTF_INTMAX_MODIFIER
#define PRINTF_INTMAX_MODIFIER PRINTF_INT64_MODIFIER
#endif
#ifndef PRINTF_INTMAX_HEX_WIDTH
#define PRINTF_INTMAX_HEX_WIDTH PRINTF_INT64_HEX_WIDTH
#endif
#ifndef PRINTF_INTMAX_DEC_WIDTH
#define PRINTF_INTMAX_DEC_WIDTH PRINTF_INT64_DEC_WIDTH
#endif
#else
typedef int32_t intmax_t;
typedef uint32_t uintmax_t;
#define INTMAX_MAX INT32_MAX
#define UINTMAX_MAX UINT32_MAX
#define UINTMAX_C(v) UINT32_C(v)
#define INTMAX_C(v) INT32_C(v)
#ifndef PRINTF_INTMAX_MODIFIER
#define PRINTF_INTMAX_MODIFIER PRINTF_INT32_MODIFIER
#endif
#ifndef PRINTF_INTMAX_HEX_WIDTH
#define PRINTF_INTMAX_HEX_WIDTH PRINTF_INT32_HEX_WIDTH
#endif
#ifndef PRINTF_INTMAX_DEC_WIDTH
#define PRINTF_INTMAX_DEC_WIDTH PRINTF_INT32_DEC_WIDTH
#endif
#endif

/*
 *  Because this file currently only supports platforms which have
 *  precise powers of 2 as bit sizes for the default integers, the
 *  least definitions are all trivial.  Its possible that a future
 *  version of this file could have different definitions.
 */

#ifndef stdint_least_defined
typedef int8_t int_least8_t;
typedef uint8_t uint_least8_t;
typedef int16_t int_least16_t;
typedef uint16_t uint_least16_t;
typedef int32_t int_least32_t;
typedef uint32_t uint_least32_t;
#define PRINTF_LEAST32_MODIFIER PRINTF_INT32_MODIFIER
#define PRINTF_LEAST16_MODIFIER PRINTF_INT16_MODIFIER
#define UINT_LEAST8_MAX UINT8_MAX
#define INT_LEAST8_MAX INT8_MAX
#define UINT_LEAST16_MAX UINT16_MAX
#define INT_LEAST16_MAX INT16_MAX
#define UINT_LEAST32_MAX UINT32_MAX
#define INT_LEAST32_MAX INT32_MAX
#define INT_LEAST8_MIN INT8_MIN
#define INT_LEAST16_MIN INT16_MIN
#define INT_LEAST32_MIN INT32_MIN
#ifdef stdint_int64_defined
typedef int64_t int_least64_t;
typedef uint64_t uint_least64_t;
#define PRINTF_LEAST64_MODIFIER PRINTF_INT64_MODIFIER
#define UINT_LEAST64_MAX UINT64_MAX
#define INT_LEAST64_MAX INT64_MAX
#define INT_LEAST64_MIN INT64_MIN
#endif
#endif
#undef stdint_least_defined

/*
 *  The ANSI C committee pretending to know or specify anything about
 *  performance is the epitome of misguided arrogance.  The mandate of
 *  this file is to *ONLY* ever support that absolute minimum
 *  definition of the fast integer types, for compatibility purposes.
 *  No extensions, and no attempt to suggest what may or may not be a
 *  faster integer type will ever be made in this file.  Developers are
 *  warned to stay away from these types when using this or any other
 *  stdint.h.
 */

#ifndef int_fast8_t
typedef int_least8_t int_fast8_t;
#endif
#ifndef uint_fast8_t
typedef uint_least8_t uint_fast8_t;
#endif
#ifndef int_fast16_t
typedef int_least16_t int_fast16_t;
#endif
#ifndef uint_fast16_t
typedef uint_least16_t uint_fast16_t;
#endif
#ifndef int_fast32_t
typedef int_least32_t int_fast32_t;
#endif
#ifndef uint_fast32_t
typedef uint_least32_t uint_fast32_t;
#endif
#define UINT_FAST8_MAX UINT_LEAST8_MAX
#define INT_FAST8_MAX INT_LEAST8_MAX
#define UINT_FAST16_MAX UINT_LEAST16_MAX
#define INT_FAST16_MAX INT_LEAST16_MAX
#define UINT_FAST32_MAX UINT_LEAST32_MAX
#define INT_FAST32_MAX INT_LEAST32_MAX
#define INT_FAST8_MIN INT_LEAST8_MIN
#define INT_FAST16_MIN INT_LEAST16_MIN
#define INT_FAST32_MIN INT_LEAST32_MIN
#ifdef stdint_int64_defined
typedef int_least64_t int_fast64_t;
typedef uint_least64_t uint_fast64_t;
#define UINT_FAST64_MAX UINT_LEAST64_MAX
#define INT_FAST64_MAX INT_LEAST64_MAX
#define INT_FAST64_MIN INT_LEAST64_MIN
#endif

/*
 *  Whatever piecemeal, per compiler thing we can do about the wchar_t
 *  type limits.
 */

#if defined(__WATCOMC__) || defined(_MSC_VER) || defined(__GNUC__)
#include <wchar.h>
#ifndef WCHAR_MIN
#define WCHAR_MIN 0
#endif
#ifndef WCHAR_MAX
#define WCHAR_MAX ((wchar_t)-1)
#endif
#endif

/*
 *  Whatever piecemeal, per compiler/platform thing we can do about the
 *  (u)intptr_t types and limits.
 */

#if defined(_MSC_VER) && defined(_UINTPTR_T_DEFINED)
#define STDINT_H_UINTPTR_T_DEFINED
#endif

#ifndef STDINT_H_UINTPTR_T_DEFINED
#if defined(__alpha__) || defined(__ia64__) || defined(__x86_64__) || \
	defined(_WIN64)
#define stdint_intptr_bits 64
#elif defined(__WATCOMC__) || defined(__TURBOC__)
#if defined(__TINY__) || defined(__SMALL__) || defined(__MEDIUM__)
#define stdint_intptr_bits 16
#else
#define stdint_intptr_bits 32
#endif
#elif defined(__i386__) || defined(_WIN32) || defined(WIN32)
#define stdint_intptr_bits 32
#elif defined(__INTEL_COMPILER)
/* TODO -- what will Intel do about x86-64? */
#endif

#ifdef stdint_intptr_bits
#define stdint_intptr_glue3_i(a, b, c) a##b##c
#define stdint_intptr_glue3(a, b, c) stdint_intptr_glue3_i(a, b, c)
#ifndef PRINTF_INTPTR_MODIFIER
#define PRINTF_INTPTR_MODIFIER \
	stdint_intptr_glue3(PRINTF_INT, stdint_intptr_bits, _MODIFIER)
#endif
#ifndef PTRDIFF_MAX
#define PTRDIFF_MAX stdint_intptr_glue3(INT, stdint_intptr_bits, _MAX)
#endif
#ifndef PTRDIFF_MIN
#define PTRDIFF_MIN stdint_intptr_glue3(INT, stdint_intptr_bits, _MIN)
#endif
#ifndef UINTPTR_MAX
#define UINTPTR_MAX stdint_intptr_glue3(UINT, stdint_intptr_bits, _MAX)
#endif
#ifndef INTPTR_MAX
#define INTPTR_MAX stdint_intptr_glue3(INT, stdint_intptr_bits, _MAX)
#endif
#ifndef INTPTR_MIN
#define INTPTR_MIN stdint_intptr_glue3(INT, stdint_intptr_bits, _MIN)
#endif
#ifndef INTPTR_C
#define INTPTR_C(x) stdint_intptr_glue3(INT, stdint_intptr_bits, _C)(x)
#endif
#ifndef UINTPTR_C
#define UINTPTR_C(x) stdint_intptr_glue3(UINT, stdint_intptr_bits, _C)(x)
#endif
typedef stdint_intptr_glue3(uint, stdint_intptr_bits, _t) uintptr_t;
typedef stdint_intptr_glue3(int, stdint_intptr_bits, _t) intptr_t;
#else
/* TODO -- This following is likely wrong for some platforms, and does
   nothing for the definition of uintptr_t. */
typedef ptrdiff_t intptr_t;
#endif
#define STDINT_H_UINTPTR_T_DEFINED
#endif

/*
 *  Assumes sig_atomic_t is signed and we have a 2s complement machine.
 */

#ifndef SIG_ATOMIC_MAX
#define SIG_ATOMIC_MAX \
	((((sig_atomic_t)1) << (sizeof(sig_atomic_t) * CHAR_BIT - 1)) - 1)
#endif

#endif

#if defined(__TEST_PSTDINT_FOR_CORRECTNESS)

/*
 *  Please compile with the maximum warning settings to make sure macros are not
 *  defined more than once.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define glue3_aux(x, y, z) x##y##z
#define glue3(x, y, z) glue3_aux(x, y, z)

#define DECLU(bits) \
	glue3(uint, bits, _t) glue3(u, bits, =) glue3(UINT, bits, _C)(0);
#define DECLI(bits) \
	glue3(int, bits, _t) glue3(i, bits, =) glue3(INT, bits, _C)(0);

#define DECL(us, bits) glue3(DECL, us, )(bits)

#define TESTUMAX(bits)                              \
	glue3(u, bits, =) glue3(~, u, bits);            \
	if (glue3(UINT, bits, _MAX) glue3(!=, u, bits)) \
	printf("Something wrong with UINT%d_MAX\n", bits)

int main()
{
	DECL(I, 8)
	DECL(U, 8)
	DECL(I, 16)
	DECL(U, 16)
	DECL(I, 32)
	DECL(U, 32)
#ifdef INT64_MAX
	DECL(I, 64)
	DECL(U, 64)
#endif
	intmax_t imax = INTMAX_C(0);
	uintmax_t umax = UINTMAX_C(0);
	char str0[256], str1[256];

	sprintf(str0, "%d %x\n", 0, ~0);

	sprintf(str1, "%d %x\n", i8, ~0);
	if (0 != strcmp(str0, str1)) printf("Something wrong with i8 : %s\n", str1);
	sprintf(str1, "%u %x\n", u8, ~0);
	if (0 != strcmp(str0, str1)) printf("Something wrong with u8 : %s\n", str1);
	sprintf(str1, "%d %x\n", i16, ~0);
	if (0 != strcmp(str0, str1))
		printf("Something wrong with i16 : %s\n", str1);
	sprintf(str1, "%u %x\n", u16, ~0);
	if (0 != strcmp(str0, str1))
		printf("Something wrong with u16 : %s\n", str1);
	sprintf(str1, "%" PRINTF_INT32_MODIFIER "d %x\n", i32, ~0);
	if (0 != strcmp(str0, str1))
		printf("Something wrong with i32 : %s\n", str1);
	sprintf(str1, "%" PRINTF_INT32_MODIFIER "u %x\n", u32, ~0);
	if (0 != strcmp(str0, str1))
		printf("Something wrong with u32 : %s\n", str1);
#ifdef INT64_MAX
	sprintf(str1, "%" PRINTF_INT64_MODIFIER "d %x\n", i64, ~0);
	if (0 != strcmp(str0, str1))
		printf("Something wrong with i64 : %s\n", str1);
#endif
	sprintf(str1, "%" PRINTF_INTMAX_MODIFIER "d %x\n", imax, ~0);
	if (0 != strcmp(str0, str1))
		printf("Something wrong with imax : %s\n", str1);
	sprintf(str1, "%" PRINTF_INTMAX_MODIFIER "u %x\n", umax, ~0);
	if (0 != strcmp(str0, str1))
		printf("Something wrong with umax : %s\n", str1);

	TESTUMAX(8);
	TESTUMAX(16);
	TESTUMAX(32);
#ifdef INT64_MAX
	TESTUMAX(64);
#endif

	return EXIT_SUCCESS;
}

#endif

#if defined(_STDINT) && defined(_MSC_VER)
#define stdint_int64_defined
#endif

#endif  // file guard

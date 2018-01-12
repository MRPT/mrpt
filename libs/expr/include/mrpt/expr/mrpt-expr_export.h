/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#ifdef _MSC_VER
#pragma warning(disable : 4251)  // DLL export private fields
#endif

#if defined(MRPT_EXPR_STATIC_DEFINE) || !defined(_WIN32)
#define MRPT_EXPR_EXPORT
#define MRPT_EXPR_NO_EXPORT
#else
#ifndef MRPT_EXPR_EXPORT
#ifdef mrpt_expr_EXPORTS
/* We are building this library */
#define MRPT_EXPR_EXPORT __declspec(dllexport)
#else
/* We are using this library */
#define MRPT_EXPR_EXPORT __declspec(dllimport)
#endif
#endif

#ifndef MRPT_EXPR_NO_EXPORT
#define MRPT_EXPR_NO_EXPORT
#endif
#endif

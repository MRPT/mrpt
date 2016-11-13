/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

/** \file Macros to help implementing the PIMPL idiom and make all 
 * declarations easier to read and less error-prone. 
 */

#define PIMPL_DECLARE_TYPE(_TYPE, _VAR_NAME)  void *_VAR_NAME

#define PIMPL_CTOR_INIT(_TYPE, _VAR_NAME)  _VAR_NAME(NULL)

#define PIMPL_DESTROY(_TYPE,_VAR_NAME)  \
	if (_VAR_NAME) { delete reinterpret_cast<_TYPE*>(_VAR_NAME); _VAR_NAME=NULL; }

#define PIMPL_CONSTRUCT(_TYPE,_VAR_NAME)  \
	PIMPL_DESTROY(_TYPE,_VAR_NAME) \
	_VAR_NAME = new _TYPE();

#define PIMPL_GET_REF(_TYPE, _VAR_NAME)  (*reinterpret_cast<_TYPE*>(_VAR_NAME))
#define PIMPL_GET_CONSTREF(_TYPE, _VAR_NAME)  (*reinterpret_cast<const _TYPE*>(_VAR_NAME))


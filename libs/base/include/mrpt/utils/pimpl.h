/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/config.h>
#include <mrpt/utils/mrpt_macros.h>
#include <memory>

/** \file Macros to help implementing the PIMPL idiom and make all 
 * declarations easier to read and less error-prone. 
 */
namespace mrpt {
	namespace utils {
		/** Pointer to IMPLementation auxiliary structure to make raw pointers movable, copiable and automatically deleted.
		 *\ingroup mrpt_base_grp
		 */
		template <typename T>
		struct pimpl
		{
			// This should be only parsed by MRPT sources (which since 1.5.0 require C++11). 
			// It does not matter if user code, using C++98, does not see this declaration, since PIMPL 
			// are always "private" or "protected".
#if MRPT_HAS_UNIQUE_PTR
			std::unique_ptr<T> ptr;

			// All these must be defined in a .cpp file with PIMPL_DEFINE(_TYPE), after including the 
			// real definition of T, which is only forward-declared in the headers:
			pimpl();
			~pimpl();
			pimpl(pimpl && op) noexcept;              // movable
			pimpl& operator=(pimpl&& op) noexcept;   //
			pimpl(const pimpl& op);                   // and copyable
			pimpl& operator=(const pimpl& op);        //
#endif
		};
	}
}

// ========== Header file ==============
#define PIMPL_FORWARD_DECLARATION(_TYPE)  _TYPE

#define PIMPL_DECLARE_TYPE(_TYPE, _VAR_NAME)  mrpt::utils::pimpl<_TYPE> _VAR_NAME

// ========== Implementation file ==============
#define PIMPL_IMPLEMENT(_TYPE)  \
	namespace mrpt { namespace utils { \
	template <> pimpl<_TYPE>::pimpl() : ptr() {} \
	template <> pimpl<_TYPE>::~pimpl() {} \
	/* Movable */ \
	template <> pimpl<_TYPE>::pimpl(pimpl<_TYPE> && op) noexcept : ptr(std::move(op.ptr)) {} \
	template <> pimpl<_TYPE>& pimpl<_TYPE>::operator=(pimpl<_TYPE>&& op) noexcept { ptr = std::move(op.ptr); return *this; } \
	/* Copyable: */ \
	template <> pimpl<_TYPE>::pimpl(const pimpl<_TYPE> & op) { \
		if (op.ptr.get() == ptr.get()) return; \
		ptr.reset(new  _TYPE()); \
		*ptr = *op.ptr; \
	} \
	template <> pimpl<_TYPE>& pimpl<_TYPE>::operator=(const pimpl<_TYPE>& op) { \
		if (op.ptr.get() == ptr.get()) return *this; \
		ptr.reset(new  _TYPE()); \
		*ptr = *op.ptr; \
		return *this; \
	}  \
} } 

// Put in the constructor, initializer, etc.
#define PIMPL_CONSTRUCT(_TYPE,_VAR_NAME)  _VAR_NAME.ptr.reset( new _TYPE())

#define PIMPL_GET_PTR(_TYPE, _VAR_NAME)   _VAR_NAME.ptr.get()
#define PIMPL_GET_REF(_TYPE, _VAR_NAME)  (*_VAR_NAME.ptr.get())
#define PIMPL_GET_CONSTREF(_TYPE, _VAR_NAME)  (*_VAR_NAME.ptr.get())


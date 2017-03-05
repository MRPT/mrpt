/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \page porting_mrpt2 Porting code from MRPT 1.{3,4,5} to MRPT 2.*
*
* MRPT 2.0 includes several fundamental changes, most of them related to API clean ups
* and the introduction of C++14 as the minimum supported version of the language.
*
* Existing user applications may need to be adapted to continue compiling and working
* as usual after updating to MRPT 2.*:
*
* **Mandatory changes**
*  - **Smart pointers** are now standard [`std::shared_ptr<>`](http://en.cppreference.com/w/cpp/memory/shared_ptr) instead of those based on `stlplus`. Required changes:
*     - `ptr.clear()`  --> `ptr.reset()`. Also, notice that the former `stlplus` semantics of `clear()` deleting **all** copies of the object, as hold by different smart pointers, is no longer maintained. There is no longer such a possibility, since the C++11 standard does not allow it to happen (and it makes sense in this way).
*     - `ptr.clear_unique()` --> `ptr.reset()`. (Read this note above)
*     - `ptr.make_unique()` does no longer exists, and does not make sense (read above).
*     - `ptr.pointer()` --> `ptr.get()`
*  - `mrpt::utils::CObject`
*     - `duplicate()` method has been removed, since its functionality is redundant with `clone()`.
*
*
* **Optional changes**
*   - Use the `Foo::Const::Ptr` smart pointers when possible instead of its non-const counterpart.
*
*
*/

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef STLPLUS_EXCEPTIONS
#define STLPLUS_EXCEPTIONS
////////////////////////////////////////////////////////////////////////////////

//   Author: Andy Rushton
//   Copyright: (c) Andy Rushton, 2007
//   License:   BSD License, see ../docs/license.html

//   The set of general exceptions thrown by STLplus components

////////////////////////////////////////////////////////////////////////////////
#include "containers_fixes.hpp"
#include <stdexcept>
#include <string>

namespace stlplus
{

  ////////////////////////////////////////////////////////////////////////////////
  // Thrown if a pointer or an iterator is dereferenced when it is null

  class null_dereference : public std::logic_error
  {
  public:
    null_dereference(const std::string& description) throw() :
      std::logic_error(std::string("stlplus::null_dereference: ") + description) {}
    ~null_dereference(void) throw() {}
  };

  ////////////////////////////////////////////////////////////////////////////////
  // Thrown if an iterator is dereferenced when it is pointing to the end element

  class end_dereference : public std::logic_error
  {
  public:
    end_dereference(const std::string& description) throw() :
      std::logic_error("stlplus::end_dereference: " + description) {}
    ~end_dereference(void) throw() {}
  };

  ////////////////////////////////////////////////////////////////////////////////
  // Thrown if an iterator is used with the wrong container. In other words, an
  // iterator is created as a pointer to a sub-object within a container. If
  // that iterator is then used with a different container, this exception is
  // thrown.

  class wrong_object : public std::logic_error
  {
  public:
    wrong_object(const std::string& description) throw() :
      std::logic_error("stlplus::wrong_object: " + description) {}
    ~wrong_object(void) throw() {}
  };

  ////////////////////////////////////////////////////////////////////////////////
  // Thrown if an attempt is made to copy an object that is uncopyable

  class illegal_copy : public std::logic_error
  {
  public:
    illegal_copy(const std::string& description) throw() :
      std::logic_error("stlplus::illegal_copy: " + description) {}
    ~illegal_copy(void) throw() {}
  };

  ////////////////////////////////////////////////////////////////////////////////

} // end namespace stlplus

#endif

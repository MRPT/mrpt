/*
    The STL+ C++ Library Collection

    Website <http://stlplus.sourceforge.net/> Collection <index.html>


      License Agreement

    <http://www.opensource.org/>

        * License for using the STLplus Library Collection <#license>
        * The Intent of this License <#intent>
        * How to Comply with this License <#compliance>
        * Historical Note <#history>


        License for using the STLplus Library Collection

    *© 1999-2008 Andy Rushton. All rights reserved.*

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

        * Redistributions of source code must retain the above Copyright
          notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above Copyright
          notice, this list of conditions and the following disclaimer in
          the documentation and/or other materials provided with the
          distribution.
        * Neither the name of the STLplus library nor the names of its
          contributors may be used to endorse or promote products derived
          from this software without specific prior written permission.

    This software is provided by the Copyright holders and contributors "as
    is" and any express or implied warranties, including, but not limited
    to, the implied warranties of merchantability and fitness for a
    particular purpose are disclaimed. In no event shall the Copyright owner
    or contributors be liable for any direct, indirect, incidental, special,
    exemplary, or consequential damages (including, but not limited to,
    procurement of substitute goods or services; loss of use, data, or
    profits; or business interruption) however caused and on any theory of
    liability, whether in contract, strict liability, or tort (including
    negligence or otherwise) arising in any way out of the use of this
    software, even if advised of the possibility of such damage.
*/

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

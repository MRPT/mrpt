/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <stdexcept>

namespace mrpt::serialization
{
/** Virtual base class for "schematic archives":classes abstracting json,(or xml) 
 * 
 * 
 * 
 */
class CSchemeArchiveBase
{
  public:
    /** @name Serialization API for schema based "archives"
     * @{ */
    virtual CSchemeArchiveBase &operator=(int) = 0;
    virtual CSchemeArchiveBase &operator=(float) = 0;
    virtual CSchemeArchiveBase &operator=(double) = 0;
    virtual CSchemeArchiveBase &operator=(std::nullptr_t) = 0;
    virtual CSchemeArchiveBase &operator=(std::string_view) = 0;
    
    //List accessor
    virtual CSchemeArchiveBase &operator[](size_t) = 0;
    //Dict accessor
    virtual CSchemeArchiveBase &operator[](std::string_view) = 0;
};

}
/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <cstdint>
#include <vector>
#include <string>
#include <stdexcept>
#include <string_view>
#include <memory>

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
    using Ptr = std::shared_ptr<CSchemeArchiveBase>;
    /** @name Serialization API for schema based "archives"
     * @{ */
    //Virtual Assignment methods
    virtual CSchemeArchiveBase &operator=(int) = 0;
    virtual CSchemeArchiveBase &operator=(float) = 0;
    virtual CSchemeArchiveBase &operator=(double) = 0;
    virtual CSchemeArchiveBase &operator=(std::nullptr_t) = 0;
    virtual CSchemeArchiveBase &operator=(const std::string) = 0;
    virtual CSchemeArchiveBase &operator=(bool) = 0;

    //Type conversion methods
    virtual explicit operator int() const = 0;
    virtual explicit operator float() const = 0;
    virtual explicit operator double() const = 0;
    virtual explicit operator bool() const = 0;
    virtual explicit operator std::string() const = 0;
    virtual void asSerializableObject(CSerializable& obj) = 0;
    //Converts CSerializable Objects to CSchemeArchiveBase based object 
    virtual CSchemeArchiveBase &operator=(mrpt::serialization::CSerializable&) = 0;
    
    //List accessor
    virtual CSchemeArchiveBase::Ptr operator[](size_t) = 0;
    //Dict accessor
    virtual CSchemeArchiveBase::Ptr operator[](std::string) = 0;

  protected:
    //Read Object
    void ReadObject(CSchemeArchiveBase& out, CSerializable& obj)
    {
        obj.serializeTo(out);
    }
    //Write Object
    void WriteObject(CSchemeArchiveBase& in, CSerializable& obj)
    {
        obj.serializeFrom(in);
    }
};

}
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
/** Pure virtual class carrying the implementation 
 * of CSchemeArchiveBase as per the PIMPL idiom.
 */
class CSchemeArchiveBase_impl
{
  public:
    virtual CSchemeArchiveBase &operator=(const int32_t) = 0;
    virtual CSchemeArchiveBase &operator=(const uint32_t) = 0;
    virtual CSchemeArchiveBase &operator=(const int64_t) = 0;
    virtual CSchemeArchiveBase &operator=(const uint64_t) = 0;
    virtual CSchemeArchiveBase &operator=(const float) = 0;
    virtual CSchemeArchiveBase &operator=(const double) = 0;
    virtual CSchemeArchiveBase &operator=(const std::nullptr_t) = 0;
    virtual CSchemeArchiveBase &operator=(const std::string) = 0;
    virtual CSchemeArchiveBase &operator=(bool) = 0;

    //Type conversion methods
    virtual explicit operator int32_t() const = 0;
    virtual explicit operator uint32_t() const = 0;
    virtual explicit operator int64_t() const = 0;
    virtual explicit operator uint64_t() const = 0;    
    virtual explicit operator float() const = 0;
    virtual explicit operator double() const = 0;
    virtual explicit operator bool() const = 0;
    virtual explicit operator std::string() const = 0;
    virtual void asSerializableObject(CSerializable& obj) = 0;
    //Converts CSerializable Objects to CSchemeArchiveBase based object 
    virtual CSchemeArchiveBase &operator=(mrpt::serialization::CSerializable&) = 0;
    
    //List accessor
    virtual CSchemeArchiveBase operator[](size_t) = 0;
    //Dict accessor
    virtual CSchemeArchiveBase operator[](std::string) = 0;
  public:// should make it private by virtue of friend class
    void setParent(CSchemeArchiveBase* parent)
    {
        m_parent = parent;
    }
  protected:
    CSchemeArchiveBase* m_parent;
    void ReadObject(CSchemeArchiveBase& out, CSerializable& obj);
    void WriteObject(CSchemeArchiveBase& out, CSerializable& obj);
};
/** Virtual base class for "schematic archives":classes abstracting json,(or xml) 
 * 
 * 
 * 
 */
class CSchemeArchiveBase
{
  public:
    friend class CSchemeArchiveBase_impl;
    using Ptr = std::shared_ptr<CSchemeArchiveBase>;
    /** @name Serialization API for schema based "archives"
     * @{ */
    //Constructor 
    CSchemeArchiveBase(std::unique_ptr<CSchemeArchiveBase_impl> ptr) : pimpl(std::move(ptr)) 
    {
        pimpl->setParent(this);
    }
    CSchemeArchiveBase &operator=(const int32_t val)
    {
        return (*pimpl).operator=(val);
    }
    CSchemeArchiveBase &operator=(const uint32_t val)
    {
        return (*pimpl).operator=(val);
    }
    CSchemeArchiveBase &operator=(const int64_t val)
    {
        return (*pimpl).operator=(val);
    }
    CSchemeArchiveBase &operator=(const uint64_t val)
    {
        return (*pimpl).operator=(val);
    }
    CSchemeArchiveBase &operator=(const float val)
    {
        return (*pimpl).operator=(val);
    }
    CSchemeArchiveBase &operator=(const double val)
    {
        return (*pimpl).operator=(val);
    }
    CSchemeArchiveBase &operator=(const std::nullptr_t val)
    {
        return (*pimpl).operator=(val);
    }
    CSchemeArchiveBase &operator=(const std::string val)
    {
        return (*pimpl).operator=(val);
    }
    CSchemeArchiveBase &operator=(bool val)
    {
        return (*pimpl).operator=(val);
    }

    //Type conversion methods
    explicit operator int32_t() const
    {
        return static_cast<int32_t>(*pimpl);
    }
    explicit operator uint32_t() const
    {
        return static_cast<uint32_t>(*pimpl);
    }
    explicit operator int64_t() const
    {
        return static_cast<int64_t>(*pimpl);
    }
    explicit operator uint64_t() const
    {
        return static_cast<uint64_t>(*pimpl);
    }    
    explicit operator float() const
    {
        return static_cast<float>(*pimpl);
    }
    explicit operator double() const
    {
        return static_cast<double>(*pimpl);
    }
    explicit operator bool() const
    {
        return static_cast<bool>(*pimpl);
    }
    explicit operator std::string() const
    {
        return static_cast<std::string>(*pimpl);
    }
    void asSerializableObject(CSerializable& obj)
    {
        pimpl->asSerializableObject(obj);
    }
    //Converts CSerializable Objects to CSchemeArchiveBase based object 
    CSchemeArchiveBase &operator=(mrpt::serialization::CSerializable& obj)
    {
        return (*pimpl).operator=(obj);
    }
    //List accessor
    CSchemeArchiveBase operator[](size_t val)
    {
        return (*pimpl).operator[](val);
    }
    //Dict accessor
    CSchemeArchiveBase operator[](std::string val)
    {
        return (*pimpl).operator[](val);
    }
    // class CSchemeArchiveBase_impl;
  protected:
    //Read Object
    static void ReadObject(CSchemeArchiveBase& out, CSerializable& obj)
    {
        obj.serializeTo(out);
    }
    //Write Object
    static void WriteObject(CSchemeArchiveBase& in, CSerializable& obj)
    {
        obj.serializeFrom(in);
    }
  private:
    std::unique_ptr<CSchemeArchiveBase_impl> pimpl;
};
}
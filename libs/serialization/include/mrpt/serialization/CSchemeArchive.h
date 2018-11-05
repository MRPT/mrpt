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
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <string>
#include <memory>
#include <optional>
#include <sstream>

namespace mrpt::serialization
{
/** Base template class for schema-capable "archives", e.g. JSON, YAML,
 * from which to (de)serialize objects.
 *
 * See \ref mrpt_serialization_grp for examples of use.
 * \ingroup mrpt_serialization_grp
 * \note Original version by https://github.com/rachit173 for GSoC 2018.
 */
template <typename SCHEME_CAPABLE>
class CSchemeArchive : public mrpt::serialization::CSchemeArchiveBase_impl
{
	std::optional<SCHEME_CAPABLE> m_own_val;

   public:
	/** Ctor that creates an own SCHEME_CAPABLE object. */
	CSchemeArchive() : m_own_val{SCHEME_CAPABLE()}, m_val{m_own_val.value()} {}
	/** Ctor that uses user-providen SCHEME_CAPABLE object. */
	CSchemeArchive(SCHEME_CAPABLE& val) : m_val(val) {}
	// Virtual assignment operators
	mrpt::serialization::CSchemeArchiveBase& operator=(
		const int32_t val) override
	{
		m_val = val;
		return *m_parent;
	}
	mrpt::serialization::CSchemeArchiveBase& operator=(
		const uint32_t val) override
	{
		m_val = val;
		return *m_parent;
	}
	mrpt::serialization::CSchemeArchiveBase& operator=(
		const int64_t val) override
	{
		m_val = typename SCHEME_CAPABLE::Int64(val);
		return *m_parent;
	}
	mrpt::serialization::CSchemeArchiveBase& operator=(
		const uint64_t val) override
	{
		m_val = typename SCHEME_CAPABLE::UInt64(val);
		return *m_parent;
	}
	mrpt::serialization::CSchemeArchiveBase& operator=(const float val) override
	{
		m_val = val;
		return *m_parent;
	}
	mrpt::serialization::CSchemeArchiveBase& operator=(
		const double val) override
	{
		m_val = val;
		return *m_parent;
	}
	mrpt::serialization::CSchemeArchiveBase& operator=(
		const std::nullptr_t val) override
	{
		m_val = val;
		return *m_parent;
	}
	mrpt::serialization::CSchemeArchiveBase& operator=(
		const std::string val) override
	{
		m_val = val;
		return *m_parent;
	}
	mrpt::serialization::CSchemeArchiveBase& operator=(const bool val) override
	{
		m_val = val;
		return *m_parent;
	}

	explicit operator int32_t() const override { return m_val.asInt(); }
	explicit operator uint32_t() const override { return m_val.asUInt(); }
	explicit operator int64_t() const override { return m_val.asInt64(); }
	explicit operator uint64_t() const override { return m_val.asUInt64(); }
	explicit operator float() const override { return m_val.asFloat(); }
	explicit operator double() const override { return m_val.asDouble(); }
	explicit operator bool() const override { return m_val.asBool(); }
	explicit operator std::string() const override { return m_val.asString(); }
	mrpt::serialization::CSchemeArchiveBase& operator=(
		const mrpt::serialization::CSerializable& obj) override
	{
		ReadObject(*m_parent, obj);
		return *m_parent;
	}
	void readTo(mrpt::serialization::CSerializable& obj) override
	{
		WriteObject(*m_parent, obj);
		return;
	}

	mrpt::serialization::CSchemeArchiveBase operator[](size_t idx) override
	{
		return mrpt::serialization::CSchemeArchiveBase(
			std::make_unique<CSchemeArchive<SCHEME_CAPABLE>>(m_val[(int)idx]));
	}
	mrpt::serialization::CSchemeArchiveBase operator[](std::string str) override
	{
		return mrpt::serialization::CSchemeArchiveBase(
			std::make_unique<CSchemeArchive<SCHEME_CAPABLE>>(
				m_val[std::string(str)]));
	}

	std::ostream& writeToStream(std::ostream& out) const override
	{
		out << m_val;
		return out;
	}
	std::istream& readFromStream(std::istream& in) override
	{
		in >> m_val;
		return in;
	}

   private:
	SCHEME_CAPABLE& m_val;
};

/** Returns an archive for reading/writing in JSON format.
 * This feature requires compiling MRPT with jsoncpp support.
 * See \ref mrpt_serialization_grp for examples of use.
 * \ingroup mrpt_serialization_grp */
CSchemeArchiveBase archiveJSON();

}  // namespace mrpt::serialization

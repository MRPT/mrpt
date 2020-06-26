/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "containers-precomp.h"  // Precompiled headers
//
#include <mrpt/config.h>
#include <mrpt/containers/Parameters.h>
#include <mrpt/core/constexpr_for.h>
#include <mrpt/core/exceptions.h>

#include <iostream>
#if MRPT_HAS_YAMLCPP
#include <yaml-cpp/yaml.h>
#endif

using namespace mrpt::containers;

Parameters::Parameters(const Parameters& v) { *this = v; }

const char* mrpt::containers::internal::typeIdxToStr(const std::size_t idx)
{
	switch (idx)
	{
		case 0:
			return "uninitialized";
		case 1:
			return "double";
		case 2:
			return "uint64_t";
		case 3:
			return "std::string";
		case 4:
			return "Parameters";
		default:
			return "undefined";
	};
}

bool Parameters::isSequence() const
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call isSequence() on a value (!)");
		return p->isSequence();
	}
	return std::holds_alternative<sequence_t>(data_);
}
Parameters::sequence_t& Parameters::asSequence()
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call asSequence() on a value (!)");
		return p->asSequence();
	}
	return std::get<sequence_t>(data_);
}
const Parameters::sequence_t& Parameters::asSequence() const
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call asSequence() on a value (!)");
		return p->asSequence();
	}
	return std::get<sequence_t>(data_);
}

bool Parameters::isMap() const
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call isMap() on a value (!)");
		return p->isSequence();
	}
	return std::holds_alternative<map_t>(data_);
}
Parameters::map_t& Parameters::asMap()
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call asMap() on a value (!)");
		return p->asMap();
	}
	return std::get<map_t>(data_);
}
const Parameters::map_t& Parameters::asMap() const
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call asMap() on a value (!)");
		return p->asMap();
	}
	return std::get<map_t>(data_);
}

bool Parameters::has(const std::string& key) const
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call has() on a value (!)");
		return p->has(key);
	}
	if (!std::holds_alternative<map_t>(data_))
		THROW_EXCEPTION("has() not applicable to non-map nodes.");

	const map_t& m = std::get<map_t>(data_);
	return m.end() != m.find(key);
}

std::string Parameters::typeOfChild(const std::string& name) const
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call typeOf() on a value (!)");
		return p->typeOfChild(name);
	}
	if (!std::holds_alternative<map_t>(data_))
		THROW_EXCEPTION("typeOfChild() not applicable to non-map nodes.");

	const map_t& m = std::get<map_t>(data_);

	auto it = m.find(name);
	if (m.end() == it) return {};
	return internal::typeIdxToStr(it->second.index());
}

const Parameters* Parameters::internalValueAsSelf() const
{
	const Parameters* p = nullptr;
	if (valuenc_ && std::holds_alternative<Parameters>(*valuenc_))
		p = &std::get<Parameters>(*valuenc_);
	if (value_ && std::holds_alternative<Parameters>(*value_))
		p = &std::get<Parameters>(*value_);
	return p;
}
Parameters* Parameters::internalValueAsSelf()
{
	Parameters* p = nullptr;
	if (valuenc_ && std::holds_alternative<Parameters>(*valuenc_))
		p = &std::get<Parameters>(*valuenc_);
	return p;
}
const Parameters* Parameters::internalMeOrValue() const
{
	const Parameters* p = internalValueAsSelf();
	return p != nullptr ? p : this;
}
Parameters* Parameters::internalMeOrValue()
{
	Parameters* p = internalValueAsSelf();
	return p != nullptr ? p : this;
}

bool Parameters::empty() const
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call empty() on a value (!)");
		return p->empty();
	}
	if (std::holds_alternative<sequence_t>(data_))
		return std::get<sequence_t>(data_).empty();
	if (std::holds_alternative<map_t>(data_))
		return std::get<map_t>(data_).empty();
	return true;
}
void Parameters::clear()
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call clear() on a value (!)");
		p->clear();
	}
	if (std::holds_alternative<sequence_t>(data_))
	{
		std::get<sequence_t>(data_).clear();
		return;
	}
	if (std::holds_alternative<map_t>(data_))
	{
		std::get<map_t>(data_).clear();
		return;
	}
}

void Parameters::operator=(const double v)
{
	if (isConstProxy_)
		throw std::logic_error("Trying to write into read-only proxy");
	if (!isProxy_)
		throw std::logic_error(
			"Trying to write into a Parameter block. Use "
			"`p[\"name\"]=value;` instead");
	if (!valuenc_) throw std::logic_error("valuenc_ is nullptr");
	valuenc_->emplace<double>(v);
}
void Parameters::operator=(const std::string& v)
{
	if (isConstProxy_)
		throw std::logic_error("Trying to write into read-only proxy");
	if (!isProxy_)
		throw std::logic_error(
			"Trying to write into a Parameter block. Use "
			"`p[\"name\"]=value;` instead");
	if (!valuenc_) throw std::logic_error("valuenc_ is nullptr");
	*valuenc_ = v;
}
Parameters& Parameters::operator=(const Parameters& v)
{
	if (isConstProxy_)
		throw std::logic_error("Trying to write into read-only proxy");

	if (isProxy_)
	{
		if (!valuenc_) throw std::logic_error("valuenc_ is nullptr");
		valuenc_->emplace<Parameters>(v);
		return std::get<Parameters>(*valuenc_);
	}
	else
	{
		data_ = v.internalMeOrValue()->data_;
		isProxy_ = false;
		isConstProxy_ = false;
		return *this;
	}
}

Parameters Parameters::operator[](const char* s)
{
	ASSERT_(s != nullptr);
	Parameters* p = internalMeOrValue();
	// Init as map on first use:
	if (p->empty()) p->data_.emplace<map_t>();

	if (!p->isMap())
		THROW_EXCEPTION("write operator[] not applicable to non-map nodes.");

	return Parameters(
		internal::tag_as_proxy_t(), std::get<map_t>(p->data_)[s], s);
}

const Parameters Parameters::operator[](const char* s) const
{
	ASSERT_(s != nullptr);
	const Parameters* p = internalMeOrValue();
	if (p->empty())
		THROW_EXCEPTION("read operator[] not applicable to empty nodes.");
	if (!p->isMap())
		THROW_EXCEPTION("read operator[] only available for map nodes.");

	const map_t& m = std::get<map_t>(p->data_);
	auto it = m.find(s);
	if (m.end() == it)
		THROW_EXCEPTION_FMT("Access non-existing map key `%s`", s);

	return Parameters(internal::tag_as_const_proxy_t(), it->second, s);
}

Parameters Parameters::operator()(int index)
{
	Parameters* p = internalMeOrValue();
	if (p->empty())
		THROW_EXCEPTION(
			"write operator() not applicable to empty nodes or sequences.");

	if (!p->isSequence())
		THROW_EXCEPTION("write operator() only available for sequence nodes.");

	sequence_t& seq = std::get<sequence_t>(p->data_);
	if (index < 0 || index >= static_cast<int>(seq.size()))
		throw std::out_of_range("Parameters::operator() out of range");

	return Parameters(internal::tag_as_proxy_t(), seq.at(index), "");
}
const Parameters Parameters::operator()(int index) const
{
	const Parameters* p = internalMeOrValue();
	if (p->empty())
		THROW_EXCEPTION("read operator[] not applicable to empty nodes.");
	if (!p->isSequence())
		THROW_EXCEPTION("read operator() only available for sequence nodes.");

	const sequence_t& seq = std::get<sequence_t>(p->data_);
	if (index < 0 || index >= static_cast<int>(seq.size()))
		throw std::out_of_range("Parameters::operator() out of range");

	return Parameters(internal::tag_as_const_proxy_t(), seq.at(index), "");
}

void Parameters::printAsYAML() const { printAsYAML(std::cout); }

void Parameters::printAsYAML(std::ostream& o) const
{
	const Parameters* p = internalMeOrValue();
	internalPrintAsYAML(*p, o, 0, true);
}

void Parameters::internalPrintAsYAML(
	const Parameters& p, std::ostream& o, int indent, bool first)
{
	if (std::holds_alternative<std::monostate>(p.data_))
		return internalPrintAsYAML(std::monostate(), o, indent, first);
	if (std::holds_alternative<map_t>(p.data_))
		return internalPrintAsYAML(std::get<map_t>(p.data_), o, indent, first);
	if (std::holds_alternative<sequence_t>(p.data_))
		return internalPrintAsYAML(
			std::get<sequence_t>(p.data_), o, indent, first);
}

void Parameters::internalPrintAsYAML(
	const std::monostate&, std::ostream& o, [[maybe_unused]] int indent,
	[[maybe_unused]] bool first)
{
	o << "~\n";
}
void Parameters::internalPrintAsYAML(
	const double& v, std::ostream& o, [[maybe_unused]] int indent,
	[[maybe_unused]] bool first)
{
	o << std::to_string(v) << "\n";
}
void Parameters::internalPrintAsYAML(
	const std::string& v, std::ostream& o, [[maybe_unused]] int indent,
	[[maybe_unused]] bool first)
{
	o << v << "\n";
}
void Parameters::internalPrintAsYAML(
	const uint64_t& v, std::ostream& o, [[maybe_unused]] int indent,
	[[maybe_unused]] bool first)
{
	const std::string sInd(indent, ' ');
	o << sInd << std::to_string(v) << "\n";
}
void Parameters::internalPrintAsYAML(
	const Parameters::sequence_t& v, std::ostream& o, int indent, bool first)
{
	if (!first)
	{
		o << "\n";
		indent += 2;
	}
	const std::string sInd(indent, ' ');
	for (const auto& e : v)
	{
		o << sInd << "- ";
		const auto newIndent = indent;
		internalPrintAsYAML(e, o, newIndent, first);
	}
}
void Parameters::internalPrintAsYAML(
	const Parameters::map_t& m, std::ostream& o, int indent, bool first)
{
	if (!first)
	{
		o << "\n";
		indent += 2;
	}
	const std::string sInd(indent, ' ');
	for (const auto& kv : m)
	{
		const value_t& v = kv.second;
		o << sInd << kv.first << ": ";
		internalPrintAsYAML(v, o, indent, false);
	}
}

void Parameters::internalPrintAsYAML(
	const Parameters::value_t& v, std::ostream& o, int indent,
	[[maybe_unused]] bool first)
{
	mrpt::for_<std::variant_size_v<value_t>>(  //
		[&](auto i) {  //
			if (v.index() != i.value) return;
			return internalPrintAsYAML(std::get<i.value>(v), o, indent, false);
		});
}

Parameters Parameters::FromYAMLText(const std::string& yamlTextBlock)
{
#if MRPT_HAS_YAMLCPP
	YAML::Node n = YAML::Load(yamlTextBlock);
	return Parameters::FromYAML(n);
#else
	THROW_EXCEPTION("MRPT was built without yaml-cpp");
#endif
}

Parameters Parameters::FromYAML(const YAML::Node& n)
{
#if MRPT_HAS_YAMLCPP
	const auto invalidDbl = std::numeric_limits<double>::max();

	if (n.IsSequence())
	{
		auto ret = Parameters::Sequence();

		for (const auto& e : n)
		{
			if (e.IsScalar())
			{
				if (double v = e.as<double>(invalidDbl); v != invalidDbl)
					ret.push_back(v);
				else
					ret.push_back(e.as<std::string>());
			}
			else
			{
				// Recursive:
				ret.push_back(Parameters::FromYAML(e));
			}
		}
		return ret;
	}
	else if (n.IsMap())
	{
		auto ret = Parameters::Map();

		for (const auto& kv : n)
		{
			const auto& key = kv.first.as<std::string>();
			const auto& val = kv.second;

			if (val.IsScalar())
			{
				if (double v = val.as<double>(invalidDbl); v != invalidDbl)
					ret[key] = v;
				else
					ret[key] = val.as<std::string>();
			}
			else
			{
				// Recursive:
				ret[key] = Parameters::FromYAML(val);
			}
		}
		return ret;
	}
	else
	{
		THROW_EXCEPTION("FromYAML only supports root YAML as sequence or map");
	}

#else
	THROW_EXCEPTION("MRPT was built without yaml-cpp");
#endif
}

void Parameters::loadFromYAML(const YAML::Node& n)
{
	*this = Parameters::FromYAML(n);
}

// ============================ internal  =====================================
template <typename T>
const T& mrpt::containers::internal::implAsGetter(
	const Parameters& p, const char* expectedType)
{
	ASSERT_(p.isProxy_);
	try
	{
		if (p.isConstProxy_)
			return std::get<T>(*p.value_);
		else
			return std::get<T>(*p.valuenc_);
	}
	catch (const std::bad_variant_access&)
	{
		THROW_EXCEPTION_FMT(
			"Trying to read parameter `%s` of type `%s` as if it was `%s`",
			p.name_, typeIdxToStr(p.value_->index()), expectedType);
	}
}

template <>
const double& mrpt::containers::internal::asGetter(const Parameters& p)
{
	return implAsGetter<double>(p, "double");
}
template <>
const uint64_t& mrpt::containers::internal::asGetter(const Parameters& p)
{
	return implAsGetter<uint64_t>(p, "uint64_t");
}
template <>
const std::string& mrpt::containers::internal::asGetter(const Parameters& p)
{
	return implAsGetter<std::string>(p, "std::string");
}
template <>
const Parameters& mrpt::containers::internal::asGetter(const Parameters& p)
{
	return implAsGetter<Parameters>(p, "Parameters");
}

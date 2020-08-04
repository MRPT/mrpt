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

#include <iostream>
#if MRPT_HAS_YAMLCPP
#include <yaml-cpp/yaml.h>
#endif

using namespace mrpt::containers;

Parameters::Parameters(const Parameters& v) { *this = v; }

bool Parameters::isSequence() const
{
	auto p = internalMeOrValue();
	return std::holds_alternative<sequence_t>(p->data_);
}

Parameters::sequence_t& Parameters::asSequence()
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call asSequence() on a scalar");
		return p->asSequence();
	}
	return std::get<sequence_t>(data_);
}
const Parameters::sequence_t& Parameters::asSequence() const
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call asSequence() on a scalar");
		return p->asSequence();
	}
	return std::get<sequence_t>(data_);
}

bool Parameters::isScalar() const
{
	auto p = internalMeOrValue();
	return (p->isProxy_ || p->isConstProxy_);
}
bool Parameters::isEmptyNode() const
{
	auto p = internalMeOrValue();

	if (p->isConstProxy_)
	{
		ASSERT_(p->value_);
		return !p->value_->has_value();
	}
	if (p->isProxy_)
	{
		ASSERT_(p->valuenc_);
		return !p->valuenc_->has_value();
	}
	return false;
}

bool Parameters::isMap() const
{
	auto p = internalMeOrValue();
	return std::holds_alternative<map_t>(p->data_);
}
Parameters::map_t& Parameters::asMap()
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call asMap() on a scalar");
		return p->asMap();
	}
	return std::get<map_t>(data_);
}
const Parameters::map_t& Parameters::asMap() const
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call asMap() on a scalar");
		return p->asMap();
	}
	return std::get<map_t>(data_);
}

bool Parameters::has(const std::string& key) const
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call has() on a scalar");
		return p->has(key);
	}
	if (!std::holds_alternative<map_t>(data_))
		THROW_EXCEPTION("has() not applicable to non-map nodes.");

	const map_t& m = std::get<map_t>(data_);
	return m.end() != m.find(key);
}

const std::type_info& Parameters::typeOfChild(const std::string& name) const
{
	if (isProxy_)
	{
		auto p = internalValueAsSelf();
		ASSERTMSG_(p, "Cannot call typeOf() on a scalar.");
		return p->typeOfChild(name);
	}
	if (!std::holds_alternative<map_t>(data_))
		THROW_EXCEPTION("typeOfChild() not applicable to non-map nodes.");

	const map_t& m = std::get<map_t>(data_);

	auto it = m.find(name);
	if (m.end() == it) return typeid(void);
	return it->second.type();
}

const std::type_info& Parameters::type() const
{
	auto p = internalMeOrValue();

	if (p->isConstProxy_)
	{
		ASSERT_(p->value_);
		return p->value_->type();
	}
	if (p->isProxy_)
	{
		ASSERT_(p->valuenc_);
		return p->valuenc_->type();
	}
	THROW_EXCEPTION("type() only applicable to scalar nodes");
}

const Parameters* Parameters::internalValueAsSelf() const
{
	const Parameters* p = nullptr;
	if (valuenc_ && valuenc_->type() == typeid(Parameters))
		p = std::any_cast<Parameters>(valuenc_);
	if (value_ && value_->type() == typeid(Parameters))
		p = std::any_cast<Parameters>(value_);
	return p;
}
Parameters* Parameters::internalValueAsSelf()
{
	Parameters* p = nullptr;
	if (valuenc_ && valuenc_->type() == typeid(Parameters))
		p = std::any_cast<Parameters>(valuenc_);
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
		ASSERTMSG_(p, "Cannot call empty() on a scalar");
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
		ASSERTMSG_(p, "Cannot call clear() on a scalar");
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

void Parameters::operator=(bool v) { implOpAssign(v); }

void Parameters::operator=(double v) { implOpAssign(v); }
void Parameters::operator=(float v) { implOpAssign(v); }

void Parameters::operator=(int8_t v) { implOpAssign(v); }
void Parameters::operator=(uint8_t v) { implOpAssign(v); }

void Parameters::operator=(int16_t v) { implOpAssign(v); }
void Parameters::operator=(uint16_t v) { implOpAssign(v); }

void Parameters::operator=(int32_t v) { implOpAssign(v); }
void Parameters::operator=(uint32_t v) { implOpAssign(v); }

void Parameters::operator=(int64_t v) { implOpAssign(v); }
void Parameters::operator=(uint64_t v) { implOpAssign(v); }

void Parameters::operator=(const std::string& v) { implOpAssign(v); }
Parameters& Parameters::operator=(const Parameters& v)
{
	if (isConstProxy_) THROW_EXCEPTION("Trying to write into read-only proxy");

	if (isProxy_)
	{
		if (!valuenc_) THROW_EXCEPTION("valuenc_ is nullptr");
		valuenc_->emplace<Parameters>(v);
		return *std::any_cast<Parameters>(valuenc_);
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
	if (std::holds_alternative<std::monostate>(p->data_))
		p->data_.emplace<map_t>();

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
		THROW_TYPED_EXCEPTION(
			"Parameters::operator() out of range", std::out_of_range);

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
		THROW_TYPED_EXCEPTION(
			"Parameters::operator() out of range", std::out_of_range);

	return Parameters(internal::tag_as_const_proxy_t(), seq.at(index), "");
}

void Parameters::printAsYAML() const { printAsYAML(std::cout); }

void Parameters::printAsYAML(std::ostream& o) const
{
	const Parameters* p = internalMeOrValue();
	internalPrintAsYAML(*p, o, 0, true);
}

bool Parameters::internalPrintAsYAML(
	const Parameters& p, std::ostream& o, int indent, bool first)
{
	if (p.isProxy_ && p.valuenc_)
		return internalPrintAsYAML(*p.valuenc_, o, indent, first);
	if (p.isConstProxy_ && p.value_)
		return internalPrintAsYAML(*p.value_, o, indent, first);

	if (std::holds_alternative<map_t>(p.data_))
		return internalPrintAsYAML(std::get<map_t>(p.data_), o, indent, first);
	if (std::holds_alternative<sequence_t>(p.data_))
		return internalPrintAsYAML(
			std::get<sequence_t>(p.data_), o, indent, first);
	if (std::holds_alternative<std::monostate>(p.data_))
		return internalPrintAsYAML(std::monostate(), o, indent, first);

	THROW_EXCEPTION("Trying to print a node of unknown type (!)");
}

bool Parameters::internalPrintAsYAML(
	const std::monostate&, std::ostream& o, [[maybe_unused]] int indent,
	[[maybe_unused]] bool first)
{
	o << "~";
	return false;
}
bool Parameters::internalPrintAsYAML(
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
		if (!internalPrintAsYAML(e, o, indent, false)) o << "\n";
	}
	return true;
}
bool Parameters::internalPrintAsYAML(
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
		const scalar_t& v = kv.second;
		o << sInd << kv.first << ": ";
		if (!internalPrintAsYAML(v, o, indent, false)) o << "\n";
	}
	return true;
}

bool Parameters::internalPrintAsYAML(
	const Parameters::scalar_t& v, std::ostream& o, int indent, bool first)
{
	if (!v.has_value())
	{
		o << "~";
		return false;
	}

	if (v.type() == typeid(Parameters))
		return internalPrintAsYAML(
			std::any_cast<Parameters>(v), o, indent, first);
	else if (v.type() == typeid(bool))
		o << (std::any_cast<bool>(v) ? "true" : "false");
	else if (v.type() == typeid(uint64_t))
		o << std::any_cast<uint64_t>(v);
	else if (v.type() == typeid(int64_t))
		o << std::any_cast<int64_t>(v);
	else if (v.type() == typeid(uint32_t))
		o << std::any_cast<uint32_t>(v);
	else if (v.type() == typeid(int32_t))
		o << std::any_cast<int32_t>(v);
	else if (v.type() == typeid(int))
		o << std::any_cast<int>(v);
	else if (v.type() == typeid(unsigned int))
		o << std::any_cast<unsigned int>(v);
	else if (v.type() == typeid(const char*))
		o << std::any_cast<const char*>(v);
	else if (v.type() == typeid(std::string))
		o << std::any_cast<std::string>(v);
	else if (v.type() == typeid(float))
		o << std::any_cast<float>(v);
	else if (v.type() == typeid(double))
		o << std::any_cast<double>(v);
	else if (v.type() == typeid(uint16_t))
		o << std::any_cast<uint16_t>(v);
	else if (v.type() == typeid(int16_t))
		o << std::any_cast<int16_t>(v);
	else if (v.type() == typeid(uint8_t))
		o << std::any_cast<uint8_t>(v);
	else if (v.type() == typeid(int8_t))
		o << std::any_cast<int8_t>(v);
	else if (v.type() == typeid(char))
		o << std::any_cast<char>(v);
	else
		o << "(unknown type)";
	return false;
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
			if (e.IsNull())
			{
				sequence_t& seq = std::get<sequence_t>(ret.data_);
				seq.emplace_back();
			}
			else if (e.IsScalar())
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

			if (val.IsNull())
			{
				map_t& m = std::get<map_t>(ret.data_);
				m[key];
			}
			else if (val.IsScalar())
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

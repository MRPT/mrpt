/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "containers-precomp.h"	 // Precompiled headers
//
#include <mrpt/config.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>

#include <iostream>
#if MRPT_HAS_YAMLCPP
#include <yaml-cpp/yaml.h>
#endif

using namespace mrpt::containers;

// ============ class: yaml::node_t =======
bool yaml::node_t::isNullNode() const
{
	return std::holds_alternative<std::monostate>(d);
}
bool yaml::node_t::isScalar() const
{
	return std::holds_alternative<scalar_t>(d);
}
bool yaml::node_t::isSequence() const
{
	return std::holds_alternative<sequence_t>(d);
}
bool yaml::node_t::isMap() const { return std::holds_alternative<map_t>(d); }

std::string yaml::node_t::typeName() const
{
	MRPT_START
	if (isNullNode()) return "null";
	if (isSequence()) return "sequence";
	if (isMap()) return "map";
	ASSERT_(isScalar());
	const auto& s = std::get<scalar_t>(d);
	ASSERT_(s.has_value());
	return mrpt::format("scalar(%s)", mrpt::demangle(s.type().name()).c_str());
	MRPT_END
}

yaml::sequence_t& yaml::node_t::asSequence()
{
	MRPT_START
	return const_cast<sequence_t&>(
		const_cast<const node_t*>(this)->asSequence());
	MRPT_END
}
const yaml::sequence_t& yaml::node_t::asSequence() const
{
	MRPT_START
	ASSERTMSG_(
		std::holds_alternative<sequence_t>(d),
		mrpt::format(
			"Trying to access node of type `%s` as a sequence",
			typeName().c_str()));

	return std::get<sequence_t>(d);
	MRPT_END
}

yaml::map_t& yaml::node_t::asMap()
{
	MRPT_START
	return const_cast<map_t&>(const_cast<const node_t*>(this)->asMap());
	MRPT_END
}
const yaml::map_t& yaml::node_t::asMap() const
{
	MRPT_START
	ASSERTMSG_(
		std::holds_alternative<map_t>(d),
		mrpt::format(
			"Trying to access node of type `%s` as a map", typeName().c_str()));
	return std::get<map_t>(d);
	MRPT_END
}

yaml::scalar_t& yaml::node_t::asScalar()
{
	MRPT_START
	return const_cast<scalar_t&>(const_cast<const node_t*>(this)->asScalar());
	MRPT_END
}
const yaml::scalar_t& yaml::node_t::asScalar() const
{
	MRPT_START
	ASSERTMSG_(
		std::holds_alternative<scalar_t>(d),
		mrpt::format(
			"Trying to access node of type `%s` as a scalar",
			typeName().c_str()));

	return std::get<scalar_t>(d);
	MRPT_END
}

// ============ class: yaml =======
yaml::yaml(const yaml& v) { *this = v; }

bool yaml::isScalar() const { return dereferenceProxy()->isScalar(); }
bool yaml::isNullNode() const { return dereferenceProxy()->isNullNode(); }
bool yaml::isMap() const { return dereferenceProxy()->isMap(); }
bool yaml::isSequence() const { return dereferenceProxy()->isSequence(); }

yaml::sequence_t& yaml::asSequence()
{
	MRPT_START
	return const_cast<sequence_t&>(const_cast<const yaml*>(this)->asSequence());
	MRPT_END
}
const yaml::sequence_t& yaml::asSequence() const
{
	MRPT_START
	return dereferenceProxy()->asSequence();
	MRPT_END
}

yaml::map_t& yaml::asMap()
{
	MRPT_START
	return const_cast<map_t&>(const_cast<const yaml*>(this)->asMap());
	MRPT_END
}
const yaml::map_t& yaml::asMap() const
{
	MRPT_START
	return dereferenceProxy()->asMap();
	MRPT_END
}

yaml::scalar_t& yaml::asScalar()
{
	MRPT_START
	return const_cast<scalar_t&>(const_cast<const yaml*>(this)->asScalar());
	MRPT_END
}

const yaml::scalar_t& yaml::asScalar() const
{
	MRPT_START
	return dereferenceProxy()->asScalar();
	MRPT_END
}

bool yaml::has(const std::string& key) const
{
	const map_t& m = this->asMap();
	return m.end() != m.find(key);
}

const std::type_info& yaml::scalarType() const
{
	MRPT_START
	if (this->isNullNode()) return typeid(void);

	const scalar_t& s = this->asScalar();
	return s.type();
	MRPT_END
}

const yaml::node_t* yaml::dereferenceProxy() const
{
	MRPT_START
	if (isProxy_)
	{
		ASSERT_(proxiedNode_ != nullptr);
		return proxiedNode_;
	}
	return &root_;
	MRPT_END
}
yaml::node_t* yaml::dereferenceProxy()
{
	MRPT_START
	if (isProxy_)
	{
		ASSERT_(proxiedNode_ != nullptr);
		if (isConstProxy_)
			THROW_EXCEPTION_FMT(
				"Trying to write-access a const-proxy (key name:'%s')",
				proxiedMapEntryName_ ? proxiedMapEntryName_ : "(nullptr)");
		return const_cast<node_t*>(proxiedNode_);
	}
	return &root_;
	MRPT_END
}

bool yaml::empty() const
{
	auto n = dereferenceProxy();
	if (std::holds_alternative<map_t>(n->d))
		return std::get<map_t>(n->d).empty();

	if (std::holds_alternative<sequence_t>(n->d))
		return std::get<sequence_t>(n->d).empty();

	if (std::holds_alternative<std::monostate>(n->d)) return true;

	if (std::holds_alternative<scalar_t>(n->d))
		THROW_EXCEPTION(
			"empty() called on a scalar node: only available for maps or "
			"sequences.");

	THROW_EXCEPTION("Should never reach here");
}

void yaml::clear()
{
	auto n = dereferenceProxy();
	if (std::holds_alternative<map_t>(n->d))
		return std::get<map_t>(n->d).clear();

	if (std::holds_alternative<sequence_t>(n->d))
		return std::get<sequence_t>(n->d).clear();

	THROW_EXCEPTION_FMT(
		"clear() called on a node of type '%s': only available for maps or "
		"sequences.",
		n->typeName().c_str());
}

void yaml::operator=(bool v) { implOpAssign(v); }

void yaml::operator=(double v) { implOpAssign(v); }
void yaml::operator=(float v) { implOpAssign(v); }

void yaml::operator=(int8_t v) { implOpAssign(v); }
void yaml::operator=(uint8_t v) { implOpAssign(v); }

void yaml::operator=(int16_t v) { implOpAssign(v); }
void yaml::operator=(uint16_t v) { implOpAssign(v); }

void yaml::operator=(int32_t v) { implOpAssign(v); }
void yaml::operator=(uint32_t v) { implOpAssign(v); }

void yaml::operator=(int64_t v) { implOpAssign(v); }
void yaml::operator=(uint64_t v) { implOpAssign(v); }

void yaml::operator=(const std::string& v) { implOpAssign(v); }
yaml& yaml::operator=(const yaml& v)
{
	MRPT_START
	ASSERTMSG_(!isConstProxy_, "Trying to write into read-only proxy");
	node_t* n = dereferenceProxy();
	*n = *v.dereferenceProxy();
	return *this;
	MRPT_END
}

yaml yaml::operator[](const char* s)
{
	ASSERT_(s != nullptr);
	node_t* n = dereferenceProxy();
	// Init as map on first use:
	if (n->isNullNode()) n->d.emplace<map_t>();

	if (!n->isMap())
		THROW_EXCEPTION("write operator[] not applicable to non-map nodes.");

	return yaml(internal::tag_as_proxy_t(), std::get<map_t>(n->d)[s], s);
}

const yaml yaml::operator[](const char* s) const
{
	ASSERT_(s != nullptr);
	const node_t* n = dereferenceProxy();
	if (n->isNullNode())
		THROW_EXCEPTION("read operator[] not applicable to null nodes.");
	if (!n->isMap())
		THROW_EXCEPTION("read operator[] only available for map nodes.");

	const map_t& m = std::get<map_t>(n->d);
	auto it = m.find(s);
	if (m.end() == it)
		THROW_EXCEPTION_FMT("Access non-existing map key `%s`", s);

	return yaml(internal::tag_as_const_proxy_t(), it->second, s);
}

yaml yaml::operator()(int index)
{
	node_t* n = dereferenceProxy();
	ASSERTMSG_(
		!n->isNullNode(),
		"write operator() not applicable to empty nodes or sequences.");

	ASSERTMSG_(
		n->isSequence(), "write operator() only available for sequence nodes.");

	sequence_t& seq = std::get<sequence_t>(n->d);
	if (index < 0 || index >= static_cast<int>(seq.size()))
		THROW_TYPED_EXCEPTION(
			"yaml::operator() out of range", std::out_of_range);

	return yaml(internal::tag_as_proxy_t(), seq.at(index), "");
}
const yaml yaml::operator()(int index) const
{
	const node_t* n = dereferenceProxy();
	ASSERTMSG_(
		!n->isNullNode(),
		"read operator() not applicable to empty nodes or sequences.");

	ASSERTMSG_(
		n->isSequence(), "read operator() only available for sequence nodes.");

	const sequence_t& seq = std::get<sequence_t>(n->d);
	if (index < 0 || index >= static_cast<int>(seq.size()))
		THROW_TYPED_EXCEPTION(
			"yaml::operator() out of range", std::out_of_range);

	return yaml(internal::tag_as_const_proxy_t(), seq.at(index), "");
}

void yaml::printAsYAML() const { printAsYAML(std::cout); }

void yaml::printAsYAML(std::ostream& o, bool di) const
{
	const node_t* n = dereferenceProxy();
	yaml::internalPrintNodeAsYAML(*n, o, 0, true, di);
}

bool yaml::internalPrintNodeAsYAML(
	const node_t& p, std::ostream& o, int indent, bool first, bool di)
{
	if (di) o << "[printNode] type=`" << p.typeName() << "`\n";

	if (p.isScalar())
		return internalPrintAsYAML(
			std::get<scalar_t>(p.d), o, indent, first, di);

	if (p.isMap())
		return internalPrintAsYAML(std::get<map_t>(p.d), o, indent, first, di);

	if (p.isSequence())
		return internalPrintAsYAML(
			std::get<sequence_t>(p.d), o, indent, first, di);

	if (p.isNullNode())
		return internalPrintAsYAML(std::monostate(), o, indent, first, di);

	THROW_EXCEPTION("Should never reach here");
}

bool yaml::internalPrintAsYAML(
	const std::monostate&, std::ostream& o, [[maybe_unused]] int indent,
	[[maybe_unused]] bool first, [[maybe_unused]] bool di)
{
	o << "~";
	return false;
}
bool yaml::internalPrintAsYAML(
	const yaml::sequence_t& v, std::ostream& o, int indent, bool first, bool di)
{
	if (di) o << "[printSequence] size=" << v.size() << "\n";

	if (!first)
	{
		o << "\n";
		indent += 2;
	}

	const std::string sInd(indent, ' ');
	for (const auto& e : v)
	{
		o << sInd << "- ";
		if (!internalPrintNodeAsYAML(e, o, indent, false, di)) o << "\n";
	}
	return true;
}
bool yaml::internalPrintAsYAML(
	const yaml::map_t& m, std::ostream& o, int indent, bool first, bool di)
{
	if (di) o << "[printMap] size=" << m.size() << "\n";

	if (!first)
	{
		o << "\n";
		indent += 2;
	}

	const std::string sInd(indent, ' ');
	for (const auto& kv : m)
	{
		const node_t& v = kv.second;

		if (v.comment.has_value())
		{
			o << sInd << "# ";
			o << v.comment.value() << "\n";
		}
		o << sInd << kv.first << ": ";
		if (!internalPrintNodeAsYAML(v, o, indent, false, di)) o << "\n";
	}
	return true;
}

bool yaml::internalPrintAsYAML(
	const yaml::scalar_t& v, std::ostream& o, int indent, bool first, bool di)
{
	if (di)
		o << "[printScalar] type=`" << mrpt::demangle(v.type().name()) << "`\n";

	if (!v.has_value())
	{
		o << "~";
		return false;
	}

	if (v.type() == typeid(yaml))
		return internalPrintNodeAsYAML(
			*std::any_cast<yaml>(v).dereferenceProxy(), o, indent, first, di);
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

yaml yaml::FromYAMLText(const std::string& yamlTextBlock)
{
#if MRPT_HAS_YAMLCPP
	YAML::Node n = YAML::Load(yamlTextBlock);
	return yaml::FromYAMLCPP(n);
#else
	THROW_EXCEPTION("MRPT was built without yaml-cpp");
#endif
}

yaml yaml::FromYAMLCPP(const YAML::Node& n)
{
#if MRPT_HAS_YAMLCPP
	const auto invalidDbl = std::numeric_limits<double>::max();

	if (n.IsSequence())
	{
		yaml ret = yaml(Sequence());

		for (const auto& e : n)
		{
			if (e.IsNull())
			{
				sequence_t& seq =
					std::get<sequence_t>(ret.dereferenceProxy()->d);
				seq.push_back(node_t());
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
				ret.push_back(yaml::FromYAMLCPP(e));
			}
		}
		return ret;
	}
	else if (n.IsMap())
	{
		yaml ret = yaml(yaml::Map());

		for (const auto& kv : n)
		{
			const auto& key = kv.first.as<std::string>();
			const auto& val = kv.second;

			if (val.IsNull())
			{
				map_t& m = std::get<map_t>(ret.dereferenceProxy()->d);
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
				ret[key] = yaml::FromYAMLCPP(val);
			}
		}
		return ret;
	}
	else
	{
		THROW_EXCEPTION(
			"FromYAMLCPP only supports root YAML as sequence or map");
	}

#else
	THROW_EXCEPTION("MRPT was built without yaml-cpp");
#endif
}

void yaml::loadFromYAMLCPP(const YAML::Node& n)
{
	*this = yaml::FromYAMLCPP(n);
}

bool yaml::hasComment() const
{
	const node_t* n = dereferenceProxy();
	return n->comment.has_value();
}

const std::string& yaml::comment()
{
	node_t* n = dereferenceProxy();
	ASSERTMSG_(n->comment.has_value(), "");
	return n->comment.value();
}

void yaml::comment(const std::string_view& c)
{
	node_t* n = dereferenceProxy();
	n->comment.emplace(c);
}

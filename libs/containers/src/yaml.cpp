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
#include <mrpt/containers/yaml.h>
#include <mrpt/core/exceptions.h>

#include <fstream>
#include <iostream>
#include <istream>
#include <stack>

#if MRPT_HAS_FYAML
#include <libfyaml.h>
#endif

using namespace mrpt::containers;

// ============ class: yaml::node_t =======
bool yaml::node_t::isNullNode() const
{
	return std::holds_alternative<std::monostate>(d) ||
		   (std::holds_alternative<scalar_t>(d) &&
			!std::get<scalar_t>(d).has_value());
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

size_t yaml::node_t::size() const
{
	if (isScalar() || isNullNode())
		return 1;
	else if (isMap())
		return asMap().size();
	else
		return asSequence().size();
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

size_t yaml::size() const { return dereferenceProxy()->size(); }

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
	*n = node_t();
}

yaml& yaml::operator=(bool v) { return implOpAssign(v); }

yaml& yaml::operator=(double v) { return implOpAssign(v); }
yaml& yaml::operator=(float v) { return implOpAssign(v); }

yaml& yaml::operator=(int8_t v) { return implOpAssign(v); }
yaml& yaml::operator=(uint8_t v) { return implOpAssign(v); }

yaml& yaml::operator=(int16_t v) { return implOpAssign(v); }
yaml& yaml::operator=(uint16_t v) { return implOpAssign(v); }

yaml& yaml::operator=(int32_t v) { return implOpAssign(v); }
yaml& yaml::operator=(uint32_t v) { return implOpAssign(v); }

yaml& yaml::operator=(int64_t v) { return implOpAssign(v); }
yaml& yaml::operator=(uint64_t v) { return implOpAssign(v); }

yaml& yaml::operator=(const std::string& v) { return implOpAssign(v); }
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
	bool lastLineNL = yaml::internalPrintNodeAsYAML(*n, o, 0, true, di, false);
	if (!lastLineNL) o << "\n";
}

bool yaml::internalPrintNodeAsYAML(
	const node_t& p, std::ostream& o, int indent, bool first, bool di,
	bool needsSpace)
{
	if (di) o << "[printNode] type=`" << p.typeName() << "`\n";

	const auto& rc = p.comments[static_cast<size_t>(CommentPosition::RIGHT)];

	if (p.isScalar())
		return internalPrintAsYAML(
			std::get<scalar_t>(p.d), o, indent, first, di, rc, needsSpace);

	if (p.isMap())
		return internalPrintAsYAML(
			std::get<map_t>(p.d), o, indent, first, di, rc, needsSpace);

	if (p.isSequence())
		return internalPrintAsYAML(
			std::get<sequence_t>(p.d), o, indent, first, di, rc, needsSpace);

	if (p.isNullNode())
		return internalPrintAsYAML(
			std::monostate(), o, indent, first, di, rc, needsSpace);

	THROW_EXCEPTION("Should never reach here");
}

bool yaml::internalPrintAsYAML(
	const std::monostate&, std::ostream& o, [[maybe_unused]] int indent,
	[[maybe_unused]] bool first, [[maybe_unused]] bool di,
	const std::optional<std::string>& rightComment, bool needsSpace)
{
	if (needsSpace) o << " ";

	o << "~";

	if (rightComment)
	{
		o << " # " << rightComment.value() << "\n";
		return true;  // \n already emitted.
	}
	else
	{
		return false;  // \n not emitted.
	}
}
bool yaml::internalPrintAsYAML(
	const yaml::sequence_t& v, std::ostream& o, int indent, bool first, bool di,
	const std::optional<std::string>& rightComment,
	[[maybe_unused]] bool needsSpace)
{
	if (di) o << "[printSequence] size=" << v.size() << "\n";

	if (!first)
	{
		if (rightComment)
			o << " # " << rightComment.value() << "\n";
		else
			o << "\n";

		indent += 2;
		// needsSpace = false;
	}

	const std::string sInd(indent, ' ');
	for (const auto& e : v)
	{
		o << sInd << "- ";
		if (!internalPrintNodeAsYAML(e, o, indent, false, di, false)) o << "\n";
	}
	return true;
}
bool yaml::internalPrintAsYAML(
	const yaml::map_t& m, std::ostream& o, int indent, bool first, bool di,
	const std::optional<std::string>& rightComment,
	[[maybe_unused]] bool needsSpace)
{
	if (di) o << "[printMap] size=" << m.size() << "\n";

	if (!first)
	{
		if (rightComment)
			o << " # " << rightComment.value() << "\n";
		else
			o << "\n";
		indent += 2;
	}

	const std::string sInd(indent, ' ');
	for (const auto& kv : m)
	{
		const node_t& v = kv.second;

		if (const auto& c =
				v.comments[static_cast<size_t>(CommentPosition::TOP)];
			c.has_value())
		{
			o << sInd << "# " << c.value() << "\n";
		}
		o << sInd << kv.first << ":";
		bool r = internalPrintNodeAsYAML(v, o, indent, false, di, true);

		if (!r) o << "\n";
	}
	return true;
}

bool yaml::internalPrintAsYAML(
	const yaml::scalar_t& v, std::ostream& o, int indent, bool first, bool di,
	const std::optional<std::string>& rightComment, bool needsSpace)
{
	if (di)
		o << "[printScalar] type=`" << mrpt::demangle(v.type().name()) << "`\n";

	if (!v.has_value())
	{
		return internalPrintAsYAML(
			std::monostate(), o, indent, first, di, rightComment, needsSpace);
	}

	if (v.type() == typeid(yaml))
		return internalPrintNodeAsYAML(
			*std::any_cast<yaml>(v).dereferenceProxy(), o, indent, first, di,
			needsSpace);

	if (needsSpace) o << " ";

	if (v.type() == typeid(bool))
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

	if (rightComment)
	{
		o << " # " << rightComment.value() << "\n";
		return true;  // \n already emitted
	}
	else
		return false;  // \n not emitted
}

yaml yaml::FromText(const std::string& yamlTextBlock)
{
	MRPT_START
	yaml doc;
	doc.loadFromText(yamlTextBlock);
	return doc;
	MRPT_END
}

// TODO: Allow users to add custom filters?
static yaml::scalar_t textToScalar(const std::string& s)
{
	// tag:yaml.org,2002:null
	// https://yaml.org/spec/1.2/spec.html#id2803362
	if (s == "~" || s == "null" || s == "Null" || s == "NULL") return {};

	// TODO: Try to parse to int or double?

	return {s};
}

#if MRPT_HAS_FYAML
static std::optional<yaml::node_t> recursiveParse(struct fy_parser* p);

static std::optional<std::string> extractComment(
	struct fy_token* t, enum fy_comment_placement cp)
{
	std::array<char, 2048> str;
	const char* strEnd = fy_token_get_comment(t, str.data(), str.size(), cp);
	if (strEnd == str.data()) return {};

	std::string c(str.data(), strEnd - str.data());
	if (c.size() >= 2 && c[0] == '#' && isblank(c[1])) c.erase(0, 2);

	return c;
}

static std::optional<yaml::node_t> recursiveParse(struct fy_parser* p)
{
	MRPT_START

#if 0  // debug
#define PARSER_DBG_OUT(STR_) std::cout << ">> " << STR_ << "\n"
#else
#define PARSER_DBG_OUT(STR) while (0)
#endif

	struct fy_event* event = fy_parser_parse(p);
	if (!event) return {};

	// process event:
	switch (event->type)
	{
		case FYET_NONE:
		{
			PARSER_DBG_OUT("Event: None");
			fy_parser_event_free(p, event);  // free event
			return recursiveParse(p);  // Keep going
		}
		break;
		case FYET_STREAM_START:
		{
			PARSER_DBG_OUT("Event: Stream start");
			fy_parser_event_free(p, event);  // free event
			return recursiveParse(p);  // Keep going
		}
		break;
		case FYET_STREAM_END:
		{
			PARSER_DBG_OUT("Event: Stream end");
			fy_parser_event_free(p, event);  // free event
			return recursiveParse(p);  // Keep going
		}
		break;
		case FYET_DOCUMENT_START:
		{
			PARSER_DBG_OUT("Event: Doc start");
			fy_parser_event_free(p, event);  // free event
			return recursiveParse(p);  // Keep going
		}
		break;
		case FYET_DOCUMENT_END:
		{
			PARSER_DBG_OUT("Event: Doc end");
			fy_parser_event_free(p, event);  // free event
			return recursiveParse(p);  // Keep going
		}
		break;
		case FYET_MAPPING_START:
		{
			PARSER_DBG_OUT("Event: MAP START");
			fy_parser_event_free(p, event);  // free event

			yaml::node_t n;
			yaml::map_t& m = n.d.emplace<yaml::map_t>();

			for (;;)
			{
				// Next event is map key:
				auto nKey = recursiveParse(p);
				// end of map reached?
				if (!nKey.has_value()) break;

				ASSERT_(nKey->isScalar());

				yaml::node_t& val = m[nKey->as<std::string>()];

				// and next event is mapped content:
				auto nVal = recursiveParse(p);
				ASSERT_(nVal.has_value());

				val = std::move(nVal.value());
			}

			return n;
		}
		break;
		case FYET_MAPPING_END:
		{
			PARSER_DBG_OUT("Event: MAP END");
			fy_parser_event_free(p, event);  // free event
			return {};
		}
		break;
		case FYET_SEQUENCE_START:
		{
			PARSER_DBG_OUT("Event: SEQ START");
			fy_parser_event_free(p, event);  // free event
			yaml::node_t n;
			yaml::sequence_t& s = n.d.emplace<yaml::sequence_t>();

			for (;;)
			{
				auto entry = recursiveParse(p);
				if (!entry.has_value()) break;  // end of sequence reached?
				s.push_back(std::move(entry.value()));
			}

			return n;
		}
		break;
		case FYET_SEQUENCE_END:
		{
			PARSER_DBG_OUT("Event: SEQ END");
			fy_parser_event_free(p, event);  // free event
			return {};
		}
		break;
		case FYET_SCALAR:
		{
			size_t strValueLen = 0;
			const char* strValue =
				fy_token_get_text(event->scalar.value, &strValueLen);
			const std::string sValue(strValue, strValueLen);

			PARSER_DBG_OUT(
				">> Scalar: implicit="
				<< (event->scalar.tag_implicit ? "1" : "0")
				<< " tag: " << ((void*)event->scalar.tag)
				<< " anchor: " << ((void*)event->scalar.anchor)
				<< fy_token_get_text0(event->scalar.anchor)
				<< " value: " << sValue);

			if (event->scalar.value)
			{
				yaml::node_t n;
				n.d.emplace<yaml::scalar_t>(textToScalar(sValue));

				{
					auto cT = extractComment(event->scalar.value, fycp_top);
					if (cT)
						n.comments[static_cast<size_t>(CommentPosition::TOP)] =
							std::move(cT.value());
				}
				{
					auto cR = extractComment(event->scalar.value, fycp_right);
					if (cR)
						n.comments[static_cast<size_t>(
							CommentPosition::RIGHT)] = std::move(cR.value());
				}

				fy_parser_event_free(p, event);  // free event
				return n;
			}
			else
			{
				THROW_EXCEPTION("Unexpected empty scalar?!");
			}
		}
		break;
		case FYET_ALIAS:
			fy_parser_event_free(p, event);  // free event
			return recursiveParse(p);  // Keep going
	};

	THROW_EXCEPTION_FMT("Unexpected parser event type %i", event->type);

#undef PARSER_DBG_OUT
	MRPT_END
}
#endif

void yaml::loadFromText(const std::string& yamlTextBlock)
{
	MRPT_START
#if MRPT_HAS_FYAML

	// Reset:
	*this = yaml();

	struct fy_parse_cfg cfg;
	cfg.search_path = "";
	cfg.diag = nullptr;
	cfg.flags = FYPCF_PARSE_COMMENTS;

	struct fy_parser* parser = fy_parser_create(&cfg);
	ASSERT_(parser);

	if (fy_parser_set_string(
			parser, yamlTextBlock.data(), yamlTextBlock.size()))
		THROW_EXCEPTION("Error in fy_parser_set_string()");

	auto optNode = recursiveParse(parser);
	if (optNode.has_value()) root_ = std::move(optNode.value());

	fy_parser_destroy(parser);

#else
	THROW_EXCEPTION("MRPT was built without libfyaml");
#endif

	MRPT_END
}

yaml yaml::FromStream(std::istream& i)
{
	MRPT_START
	yaml doc;
	doc.loadFromStream(i);
	return doc;
	MRPT_END
}

// Replicated from mrpt::io to avoid dependency to that module:
static std::string local_file_get_contents(const std::string& fileName)
{
	// Credits: https://stackoverflow.com/a/2602258/1631514
	// Note: Add "binary" to make sure the "tellg" file size matches the actual
	// number of read bytes afterwards:
	std::ifstream t(fileName, std::ios::binary);
	if (!t.is_open())
		THROW_EXCEPTION_FMT(
			"file_get_contents(): Error opening for read file `%s`",
			fileName.c_str());

	t.seekg(0, std::ios::end);
	std::size_t size = t.tellg();
	std::string buffer(size, ' ');
	t.seekg(0);
	t.read(&buffer[0], size);
	return buffer;
}

void yaml::loadFromFile(const std::string& fileName)
{
	MRPT_START
	clear();
	this->loadFromText(local_file_get_contents(fileName));
	MRPT_END
}

yaml yaml::FromFile(const std::string& fileName)
{
	MRPT_START
	yaml doc;
	doc.loadFromFile(fileName);
	return doc;
	MRPT_END
}

void yaml::loadFromStream(std::istream& i)
{
	MRPT_START
	std::string str;

	i.seekg(0, std::ios::end);
	str.reserve(i.tellg());
	i.seekg(0, std::ios::beg);
	str.assign(
		(std::istreambuf_iterator<char>(i)), std::istreambuf_iterator<char>());

	this->loadFromText(str);
	MRPT_END
}

bool yaml::hasComment() const
{
	const node_t* n = dereferenceProxy();
	for (const auto& c : n->comments)
		if (c.has_value()) return true;
	return false;
}

bool yaml::hasComment(CommentPosition pos) const
{
	MRPT_START
	int posIndex = static_cast<int>(pos);
	ASSERT_GE_(posIndex, 0);
	ASSERT_LT_(posIndex, static_cast<int>(CommentPosition::MAX));

	const node_t* n = dereferenceProxy();
	return n->comments[posIndex].has_value();
	MRPT_END
}

const std::string& yaml::comment() const
{
	MRPT_START
	const node_t* n = dereferenceProxy();
	for (const auto& c : n->comments)
		if (c.has_value()) return c.value();

	THROW_EXCEPTION("Trying to access comment but this node has none.");
	MRPT_END
}

const std::string& yaml::comment(CommentPosition pos) const
{
	MRPT_START
	const node_t* n = dereferenceProxy();

	int posIndex = static_cast<int>(pos);
	ASSERT_GE_(posIndex, 0);
	ASSERT_LT_(posIndex, static_cast<int>(CommentPosition::MAX));

	ASSERTMSG_(
		n->comments[posIndex].has_value(),
		"Trying to access comment but this node has none.");
	return n->comments[posIndex].value();
	MRPT_END
}

void yaml::comment(const std::string_view& c, CommentPosition position)
{
	int posIndex = static_cast<int>(position);
	ASSERT_GE_(posIndex, 0);
	ASSERT_LT_(posIndex, static_cast<int>(CommentPosition::MAX));

	node_t* n = dereferenceProxy();
	n->comments[posIndex].emplace(c);
}

std::ostream& mrpt::containers::operator<<(std::ostream& o, const yaml& p)
{
	p.printAsYAML(o);
	return o;
}

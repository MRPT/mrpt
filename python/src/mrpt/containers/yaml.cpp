#include <any>
#include <functional>
#include <ios>
#include <istream>
#include <iterator>
#include <locale>
#include <map>
#include <memory>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <string_view>
#include <type_traits>
#include <typeinfo>
#include <utility>
#include <vector>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_containers_yaml(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::containers::yaml file:mrpt/containers/yaml.h line:85
		pybind11::class_<mrpt::containers::yaml, std::shared_ptr<mrpt::containers::yaml>> cl(M("mrpt::containers"), "yaml", "Powerful YAML-like container for possibly-nested blocks of parameters or\nany arbitrary structured data contents, including documentation in the\nform of comments attached to each node. Supports parsing from YAML or JSON\nstreams, files, or text strings.\n\n This class holds the root \"node\" in a YAML-like tree structure.\n Each tree node can be of one of these types:\n - Scalar values (\"leaf nodes\"): Can hold any type, stored as C++17 std::any.\n - Sequence container.\n - Map (\"dictionary\"): pairs of `name: value`.\n - Null, empty nodes: yaml `~` or `null`.\n\n Sequences and dictionaries can hold, in turn, any of the four types above,\n leading to arbitrarialy-complex nested structures.\n\n This class was designed as a lightweight, while structured, way to pass\narbitrarialy-complex parameter blocks but can be used to load and save\nYAML files or as a database.\n\n yaml can be used to parse YAML (v1.2) or JSON streams, and to emit YAML.\n It does not support event-based parsing.\n The parser uses Pantelis Antoniou's awesome\n[libfyaml](https://github.com/pantoniou/libfyaml), which\n[passes](http://matrix.yaml.io/) the full [YAML\ntestsuite](https://github.com/yaml/yaml-test-suite).\n\n Known limitations:\n - *Parsing* comments is limited to right-hand comments for *sequence* or\n   *map* entries.\n\n See examples below (\n \n\n\n Output:\n  \n\n\n Verbose debug information on YAML document parsing is emitted if the\n environment variable `MRPT_YAML_PARSER_VERBOSE` is set to `1`.\n\n \n\n \n [New in MRPT 2.1.0]");
		cl.def( pybind11::init( [](){ return new mrpt::containers::yaml(); } ) );
		cl.def( pybind11::init( [](mrpt::containers::yaml const &o){ return new mrpt::containers::yaml(o); } ) );
		cl.def( pybind11::init<const struct mrpt::containers::yaml::node_t &>(), pybind11::arg("s") );

		cl.def("getOrDefault", (const bool (mrpt::containers::yaml::*)(const std::string &, const bool &) const) &mrpt::containers::yaml::getOrDefault<bool>, "C++: mrpt::containers::yaml::getOrDefault(const std::string &, const bool &) const --> const bool", pybind11::arg("key"), pybind11::arg("defaultValue"));
		cl.def("getOrDefault", (const double (mrpt::containers::yaml::*)(const std::string &, const double &) const) &mrpt::containers::yaml::getOrDefault<double>, "C++: mrpt::containers::yaml::getOrDefault(const std::string &, const double &) const --> const double", pybind11::arg("key"), pybind11::arg("defaultValue"));
		cl.def("getOrDefault", (const unsigned int (mrpt::containers::yaml::*)(const std::string &, const unsigned int &) const) &mrpt::containers::yaml::getOrDefault<unsigned int>, "C++: mrpt::containers::yaml::getOrDefault(const std::string &, const unsigned int &) const --> const unsigned int", pybind11::arg("key"), pybind11::arg("defaultValue"));
		cl.def("as", (bool (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::as<bool>, "C++: mrpt::containers::yaml::as() const --> bool");
		cl.def("as", (double (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::as<double>, "C++: mrpt::containers::yaml::as() const --> double");
		cl.def("as", (float (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::as<float>, "C++: mrpt::containers::yaml::as() const --> float");
		cl.def("as", (signed char (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::as<signed char>, "C++: mrpt::containers::yaml::as() const --> signed char");
		cl.def("as", (unsigned char (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::as<unsigned char>, "C++: mrpt::containers::yaml::as() const --> unsigned char");
		cl.def("as", (short (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::as<short>, "C++: mrpt::containers::yaml::as() const --> short");
		cl.def("as", (unsigned short (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::as<unsigned short>, "C++: mrpt::containers::yaml::as() const --> unsigned short");
		cl.def("as", (int (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::as<int>, "C++: mrpt::containers::yaml::as() const --> int");
		cl.def("as", (unsigned int (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::as<unsigned int>, "C++: mrpt::containers::yaml::as() const --> unsigned int");
		cl.def("as", (long (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::as<long>, "C++: mrpt::containers::yaml::as() const --> long");
		cl.def("as", (unsigned long (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::as<unsigned long>, "C++: mrpt::containers::yaml::as() const --> unsigned long");
		cl.def("as", (std::string (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::as<std::string>, "C++: mrpt::containers::yaml::as() const --> std::string");
		cl.def("assign", (class mrpt::containers::yaml & (mrpt::containers::yaml::*)(std::size_t)) &mrpt::containers::yaml::operator=<std::enable_if<false>>, "C++: mrpt::containers::yaml::operator=(std::size_t) --> class mrpt::containers::yaml &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def_static("Sequence", (struct mrpt::containers::yaml::node_t (*)()) &mrpt::containers::yaml::Sequence, "C++: mrpt::containers::yaml::Sequence() --> struct mrpt::containers::yaml::node_t");
		cl.def_static("Map", (struct mrpt::containers::yaml::node_t (*)()) &mrpt::containers::yaml::Map, "C++: mrpt::containers::yaml::Map() --> struct mrpt::containers::yaml::node_t");
		cl.def_static("FromText", (class mrpt::containers::yaml (*)(const std::string &)) &mrpt::containers::yaml::FromText, "Parses a text as YAML or JSON (autodetected) and returns a document.\n \n\n std::exception Upon format errors\n\nC++: mrpt::containers::yaml::FromText(const std::string &) --> class mrpt::containers::yaml", pybind11::arg("yamlTextBlock"));
		cl.def("loadFromText", (void (mrpt::containers::yaml::*)(const std::string &)) &mrpt::containers::yaml::loadFromText, "Parses a text as YAML or JSON (autodetected) and stores the contents\n into this document.\n\n \n std::exception Upon format errors\n\nC++: mrpt::containers::yaml::loadFromText(const std::string &) --> void", pybind11::arg("yamlTextBlock"));
		cl.def("loadFromFile", (void (mrpt::containers::yaml::*)(const std::string &)) &mrpt::containers::yaml::loadFromFile, "Parses a text as YAML or JSON (autodetected) and stores the contents\n into this document.\n\n \n std::exception Upon I/O or format errors\n\nC++: mrpt::containers::yaml::loadFromFile(const std::string &) --> void", pybind11::arg("fileName"));
		cl.def_static("FromFile", (class mrpt::containers::yaml (*)(const std::string &)) &mrpt::containers::yaml::FromFile, "Parses the filename as YAML or JSON (autodetected) and returns a\n document.\n \n\n std::exception Upon I/O or format errors.\n\nC++: mrpt::containers::yaml::FromFile(const std::string &) --> class mrpt::containers::yaml", pybind11::arg("fileName"));
		cl.def("has", (bool (mrpt::containers::yaml::*)(const std::string &) const) &mrpt::containers::yaml::has, "@{ \n\n For map nodes, checks if the given key name exists.\n  Returns false if the node is a `null` node.\n  Throws if the node is not a map or null.\n\nC++: mrpt::containers::yaml::has(const std::string &) const --> bool", pybind11::arg("key"));
		cl.def("empty", (bool (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::empty, "For map or sequence nodes, checks if the container is empty. Also\n returns true for null(empty) nodes. \n\nC++: mrpt::containers::yaml::empty() const --> bool");
		cl.def("clear", (void (mrpt::containers::yaml::*)()) &mrpt::containers::yaml::clear, "Resets to empty (can be called on a root node or any other node to\n clear that subtree only). \n\nC++: mrpt::containers::yaml::clear() --> void");
		cl.def("isNullNode", (bool (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::isNullNode, "C++: mrpt::containers::yaml::isNullNode() const --> bool");
		cl.def("isScalar", (bool (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::isScalar, "C++: mrpt::containers::yaml::isScalar() const --> bool");
		cl.def("isSequence", (bool (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::isSequence, "C++: mrpt::containers::yaml::isSequence() const --> bool");
		cl.def("isMap", (bool (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::isMap, "C++: mrpt::containers::yaml::isMap() const --> bool");
		cl.def("asMap", (class std::map<struct mrpt::containers::yaml::node_t, struct mrpt::containers::yaml::node_t> & (mrpt::containers::yaml::*)()) &mrpt::containers::yaml::asMap, "Use: `for (auto &kv: n.asMap()) {...}`\n \n\n std::exception If called on a non-map node. \n\nC++: mrpt::containers::yaml::asMap() --> class std::map<struct mrpt::containers::yaml::node_t, struct mrpt::containers::yaml::node_t> &", pybind11::return_value_policy::automatic);
		cl.def("asMapRange", (const class std::map<struct mrpt::containers::yaml::node_t, struct mrpt::containers::yaml::node_t> (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::asMapRange, "Returns a copy of asMap(), suitable for range-based loops\n\nC++: mrpt::containers::yaml::asMapRange() const --> const class std::map<struct mrpt::containers::yaml::node_t, struct mrpt::containers::yaml::node_t>");
		cl.def("size", (size_t (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::size, "Returns 1 for null or scalar nodes, the number of children for\n sequence or map nodes. \n\nC++: mrpt::containers::yaml::size() const --> size_t");
		cl.def("node", (struct mrpt::containers::yaml::node_t & (mrpt::containers::yaml::*)()) &mrpt::containers::yaml::node, "For a master yaml document, returns the root node; otherwise, the\n referenced node. \n\nC++: mrpt::containers::yaml::node() --> struct mrpt::containers::yaml::node_t &", pybind11::return_value_policy::automatic);
		cl.def("keyNode", (struct mrpt::containers::yaml::node_t & (mrpt::containers::yaml::*)(const std::string &)) &mrpt::containers::yaml::keyNode, "C++: mrpt::containers::yaml::keyNode(const std::string &) --> struct mrpt::containers::yaml::node_t &", pybind11::return_value_policy::automatic, pybind11::arg("keyName"));
		cl.def("printAsYAML", (void (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::printAsYAML, "C++: mrpt::containers::yaml::printAsYAML() const --> void");
		cl.def("__getitem__", (class mrpt::containers::yaml (mrpt::containers::yaml::*)(const std::string &)) &mrpt::containers::yaml::operator[], "Write access for maps \n\nC++: mrpt::containers::yaml::operator[](const std::string &) --> class mrpt::containers::yaml", pybind11::arg("key"));
		cl.def("__getitem__", (class mrpt::containers::yaml (mrpt::containers::yaml::*)(const char *)) &mrpt::containers::yaml::operator[], "C++: mrpt::containers::yaml::operator[](const char *) --> class mrpt::containers::yaml", pybind11::arg("key"));
		cl.def("__call__", (class mrpt::containers::yaml (mrpt::containers::yaml::*)(int)) &mrpt::containers::yaml::operator(), "Write into an existing index of a sequence.\n \n\n std::out_of_range if index is out of range. \n\nC++: mrpt::containers::yaml::operator()(int) --> class mrpt::containers::yaml", pybind11::arg("index"));
		cl.def("push_back", (void (mrpt::containers::yaml::*)(double)) &mrpt::containers::yaml::push_back, "Append a new value to a sequence.\n \n\n std::exception If this is not a sequence \n\nC++: mrpt::containers::yaml::push_back(double) --> void", pybind11::arg("v"));
		cl.def("push_back", (void (mrpt::containers::yaml::*)(const std::string &)) &mrpt::containers::yaml::push_back, "C++: mrpt::containers::yaml::push_back(const std::string &) --> void", pybind11::arg("v"));
		cl.def("push_back", (void (mrpt::containers::yaml::*)(uint64_t)) &mrpt::containers::yaml::push_back, "C++: mrpt::containers::yaml::push_back(uint64_t) --> void", pybind11::arg("v"));
		cl.def("push_back", (void (mrpt::containers::yaml::*)(bool)) &mrpt::containers::yaml::push_back, "C++: mrpt::containers::yaml::push_back(bool) --> void", pybind11::arg("v"));
		cl.def("push_back", (void (mrpt::containers::yaml::*)(const class mrpt::containers::yaml &)) &mrpt::containers::yaml::push_back, "C++: mrpt::containers::yaml::push_back(const class mrpt::containers::yaml &) --> void", pybind11::arg("v"));
		cl.def("assign", (class mrpt::containers::yaml & (mrpt::containers::yaml::*)(bool)) &mrpt::containers::yaml::operator=, "C++: mrpt::containers::yaml::operator=(bool) --> class mrpt::containers::yaml &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("assign", (class mrpt::containers::yaml & (mrpt::containers::yaml::*)(float)) &mrpt::containers::yaml::operator=, "C++: mrpt::containers::yaml::operator=(float) --> class mrpt::containers::yaml &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("assign", (class mrpt::containers::yaml & (mrpt::containers::yaml::*)(double)) &mrpt::containers::yaml::operator=, "C++: mrpt::containers::yaml::operator=(double) --> class mrpt::containers::yaml &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("assign", (class mrpt::containers::yaml & (mrpt::containers::yaml::*)(int8_t)) &mrpt::containers::yaml::operator=, "C++: mrpt::containers::yaml::operator=(int8_t) --> class mrpt::containers::yaml &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("assign", (class mrpt::containers::yaml & (mrpt::containers::yaml::*)(uint8_t)) &mrpt::containers::yaml::operator=, "C++: mrpt::containers::yaml::operator=(uint8_t) --> class mrpt::containers::yaml &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("assign", (class mrpt::containers::yaml & (mrpt::containers::yaml::*)(int16_t)) &mrpt::containers::yaml::operator=, "C++: mrpt::containers::yaml::operator=(int16_t) --> class mrpt::containers::yaml &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("assign", (class mrpt::containers::yaml & (mrpt::containers::yaml::*)(uint16_t)) &mrpt::containers::yaml::operator=, "C++: mrpt::containers::yaml::operator=(uint16_t) --> class mrpt::containers::yaml &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("assign", (class mrpt::containers::yaml & (mrpt::containers::yaml::*)(int32_t)) &mrpt::containers::yaml::operator=, "C++: mrpt::containers::yaml::operator=(int32_t) --> class mrpt::containers::yaml &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("assign", (class mrpt::containers::yaml & (mrpt::containers::yaml::*)(uint32_t)) &mrpt::containers::yaml::operator=, "C++: mrpt::containers::yaml::operator=(uint32_t) --> class mrpt::containers::yaml &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("assign", (class mrpt::containers::yaml & (mrpt::containers::yaml::*)(int64_t)) &mrpt::containers::yaml::operator=, "C++: mrpt::containers::yaml::operator=(int64_t) --> class mrpt::containers::yaml &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("assign", (class mrpt::containers::yaml & (mrpt::containers::yaml::*)(uint64_t)) &mrpt::containers::yaml::operator=, "C++: mrpt::containers::yaml::operator=(uint64_t) --> class mrpt::containers::yaml &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("assign", (class mrpt::containers::yaml & (mrpt::containers::yaml::*)(const std::string &)) &mrpt::containers::yaml::operator=, "C++: mrpt::containers::yaml::operator=(const std::string &) --> class mrpt::containers::yaml &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("assign", (class mrpt::containers::yaml & (mrpt::containers::yaml::*)(const char *)) &mrpt::containers::yaml::operator=, "C++: mrpt::containers::yaml::operator=(const char *) --> class mrpt::containers::yaml &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("assign", (class mrpt::containers::yaml & (mrpt::containers::yaml::*)(const class mrpt::containers::yaml &)) &mrpt::containers::yaml::operator=, "C++: mrpt::containers::yaml::operator=(const class mrpt::containers::yaml &) --> class mrpt::containers::yaml &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("hasComment", (bool (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::hasComment, "Returns true if the proxied node has an associated comment block, at\n any location \n\nC++: mrpt::containers::yaml::hasComment() const --> bool");
		cl.def("hasComment", (bool (mrpt::containers::yaml::*)(enum mrpt::containers::CommentPosition) const) &mrpt::containers::yaml::hasComment, "Returns true if the proxied node has an associated comment block at\n a particular position \n\nC++: mrpt::containers::yaml::hasComment(enum mrpt::containers::CommentPosition) const --> bool", pybind11::arg("pos"));
		cl.def("comment", (const std::string & (mrpt::containers::yaml::*)() const) &mrpt::containers::yaml::comment, "Gets the comment associated to the proxied node. This version\n returns the first comment, of all possible (top, right).\n\n \n std::exception If there is no comment attached.\n \n\n hasComment()\n\nC++: mrpt::containers::yaml::comment() const --> const std::string &", pybind11::return_value_policy::automatic);
		cl.def("comment", (const std::string & (mrpt::containers::yaml::*)(enum mrpt::containers::CommentPosition) const) &mrpt::containers::yaml::comment, "Gets the comment associated to the proxied node, at the particular\n position. See code examples in mrpt::containers::yaml.\n\n \n std::exception If there is no comment attached.\n \n\n hasComment()\n\nC++: mrpt::containers::yaml::comment(enum mrpt::containers::CommentPosition) const --> const std::string &", pybind11::return_value_policy::automatic, pybind11::arg("pos"));
		cl.def("comment", [](mrpt::containers::yaml &o, const std::string & a0) -> void { return o.comment(a0); }, "", pybind11::arg("c"));
		cl.def("comment", (void (mrpt::containers::yaml::*)(const std::string &, enum mrpt::containers::CommentPosition)) &mrpt::containers::yaml::comment, "Sets the comment attached to a given proxied node.\n See code examples in mrpt::containers::yaml\n \n\n hasComment()\n\nC++: mrpt::containers::yaml::comment(const std::string &, enum mrpt::containers::CommentPosition) --> void", pybind11::arg("c"), pybind11::arg("position"));
		cl.def("keyHasComment", (bool (mrpt::containers::yaml::*)(const std::string &) const) &mrpt::containers::yaml::keyHasComment, "Maps only: returns true if the given key node has an associated\n comment block, at any location. \n\n std::exception If called\n on a non-map or key does not exist.\n\nC++: mrpt::containers::yaml::keyHasComment(const std::string &) const --> bool", pybind11::arg("key"));
		cl.def("keyHasComment", (bool (mrpt::containers::yaml::*)(const std::string &, enum mrpt::containers::CommentPosition) const) &mrpt::containers::yaml::keyHasComment, "Maps only: Returns true if the given key has an associated comment\n block at a particular position. \n\n std::exception If called\n on a non-map or key does not exist.\n\nC++: mrpt::containers::yaml::keyHasComment(const std::string &, enum mrpt::containers::CommentPosition) const --> bool", pybind11::arg("key"), pybind11::arg("pos"));
		cl.def("keyComment", (const std::string & (mrpt::containers::yaml::*)(const std::string &) const) &mrpt::containers::yaml::keyComment, "Maps only: Gets the comment associated to the given key. This\n version returns the first comment, of all possible (top, right).\n\n \n std::exception If called on a non-map or key does not\n exist. \n\n std::exception If there is no comment attached. \n\n hasComment()\n\nC++: mrpt::containers::yaml::keyComment(const std::string &) const --> const std::string &", pybind11::return_value_policy::automatic, pybind11::arg("key"));
		cl.def("keyComment", (const std::string & (mrpt::containers::yaml::*)(const std::string &, enum mrpt::containers::CommentPosition) const) &mrpt::containers::yaml::keyComment, "Maps only: Gets the comment associated to the given key, at the\n particular position. See code examples in mrpt::containers::yaml.\n\n \n std::exception If called on a non-map or key does not\n exist. \n\n std::exception If there is no comment attached. \n\n hasComment()\n\nC++: mrpt::containers::yaml::keyComment(const std::string &, enum mrpt::containers::CommentPosition) const --> const std::string &", pybind11::return_value_policy::automatic, pybind11::arg("key"), pybind11::arg("pos"));
		cl.def("keyComment", [](mrpt::containers::yaml &o, const std::string & a0, const std::string & a1) -> void { return o.keyComment(a0, a1); }, "", pybind11::arg("key"), pybind11::arg("c"));
		cl.def("keyComment", (void (mrpt::containers::yaml::*)(const std::string &, const std::string &, enum mrpt::containers::CommentPosition)) &mrpt::containers::yaml::keyComment, "Maps only: Sets the comment attached to a given key.\n See code examples in mrpt::containers::yaml\n\n \n std::exception If called on a non-map or key does not\n exist. \n\n hasComment()\n\nC++: mrpt::containers::yaml::keyComment(const std::string &, const std::string &, enum mrpt::containers::CommentPosition) --> void", pybind11::arg("key"), pybind11::arg("c"), pybind11::arg("position"));

		cl.def("__str__", [](mrpt::containers::yaml const &o) -> std::string { std::ostringstream s; using namespace mrpt::containers; s << o; return s.str(); } );

		{ // mrpt::containers::yaml::node_t file:mrpt/containers/yaml.h line:107
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::containers::yaml::node_t, std::shared_ptr<mrpt::containers::yaml::node_t>> cl(enclosing_class, "node_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::containers::yaml::node_t(); } ) );
			cl.def( pybind11::init<const char *>(), pybind11::arg("str") );

			cl.def( pybind11::init( [](mrpt::containers::yaml::node_t const &o){ return new mrpt::containers::yaml::node_t(o); } ) );
			cl.def_readwrite("d", &mrpt::containers::yaml::node_t::d);
			cl.def_readwrite("comments", &mrpt::containers::yaml::node_t::comments);
			cl.def_readwrite("printInShortFormat", &mrpt::containers::yaml::node_t::printInShortFormat);
			cl.def_readwrite("marks", &mrpt::containers::yaml::node_t::marks);
			cl.def("isNullNode", (bool (mrpt::containers::yaml::node_t::*)() const) &mrpt::containers::yaml::node_t::isNullNode, "C++: mrpt::containers::yaml::node_t::isNullNode() const --> bool");
			cl.def("isScalar", (bool (mrpt::containers::yaml::node_t::*)() const) &mrpt::containers::yaml::node_t::isScalar, "C++: mrpt::containers::yaml::node_t::isScalar() const --> bool");
			cl.def("isSequence", (bool (mrpt::containers::yaml::node_t::*)() const) &mrpt::containers::yaml::node_t::isSequence, "C++: mrpt::containers::yaml::node_t::isSequence() const --> bool");
			cl.def("isMap", (bool (mrpt::containers::yaml::node_t::*)() const) &mrpt::containers::yaml::node_t::isMap, "C++: mrpt::containers::yaml::node_t::isMap() const --> bool");
			cl.def("typeName", (std::string (mrpt::containers::yaml::node_t::*)() const) &mrpt::containers::yaml::node_t::typeName, "Returns: \"null\", \"sequence\", \"map\", \"scalar(<TYPE>)\" \n\nC++: mrpt::containers::yaml::node_t::typeName() const --> std::string");
			cl.def("asMap", (class std::map<struct mrpt::containers::yaml::node_t, struct mrpt::containers::yaml::node_t> & (mrpt::containers::yaml::node_t::*)()) &mrpt::containers::yaml::node_t::asMap, "Use: `for (auto &kv: n.asMap()) {...}`\n \n\n std::exception If called on a non-map node. \n\nC++: mrpt::containers::yaml::node_t::asMap() --> class std::map<struct mrpt::containers::yaml::node_t, struct mrpt::containers::yaml::node_t> &", pybind11::return_value_policy::automatic);
			cl.def("size", (size_t (mrpt::containers::yaml::node_t::*)() const) &mrpt::containers::yaml::node_t::size, "Returns 1 for null or scalar nodes, the number of children for\n sequence or map nodes. \n\nC++: mrpt::containers::yaml::node_t::size() const --> size_t");
			cl.def("hasComment", (bool (mrpt::containers::yaml::node_t::*)() const) &mrpt::containers::yaml::node_t::hasComment, "C++: mrpt::containers::yaml::node_t::hasComment() const --> bool");
			cl.def("hasComment", (bool (mrpt::containers::yaml::node_t::*)(enum mrpt::containers::CommentPosition) const) &mrpt::containers::yaml::node_t::hasComment, "C++: mrpt::containers::yaml::node_t::hasComment(enum mrpt::containers::CommentPosition) const --> bool", pybind11::arg("pos"));
			cl.def("comment", (const std::string & (mrpt::containers::yaml::node_t::*)() const) &mrpt::containers::yaml::node_t::comment, "C++: mrpt::containers::yaml::node_t::comment() const --> const std::string &", pybind11::return_value_policy::automatic);
			cl.def("comment", (const std::string & (mrpt::containers::yaml::node_t::*)(enum mrpt::containers::CommentPosition) const) &mrpt::containers::yaml::node_t::comment, "C++: mrpt::containers::yaml::node_t::comment(enum mrpt::containers::CommentPosition) const --> const std::string &", pybind11::return_value_policy::automatic, pybind11::arg("pos"));
			cl.def("assign", (struct mrpt::containers::yaml::node_t & (mrpt::containers::yaml::node_t::*)(const struct mrpt::containers::yaml::node_t &)) &mrpt::containers::yaml::node_t::operator=, "C++: mrpt::containers::yaml::node_t::operator=(const struct mrpt::containers::yaml::node_t &) --> struct mrpt::containers::yaml::node_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::containers::yaml::mark_t file:mrpt/containers/yaml.h line:99
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::containers::yaml::mark_t, std::shared_ptr<mrpt::containers::yaml::mark_t>> cl(enclosing_class, "mark_t", "");
			cl.def( pybind11::init( [](mrpt::containers::yaml::mark_t const &o){ return new mrpt::containers::yaml::mark_t(o); } ) );
			cl.def( pybind11::init( [](){ return new mrpt::containers::yaml::mark_t(); } ) );
			cl.def_readwrite("input_pos", &mrpt::containers::yaml::mark_t::input_pos);
			cl.def_readwrite("line", &mrpt::containers::yaml::mark_t::line);
			cl.def_readwrite("column", &mrpt::containers::yaml::mark_t::column);
			cl.def("assign", (struct mrpt::containers::yaml::mark_t & (mrpt::containers::yaml::mark_t::*)(const struct mrpt::containers::yaml::mark_t &)) &mrpt::containers::yaml::mark_t::operator=, "C++: mrpt::containers::yaml::mark_t::operator=(const struct mrpt::containers::yaml::mark_t &) --> struct mrpt::containers::yaml::mark_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}

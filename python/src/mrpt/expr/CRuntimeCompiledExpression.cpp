#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/expr/CRuntimeCompiledExpression.h>
#include <sstream> // __str__
#include <string>
#include <typeinfo>
#include <utility>

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

void bind_mrpt_expr_CRuntimeCompiledExpression(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::expr::CRuntimeCompiledExpression file:mrpt/expr/CRuntimeCompiledExpression.h line:63
		pybind11::class_<mrpt::expr::CRuntimeCompiledExpression, std::shared_ptr<mrpt::expr::CRuntimeCompiledExpression>> cl(M("mrpt::expr"), "CRuntimeCompiledExpression", "A wrapper of `exprtk` runtime expression compiler: it takes a string\n representing an expression (from a simple mathematical formula to a complete\n program), compiles it and evaluates its result as many times as required. The\n result will change as the \"variables\" appearing in the expression (hold and\n managed by the user of this object) change.\n\n Refer to [exprtk documentation](https://github.com/ArashPartow/exprtk) for\n reference on supported formulas, control flow instructions, etc.\n\n This wrapper is provided to reduce the (very large) compilation time and\n memory required by the original library, at the cost of only exposing the\n most commonly used part of its API:\n  - Only expressions returning `double` are supported.\n  - Variables must be provided via a `std::map` container **or** pointers to\n user-stored variables.\n  - Custom user-defined functions taking 0-3 arguments (New in MRPT 2.5.8).\n\n See examples of usage in the [unit test\n file](https://github.com/MRPT/mrpt/blob/master/libs/base/src/math/CRuntimeCompiledExpression_unittest.cpp).\n\n If the environment variable `MRPT_EXPR_VERBOSE=1` is defined, debug\n information will be dumped to std::cout explaining the values of **all** the\n involved variables upon each call to `eval()`.\n Alternatively, the env var `MRPT_EXPR_VERBOSE` can be set to a list of\n terms split by `|`, and only those formulas that match (i.e. contain as\n substrings) any of the terms will be traced.\n Example: `MRPT_EXPR_VERBOSE=\"cos|sin|speed|if (x>0)\"`.\n\n \n (New in MRPT 1.5.0)\n \n\n (`MRPT_EXPR_VERBOSE` new in MRPT 1.5.7)\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::expr::CRuntimeCompiledExpression(); } ) );
		cl.def( pybind11::init( [](mrpt::expr::CRuntimeCompiledExpression const &o){ return new mrpt::expr::CRuntimeCompiledExpression(o); } ) );
		cl.def("compile", [](mrpt::expr::CRuntimeCompiledExpression &o, const std::string & a0) -> void { return o.compile(a0); }, "", pybind11::arg("expression"));
		cl.def("compile", [](mrpt::expr::CRuntimeCompiledExpression &o, const std::string & a0, const class std::map<std::string, double> & a1) -> void { return o.compile(a0, a1); }, "", pybind11::arg("expression"), pybind11::arg("variables"));
		cl.def("compile", (void (mrpt::expr::CRuntimeCompiledExpression::*)(const std::string &, const class std::map<std::string, double> &, const std::string &)) &mrpt::expr::CRuntimeCompiledExpression::compile, "Initializes the object by compiling an expression.\n \n\n std::runtime_error On any syntax error or undefined symbol\n while compiling the expression. The `e.what()` message describes what is\n exactly the problem.\n \n\n register_symbol_table()\n\nC++: mrpt::expr::CRuntimeCompiledExpression::compile(const std::string &, const class std::map<std::string, double> &, const std::string &) --> void", pybind11::arg("expression"), pybind11::arg("variables"), pybind11::arg("expr_name_for_error_reporting"));
		cl.def("register_function", (void (mrpt::expr::CRuntimeCompiledExpression::*)(const std::string &, const class std::function<double (void)> &)) &mrpt::expr::CRuntimeCompiledExpression::register_function, "Register a user-defined nullary function. (New in MRPT 2.5.8) \n\nC++: mrpt::expr::CRuntimeCompiledExpression::register_function(const std::string &, const class std::function<double (void)> &) --> void", pybind11::arg("name"), pybind11::arg("func"));
		cl.def("register_function", (void (mrpt::expr::CRuntimeCompiledExpression::*)(const std::string &, const class std::function<double (double)> &)) &mrpt::expr::CRuntimeCompiledExpression::register_function, "Register a user-defined unary function. (New in MRPT 2.5.8) \n\nC++: mrpt::expr::CRuntimeCompiledExpression::register_function(const std::string &, const class std::function<double (double)> &) --> void", pybind11::arg("name"), pybind11::arg("func"));
		cl.def("register_function", (void (mrpt::expr::CRuntimeCompiledExpression::*)(const std::string &, const class std::function<double (double, double)> &)) &mrpt::expr::CRuntimeCompiledExpression::register_function, "Register a user-defined binary function. (New in MRPT 2.5.8) \n\nC++: mrpt::expr::CRuntimeCompiledExpression::register_function(const std::string &, const class std::function<double (double, double)> &) --> void", pybind11::arg("name"), pybind11::arg("func"));
		cl.def("register_function", (void (mrpt::expr::CRuntimeCompiledExpression::*)(const std::string &, const class std::function<double (double, double, double)> &)) &mrpt::expr::CRuntimeCompiledExpression::register_function, "Register a user-defined ternary function. (New in MRPT 2.5.8) \n\nC++: mrpt::expr::CRuntimeCompiledExpression::register_function(const std::string &, const class std::function<double (double, double, double)> &) --> void", pybind11::arg("name"), pybind11::arg("func"));
		cl.def("eval", (double (mrpt::expr::CRuntimeCompiledExpression::*)() const) &mrpt::expr::CRuntimeCompiledExpression::eval, "Evaluates the current value of the precompiled formula.\n \n\n std::runtime_error If the formula has not been compiled yet.\n\nC++: mrpt::expr::CRuntimeCompiledExpression::eval() const --> double");
		cl.def("is_compiled", (bool (mrpt::expr::CRuntimeCompiledExpression::*)() const) &mrpt::expr::CRuntimeCompiledExpression::is_compiled, "Returns true if compile() was called and ended without errors. \n\nC++: mrpt::expr::CRuntimeCompiledExpression::is_compiled() const --> bool");
		cl.def("get_original_expression", (const std::string & (mrpt::expr::CRuntimeCompiledExpression::*)() const) &mrpt::expr::CRuntimeCompiledExpression::get_original_expression, "Returns the original formula passed to compile(), or an empty string if\n still not compiled. \n\nC++: mrpt::expr::CRuntimeCompiledExpression::get_original_expression() const --> const std::string &", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::expr::CRuntimeCompiledExpression & (mrpt::expr::CRuntimeCompiledExpression::*)(const class mrpt::expr::CRuntimeCompiledExpression &)) &mrpt::expr::CRuntimeCompiledExpression::operator=, "C++: mrpt::expr::CRuntimeCompiledExpression::operator=(const class mrpt::expr::CRuntimeCompiledExpression &) --> class mrpt::expr::CRuntimeCompiledExpression &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}

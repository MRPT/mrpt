\defgroup mrpt_expr_grp [mrpt-expr]

Dynamic expression analyzers for interpreting small scripts.

[TOC]

# Library mrpt-expr

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-expr-dev

See: \ref mrpt_from_cmake

## Overview

`mrpt-expr` compiles mathematical formulas given as text at run time, then
evaluates them as many times as needed, with the current values of the
variables they use. It is a thin wrapper of the
[exprtk](https://github.com/ArashPartow/exprtk) library, which keeps its long
compilation times out of user code.

Use it for user-defined formulas in configuration files: cost functions,
thresholds depending on the robot speed, etc. Within MRPT it is used, for
example, by the `$eval{...}` syntax of \ref config_file_format and by the
reactive navigation engine of \ref mrpt_nav_grp.

## Key class

mrpt::expr::CRuntimeCompiledExpression:
- `compile()`: parses a formula, given optional variables by value in a
  `std::map`; throws `std::runtime_error` on syntax errors or unknown symbols.
- `register_symbol_table()`: binds variable names to user-owned `double`s,
  before calling `compile()`, so changing them changes the result of `eval()`.
- `register_function()`: adds custom functions with 0 to 3 arguments
  (any callable, e.g. a lambda).
- `eval()`: evaluates the formula, returning a `double`.

Setting the environment variable `MRPT_EXPR_VERBOSE=1` prints the values of
all the variables involved on each call to `eval()`, for debugging.

## Example

```cpp
#include <mrpt/expr/CRuntimeCompiledExpression.h>

double x = 0;
double y = 0;

mrpt::expr::CRuntimeCompiledExpression expr;
expr.register_symbol_table({{"x", &x}, {"y", &y}});
expr.compile("sqrt(x^2 + y^2) + max(x, y)");  // throws on syntax errors

x = 3.0;
y = 4.0;
std::cout << expr.eval() << "\n";  // 9
```

## Examples

- Python: [mrpt_expr_example.py](pymrpt_example_mrpt_expr_example.html).

# Library contents

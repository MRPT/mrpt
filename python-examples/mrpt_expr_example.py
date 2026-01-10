#!/bin/env python3

import mrpt.expr


def main():
    # 1. Basic Expression Evaluation
    # ---------------------------------------------------------
    expr = mrpt.expr.CRuntimeCompiledExpression()

    # We can pass variables as a dictionary
    variables = {"x": 10.0, "y": 5.5}

    formula = "x * sin(PI / 2) + y^2"
    print(f"Compiling: {formula}")

    expr.compile(formula, variables)

    if expr.is_compiled():
        result = expr.eval()
        print(f"Result: {result}")  # Expected: 10 * 1 + 5.5^2 = 40.25

    # 2. Using Custom Python Functions in the Formula
    # ---------------------------------------------------------
    print("\nRegistering custom Python function...")

    # Define a custom Python function
    def my_custom_logic(a, b):
        # This could be any complex Python logic
        return (a + b) / 2.0

    expr_custom = mrpt.expr.CRuntimeCompiledExpression()

    # Register it! Now the parser knows 'avg(a,b)'
    expr_custom.register_function("avg", my_custom_logic)

    expr_custom.compile("avg(10, 20) + 5")
    print(f"Custom result: {expr_custom.eval()}")  # Expected: 15 + 5 = 20.0

    # 3. Dynamic Updates
    # ---------------------------------------------------------
    # Note: To update variable values in this specific C++ API,
    # you re-compile the expression with the new values.
    # Because compilation is extremely fast (μs), this is standard.
    new_vars = {"x": 20.0, "y": 2.0}
    expr.compile(formula, new_vars)
    print(f"Updated result: {expr.eval()}")


if __name__ == "__main__":
    main()

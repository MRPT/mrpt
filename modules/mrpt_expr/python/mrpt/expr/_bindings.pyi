"""
Python bindings for mrpt-expr (Runtime expression parser)
"""
from __future__ import annotations
import typing
__all__: list[str] = ['CRuntimeCompiledExpression']
class CRuntimeCompiledExpression:
    """
    A wrapper of exprtk runtime expression compiler: it takes a string representing an expression (from a simple mathematical formula to a complete program), compiles it and evaluates its result as many times as required.
    """
    def __init__(self) -> None:
        """
        Default constructor: call compile() before evaluating.
        """
    def compile(self, expression: str, variables: dict[str, float] = {}) -> None:
        """
        Compiles a string expression with optional variable and constant maps.
        """
    def eval(self) -> float:
        """
        Evaluates the compiled expression and returns the result.
        """
    def get_original_expression(self) -> str:
        """
        Returns the original formula string.
        """
    def is_compiled(self) -> bool:
        """
        Returns true if the expression has been successfully compiled.
        """
    def register_function(self, name: str, func: typing.Callable) -> None:
        """
        Registers a Python function taking 0 to 3 float arguments, for use in expressions compiled afterwards. The number of arguments is taken from its signature.
        """

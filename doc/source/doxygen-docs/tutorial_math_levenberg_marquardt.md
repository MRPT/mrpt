\page tutorial_math_levenberg_marquardt Levenberg-Marquardt algorithm with numeric Jacobians

# 1. Examples

If you prefer directly jumping to example code, see:

- \ref math_optimize_lm_example

# 2.Introduction

This page first describes the Levenberg-Marquardt optimization algorithm, then shows how to use its implementation within the `mrpt-math` C++ library. All the source code discussed here, the implementation of the algorithm itself, and the examples, are available for download within the MRPT packages.

The following notation and algorithm have been extracted from the report \cite madsen2004methods

# 3. Algorithm description

The Levenberg-Marquardt (LM) method consists on an iterative least-square minimization of a cost function based on a modification of the Gauss-Newton method. Let’s state the problem formally before defining the algorithm. We will assume that derivatives of the cost functions are not available in closed form, so they will be approximated by finite-difference approximation [Finite-difference approximation](http://en.wikipedia.org/wiki/Finite_difference).

Let \f$ \mathbf{x} \in \mathcal{R}^n \f$ be the parameter vector to be optimized. We want to find the optimal \f$\mathbf{x}^\star\f$ that minimizes the scalar error function \f$F(\cdot)\f$:

\f[
\mathbf{x}^\star = \arg \min_x F(\mathbf{x})
\f]

with:
\f[
F(\mathbf{x}) = \frac{1}{2} || \mathbf{f}(\mathbf{x}) ||^2 = \frac{1}{2} \mathbf{f}^T(\mathbf{x}) \mathbf{f}(\mathbf{x}) \f]

The function \f$ \mathbf{f}: \mathcal{R}^n \to \mathcal{R}^m  \f$ may sometimes
include a comparison to some reference, or observed, data.
A very simple, linear example would be \f$ \mathbf{f}(\mathbf{x}) = \mathbf{b} - \mathbf{A}\mathbf{x} \f$.
However in the following we assume \f$ \mathbf{f}(\cdot)\f$ can have any form:

\f[
f(\mathbf{x}) = \left( f_1(\mathbf{x}) ~~ \ldots ~~ f_m(\mathbf{x}) \right) ^T
\f]

We define the Jacobian of the error functions as the \f$ m \times n \f$ matrix:

\f[
 \mathbf{J}_{ij}(\mathbf{x}) = \frac{\partial f_i}{\partial x_j}, ~~~~ i=1,…,m ~~~~ j=1,…,n
\f]

The Hessian of the error function is the \f$ n \times n \f$ matrix of second
order derivatives (\f$ n \f$ being the length of the parameter vector),
and it’s approximated by:

\f[
 \mathbf{H}(x) = \mathbf{J}(x)^T \mathbf{J}(x)
\f]

If we do not have closed form expressions for the derivatives needed for the Jacobian,
we can estimate them from finite differences using some increments for
each individual variable \f$ \Delta x_j \f$:

\f[
 \mathbf{J}_{ij}(\mathbf{x}) \simeq \frac{ f_i(\mathbf{x}+\mathbf{\Delta x}) - f_i(\mathbf{x}-\mathbf{\Delta x}) }{2\Delta x_j}
\f]

Then, the LM method minimizes the following linear approximation of the actual error function:

\f[
 \mathbf{F}(\mathbf{x}+\mathbf{h}) \simeq \mathbf{L}(\mathbf{h}) = \mathbf{F}(\mathbf{x}) + \mathbf{h} ~ \mathbf{g}(x) + \frac{1}{2} \mathbf{h}^T \mathbf{H}(x) \mathbf{h}
\f]

where the gradient \f$ \mathbf{g}(x) \f$ is given by \f$  \mathbf{J}(x)^T \mathbf{f}(x) \f$.

Now, denote as \f$  \mathbf{x}^\star_{t}\f$  for \f$ t=0,1,2,...\f$  the sequence
of iterative approximations to the optimal set of parameters
\f$ \mathbf{x}^\star\f$.
The first initial guess \f$ \mathbf{x}^\star_0 \f$  must be provided by the user.
Then, each iteration of the LM method performs:

\f[
 \mathbf{x}^\star_{t+1} = \mathbf{x}^\star_{t} + \mathbf{h}_{lm}
\f]

where each step is obtained from:

\f[
 \mathbf{h}_{lm} = - ( \mathbf{H}(x) + \lambda \mathbf{I} )^{-1} \mathbf{g}(x)
\f]

The damping factor \f$ \lambda \f$ is adapted dynamically according to a
heuristic rule, as shown in the next list of the whole algorithm.
Basically, it iterates until a maximum number of iterations is reached or
the change in the parameter vector is very small.


# 4. C++ Implementation

The LM algorithm is implemented in the C++ template class
[mrpt::math::CLevenbergMarquardtTempl<T>](class_mrpt_math_CLevenbergMarquardtTempl.html),
and there is an example in MRPT/samples/optimize-lm, which is described next.

The type [mrpt::math::CLevenbergMarquard](namespace_mrpt_math.html#doxid-namespacemrpt-1-1math-1ae54dd61d03206aef14cfbea53165d239)
is actually a shortcut for the template instantiation `mrpt::math::CLevenbergMarquardtTempl<double>`.

The image below represents the resulting path from the initial guess to the minimum for this example.
The displayed equation is the one-dimensional error (cost) function,
\f$ \mathbf{f}(\mathbf{x})\f$:

![Levenberg Marquard C++ example output](Optimize-lm-example-func.png)

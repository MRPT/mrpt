# Contributing to MRPT

Thanks for your interest in improving the Mobile Robot Programming Toolkit (MRPT)!

## Reporting bugs

Before opening a new issue:

* Search the [issue tracker](https://github.com/MRPT/mrpt/issues) (open and closed) to make sure it hasn't been reported already.
* Make sure you can reproduce the problem with the latest `develop` branch, since it may already be fixed.

When filing a bug report, please include:

* MRPT version (or commit hash) and how you installed/built it (apt package, source build, ROS, etc.).
* OS, compiler, and architecture.
* A minimal, self-contained reproduction (code snippet, dataset, or config file).
* The exact error message / stack trace / unexpected output.

## Proposing features or larger changes

For anything beyond a small fix, please open an issue or a
[discussion](https://github.com/MRPT/mrpt/discussions) first to agree on the
approach before investing time in a pull request. This avoids duplicated
effort and PRs that don't align with the project's direction.

## Development setup

MRPT 3.x is organized as a set of modular [colcon](https://colcon.readthedocs.io/)
packages, one per `modules/mrpt_*` directory. To build:

```bash
colcon build --packages-up-to mrpt_XXX
. install/setup.bash
```

A `colcon_defaults.yaml` at the repo root already configures symlink installs,
RelWithDebInfo builds, and `CMAKE_EXPORT_COMPILE_COMMANDS`.

For the full architectural overview (module layout, CMake conventions,
pybind11 binding structure, etc.), see [`agents.md`](../agents.md). This
document is the canonical reference for both human and AI-assisted
contributions.

## Coding style

* Follow the [MRPT C++ coding style](https://github.com/MRPT/mrpt/blob/develop/doc/source/MRPT_Coding_Style.rst).
* Code must conform to the repository `.clang-format` and `.clang-tidy`
  configurations. CI runs a [clang-format check](workflows/check-clang-format.yml)
  on every PR — please run `clang-format` locally before pushing.
* New `.cpp`/`.h`/`CMakeLists.txt` files must start with the standard MRPT
  SPDX license header (see `agents.md` for the exact template).
* Use modern C++ (C++17/C++20), `std::shared_ptr`/`std::unique_ptr` for
  ownership, and `[[nodiscard]]` where it adds value.

## Tests

* Unit tests live in a `tests/` subdirectory of each module and must be named
  `*_unittest.cpp` to be picked up automatically.
* New features and bug fixes should come with a regression test where
  practical.

## Branching and pull requests

* Base your branch on `develop` (the default branch), not `master`.
* Use a descriptive branch name (e.g. `fix/icp-nan-on-empty-cloud`).
* Keep PRs focused and reasonably small — several small PRs are easier to
  review than one large one.
* Write clear commit messages explaining *why*, not just *what*.
* Update [`doc/source/doxygen-docs/changelog.md`](https://github.com/MRPT/mrpt/blob/develop/doc/source/doxygen-docs/changelog.md)
  for any user-visible change.
* Add yourself to [`AUTHORS`](https://github.com/MRPT/mrpt/blob/develop/AUTHORS)
  if this is your first contribution.
* Make sure CI (Linux/macOS/Windows builds and the clang-format check) passes
  on your PR.

## AI-assisted contributions

If you used an AI coding agent (e.g. Claude, Copilot, Codex) to generate or
refactor part of your contribution, make sure it followed the guidelines in
[`agents.md`](../agents.md). You remain responsible for reviewing,
understanding, and testing the resulting diff — please do not submit
unreviewed AI-generated code.

## Getting help

* [GitHub Discussions](https://github.com/MRPT/mrpt/discussions) for
  questions and design discussions.
* [Issue tracker](https://github.com/MRPT/mrpt/issues) for bugs and feature
  requests.

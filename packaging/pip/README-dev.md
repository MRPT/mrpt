# MRPT Python wheel: developer notes

`pyproject.toml` (scikit-build-core) runs the superbuild in `CMakeLists.txt`,
which builds every MRPT module listed in `MRPT_WHEEL_MODULES` (plus their
`mrpt_*` dependencies from `package.xml`) as an ExternalProject, in dependency
order. Everything goes into a single wheel:

```
site-packages/
  mrpt/<module>/     Python package: __init__.py, .pyi stubs, _bindings.*.so
  mrpt/sdk/          C++ SDK: include/, lib/libmrpt_*.so, lib/mrpt_*/cmake/,
                     share/mrpt_common/ (CMake functions) and __init__.py
  mrpt3.libs/        third-party libs bundled by auditwheel (Linux)
```

* Shared libraries are built without SOVERSION (`MRPT_NO_SOVERSION=ON`), since
  wheels cannot hold symlinks. RPATHs (`$ORIGIN`, `$ORIGIN/../sdk/lib`) make the
  tree relocatable.
* `mrpt/sdk` is a `cmake.prefix` entry point: other scikit-build-core projects
  find the MRPT CMake packages with no extra setup. To share Python types with
  these bindings, they must build with the same pybind11 version
  (`mrpt.sdk.PYBIND11_VERSION`).
* The version comes from `modules/mrpt_common/package.xml`.

## Local build (system libraries, quick check)

```bash
python3 -m venv /tmp/venv && /tmp/venv/bin/pip install build
/tmp/venv/bin/python -m build --wheel --outdir /tmp/dist packaging/pip
/tmp/venv/bin/pip install /tmp/dist/mrpt3-*.whl
/tmp/venv/bin/python packaging/pip/tests/test_wheel.py
```

The build dir (`packaging/pip/build/<wheel tag>`) is kept, so rebuilds are
incremental. If ccache is installed, prefix the build command with
`CMAKE_ARGS="-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache"`
to also speed up builds from scratch. Wheels built this way link to the system libraries and are only
for local use.

## Portable wheels (manylinux)

Built in CI by `.github/workflows/python-wheels.yml`. To reproduce locally
(needs Docker), from the repository root:

```bash
pip install cibuildwheel
CIBW_BUILD=cp312-manylinux_x86_64 cibuildwheel --platform linux packaging/pip
```

System dependencies for the manylinux image are in `ci/manylinux_deps.sh`;
the remaining ones (assimp, octomap, tinyxml2, nanoflann...) use the copies
embedded in MRPT. auditwheel bundles the third-party shared libraries into
`mrpt3.libs/`, except the OpenGL ones (libglvnd), which must come from the user
system to load its GPU driver.

`tests/test_wheel.py` (run by cibuildwheel) imports every module, runs the
Python binding tests of all modules, and builds a C++ program against the SDK.

## CI and publishing

The "Python wheels" workflow builds and tests one Python version per platform
on pull requests that touch the wheel files, and all of them weekly and on
release tags. Pushing a release tag (`X.Y.Z`) uploads its wheels to PyPI; a
manual run can also upload to `testpypi` or `pypi` (`publish` input). It uses
PyPI Trusted Publishing, set up once per index for this repository, the
`python-wheels.yml` workflow and an environment of the same name
(`testpypi` / `pypi`). PyPI never accepts the same version twice: test upload
changes on TestPyPI first.

## oneTBB

The superbuild also builds oneTBB (`ep_tbb`) into `mrpt/sdk`, since MRPT
public headers use it. Downstream wheels must link to that copy, not to
another one.

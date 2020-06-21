Root directory for MRPT Sphinx documentation.

# Requisites

```
# Install Sphinx & dependencies:
pip install sphinx_rtd_theme
```

Install doxyrest: [instructions](https://github.com/vovkos/doxyrest_b/blob/master/README.rst).

```
# Dependencies for doxyrest_b
sudo apt install liblua5.2-dev libexpat-dev ragel

echo "set (EXPAT_LIB_DIR /usr/lib/x86_64-linux-gnu)" >> paths.cmake
echo "set (LUA_LIB_DIR /usr/lib/x86_64-linux-gnu)" >> paths.cmake
echo "set (OPENSSL_INC_DIR DISABLED)" >> paths.cmake
echo "set (AXL_CMAKE_DIR $(pwd)/axl/cmake $(pwd)/axl/build/cmake)" >> paths.cmake

```

# How to generate docs

From the MRPT source tree:

```
cd docs
make
```

You may need to either modify PATH and PYTHONPATH in your env, or edit the hard-coded paths in mrpt/docs/Makefile

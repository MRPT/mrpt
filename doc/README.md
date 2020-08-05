Root directory for MRPT Sphinx documentation.

# Dependencies

```
# Install virtualenv:
sudo apt install virtualenv
```

Install doxyrest: [instructions](https://github.com/vovkos/doxyrest_b/blob/master/README.rst).

```
# Dependencies for doxyrest_b
sudo apt install liblua5.2-dev libexpat-dev ragel
```

# How to generate docs

From the MRPT source tree:

```
cd docs
make
```

It will create the Python virtualenv for you on its first call. 

You may need to either modify PATH and PYTHONPATH in your env, or edit the hard-coded paths in mrpt/docs/Makefile

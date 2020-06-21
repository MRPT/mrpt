
.DEFAULT_GOAL := all

# Replace this path with your local build of doxyrest:
export PYTHONPATH := $(HOME)/code/doxyrest_b/doxyrest/sphinx/:$(PYTHONPATH)
export PATH := $(HOME)/code/doxyrest_b/build/doxyrest/bin/Release:$(PATH)
export MRPT_VERSION_STR := $(shell head -n1 ../version_prefix.txt)
export MRPT_LIST_INCLUDE_DIRS := $(shell for f in ../libs/*/include; do printf "$$(realpath $$f) "; done)
export MRPT_DOX_INPUT_DIRS := $(shell for f in ../libs/*; do printf "$$(realpath $$f) "; done)

# Use to quickly test the documentation scripts:
ifeq ($(MRPT_DOCS_FAST_TEST),1)
export MRPT_DOX_INPUT_DIRS := $(shell realpath ../libs/core)
endif


.PHONY: clean
clean:
	rm -fr html
	rm -fr \
		source/xml-dir \
		source/class_* \
		source/enum_* \
		source/namespace_* \
		source/group_* \
		source/struct_* \
		source/page_* \
		source/union_* \
		source/global.rst source/doxygen-index.rst \
		2> /dev/null || true

.PHONY: all
all:
	make html

html-and-view:
	make html
	xdg-open html/index.html

html:
	echo "Building docs for these directories: ${MRPT_DOX_INPUT_DIRS}"
	echo "Using MRPT_LIST_INCLUDE_DIRS: ${MRPT_LIST_INCLUDE_DIRS}"
	# 1) Build Doxygen
	cd source && doxygen
	# 2) Doxygen XML -> RST
	cd source && doxyrest -c doxyrest-config.lua
	# Remove main doc file created for nanoflann:
	rm source/page_index.rst 2>/dev/null || true
	# 3) RST -> HTML with Sphinx
	sphinx-build -b html source/ html/

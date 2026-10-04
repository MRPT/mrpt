
.DEFAULT_GOAL := all

# Path to your local build of doxyrest (override from the command line):
DOXYREST_DIR ?= $(HOME)/code/doxyrest_b
export PYTHONPATH := $(DOXYREST_DIR)/doxyrest/sphinx/:$(PYTHONPATH)
export PATH := $(DOXYREST_DIR)/build/doxyrest/bin/Release:$(PATH)
export MRPT_VERSION_STR := $(shell sed -n 's:.*<version>\([0-9.]*\)</version>.*:\1:p' ../modules/mrpt_common/package.xml | head -n1)
export MRPT_LIST_INCLUDE_DIRS := $(shell for f in ../modules/*/include; do printf "$$(realpath $$f) "; done)
export MRPT_DOX_INPUT_DIRS := $(shell for f in ../modules/*; do printf "$$(realpath $$f) "; done)

# Use to quickly test the documentation scripts:
ifeq ($(MRPT_DOCS_FAST_TEST),1)
export MRPT_DOX_INPUT_DIRS := $(shell realpath ../modules/mrpt_gui)
endif


.PHONY: clean
clean:
	rm -fr html markdown
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
		source/examples.rst source/doxygen-docs/example-*.md \
		source/python_examples.rst source/pymrpt_example_*.rst source/_extra \
		source/_python_api_src source/python_api \
		2> /dev/null || true

.PHONY: all
all:
	make html

python-venv: requirements.txt
	# Note: the DEB_PYTHON_INSTALL_LAYOUT=deb below 
	# is required in u22.04 to fix a bug in virtualenv.
	# That env var can be removed once virtualenv > 20.16.5 is everywhere.
	# Read: https://askubuntu.com/a/1415153/97996
	
	# Create virtual env for building the docs
	DEB_PYTHON_INSTALL_LAYOUT=deb virtualenv -p /usr/bin/python3 venv
	bash -c "source venv/bin/activate && pip install -r requirements.txt"

html-and-view:
	make html
	xdg-open html/index.html

html: python-venv
	echo "Building docs for these directories: ${MRPT_DOX_INPUT_DIRS}"
	echo "Using MRPT_LIST_INCLUDE_DIRS: ${MRPT_LIST_INCLUDE_DIRS}"
	# 0) Generate the pages of C++ and Python examples:
	python3 ../scripts/generate_rst_docs_examples.py
	# 1) Build Doxygen
	cd source && doxygen
	# 2) Doxygen XML -> RST
	cd source && bash -c "source ../venv/bin/activate && doxyrest -c doxyrest-config.lua"
	# Remove main doc file created for nanoflann:
	rm source/page_index.rst 2>/dev/null || true
	# llms.txt and llms-full.txt (uses the Doxygen XML output):
	python3 ../scripts/generate_llms_txt.py
	# 3) RST -> HTML with Sphinx
	bash -c "source venv/bin/activate && sphinx-build -b html source/ html/"
	# 3b) Markdown version of each page (for AI agents), next to its HTML:
	bash -c "source venv/bin/activate && sphinx-build -b markdown source/ markdown/"
	cp -r markdown/. html/
	# 4) Copy images:
	# Doxygen has this limitation, images cannot be parsed into XML/HTML with
	# a relative path, so we must copy them to the root directory.
	cp source/images/* html/
	# Copy remote images also:
	bash remote-static-files.sh

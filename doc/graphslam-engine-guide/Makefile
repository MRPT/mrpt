# From: http://tex.stackexchange.com/questions/40738/how-to-properly-make-a-latex-project
# You want latexmk to *always* run, because make does not have all the info.
.PHONY: graphslam-engine-guide.pdf graphslam-engine-guide.ps.gz

# First rule should always be the default "all" rule, so both "make all" and
# "make" will invoke it.
all: graphslam-engine-guide.pdf

# MAIN LATEXMK RULE

# -pdf tells latexmk to generate PDF directly (instead of DVI).
# -pdflatex="" tells latexmk to call a specific backend with specific options.
# -use-make tells latexmk to call make for generating missing files.

# -interactive=nonstopmode keeps the pdflatex backend from stopping at a
# missing file reference and interactively asking you for an alternative.

# MRPT_VERSION_STR may already be exported by the caller (doc/Makefile,
# packaging/make_release.sh); fall back to parsing package.xml directly so
# this guide can also be built standalone:
MRPT_VERSION_STR ?= $(shell sed -n 's:.*<version>\([0-9.]*\)</version>.*:\1:p' ../../modules/mrpt_common/package.xml | head -n1)

tex_includes/version.tex: FORCE
	printf '%s\n' '\newcommand{\MRPTVERSION}{$(MRPT_VERSION_STR)}' > $@

FORCE:

graphslam-engine-guide.pdf: graphslam-engine-guide.tex tex_includes/version.tex
	latexmk -pdf -pdflatex="pdflatex -interactive=nonstopmode" -bibtex -use-make graphslam-engine-guide.tex

graphslam-engine-guide.ps.gz: graphslam-engine-guide.pdf
	pdf2ps graphslam-engine-guide.pdf
	gzip -f graphslam-engine-guide.ps

clean:
	latexmk -CA
	rm *.bbl *.nlo


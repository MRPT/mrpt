# From: http://tex.stackexchange.com/questions/40738/how-to-properly-make-a-latex-project
# You want latexmk to *always* run, because make does not have all the info.
.PHONY: srba-guide.pdf

# First rule should always be the default "all" rule, so both "make all" and
# "make" will invoke it.
all: srba-guide.pdf

# MAIN LATEXMK RULE

# -pdf tells latexmk to generate PDF directly (instead of DVI).
# -pdflatex="" tells latexmk to call a specific backend with specific options.
# -use-make tells latexmk to call make for generating missing files.

# -interactive=nonstopmode keeps the pdflatex backend from stopping at a
# missing file reference and interactively asking you for an alternative.

srba-guide.pdf: srba-guide.tex
	latexmk -pdf -pdflatex="pdflatex -interactive=nonstopmode" -use-make srba-guide.tex

clean:
	latexmk -CA


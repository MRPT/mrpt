PROJECT(doc_srba_guide_prj)

INCLUDE(UseLATEX.cmake)

add_custom_target(doc_srba_guide)

IF (LATEX_FOUND)
	ADD_LATEX_DOCUMENT(
		srba-guide.tex
		BIBFILES cites.bib
		)

	add_dependencies(doc_srba_guide pdf)
ENDIF(LATEX_FOUND)


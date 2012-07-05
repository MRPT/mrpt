# Example of usage: 
#  REMOVE_MATCHING_FILES_FROM_LIST(".*_LIN.cpp" my_srcs)
#

MACRO(REMOVE_MATCHING_FILES_FROM_LIST match_expr lst_files)
	SET(lst_files_aux "")
	FOREACH(FIL ${${lst_files}})
		IF(NOT ${FIL} MATCHES "${match_expr}")
			SET(lst_files_aux "${lst_files_aux}" "${FIL}")
		ENDIF(NOT ${FIL} MATCHES "${match_expr}")
	ENDFOREACH(FIL)
	SET(${lst_files} ${lst_files_aux})
ENDMACRO(REMOVE_MATCHING_FILES_FROM_LIST)

MACRO(KEEP_MATCHING_FILES_FROM_LIST match_expr lst_files)
	SET(lst_files_aux "")
	FOREACH(FIL ${${lst_files}})
		IF(${FIL} MATCHES "${match_expr}")
			SET(lst_files_aux "${lst_files_aux}" "${FIL}")
		ENDIF(${FIL} MATCHES "${match_expr}")
	ENDFOREACH(FIL)
	SET(${lst_files} ${lst_files_aux})
ENDMACRO(KEEP_MATCHING_FILES_FROM_LIST)

# Example of usage: 
#  REMOVE_MATCHING_FILES_FROM_LIST(".*_LIN.cpp" my_srcs)
#

macro(REMOVE_MATCHING_FILES_FROM_LIST match_expr lst_files)
	set(lst_files_aux "")
	foreach(FIL ${${lst_files}})
		if(NOT ${FIL} MATCHES "${match_expr}")
			set(lst_files_aux "${lst_files_aux}" "${FIL}")
		endif()
	endforeach(FIL)
	set(${lst_files} ${lst_files_aux})
endmacro()

macro(KEEP_MATCHING_FILES_FROM_LIST match_expr lst_files)
	set(lst_files_aux "")
	foreach(FIL ${${lst_files}})
		if(${FIL} MATCHES "${match_expr}")
			set(lst_files_aux "${lst_files_aux}" "${FIL}")
		endif()
	endforeach(FIL)
	set(${lst_files} ${lst_files_aux})
endmacro()

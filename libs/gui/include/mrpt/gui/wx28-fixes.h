#pragma once

// Support building in old wx2.8.0 without changing my code:
#if !wxCHECK_VERSION(2, 9, 0)
#include <wx/font.h>
#	define wxFONTWEIGHT_BOLD wxBOLD
#	define wxFONTFAMILY_TELETYPE wxTELETYPE
#endif


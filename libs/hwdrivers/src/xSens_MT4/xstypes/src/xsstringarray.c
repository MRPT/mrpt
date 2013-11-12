#include "xsstringarray.h"
#include "xsstring.h"

/*! \struct XsStringArray
	\brief A list of XsString values
	\sa XsArray
*/

//! \brief Descriptor for XsStringArray
XsArrayDescriptor const g_xsStringArrayDescriptor = {
	//lint --e{64} ignore exact type mismatches here
	sizeof(XsString),
	XSEXPCASTITEMSWAP XsArray_swap,
	XSEXPCASTITEMMAKE XsString_construct,
	XSEXPCASTITEMCOPY XsArray_copyConstruct,
	XSEXPCASTITEMMAKE XsArray_destruct,
	XSEXPCASTITEMCOPY XsArray_copy,
	XSEXPCASTITEMCOMP XsArray_compare
};

/*! \copydoc XsArray_construct
	\note Specialization for XsStringArray
*/
void XsStringArray_construct(XsStringArray* thisPtr, XsSize count, XsString const* src)
{
	XsArray_construct(thisPtr, &g_xsStringArrayDescriptor, count, src);
}

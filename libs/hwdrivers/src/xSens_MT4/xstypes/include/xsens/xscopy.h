#ifndef XSCOPY_H
#define XSCOPY_H

#define XSLISTCOPY(C)	\
	if (copy == thisPtr)\
	{\
		*((int*) &copy->m_flags) &= ~XSDF_DestructiveCopy;\
		return;\
	}\
	if ((thisPtr->m_flags & (XSDF_DestructiveCopy | XSDF_Managed)) == (XSDF_DestructiveCopy | XSDF_Managed))\
	{\
		*((int*) &thisPtr->m_flags) &= ~XSDF_DestructiveCopy;\
		if ((copy->m_flags & XSDF_Managed) || (copy->m_data == 0))\
		{\
			C##_swap((C*) thisPtr, copy); \
			return;\
		}\
	}\
	C##_assign(copy, thisPtr->m_size, thisPtr->m_data);

#define XSLISTSWAP3(C, B, S)	\
	if ((!a->m_data || (a->m_flags & XSDF_Managed)) && (!b->m_data || (b->m_flags & XSDF_Managed))) {\
		B tmp;\
		*((C**) &tmp.m_data) = a->m_data;\
		*((XsSize*) &tmp.m_size) = a->m_size;\
		*((int*) &tmp.m_flags) = a->m_flags;\
		*((C**) &a->m_data) = b->m_data;\
		*((XsSize*) &a->m_size) = b->m_size;\
		*((int*) &a->m_flags) = b->m_flags;\
		*((C**) &b->m_data) = tmp.m_data;\
		*((XsSize*) &b->m_size) = tmp.m_size;\
		*((int*) &b->m_flags) = tmp.m_flags;\
	} else {	/* elementwise swap */ \
		XsSize i;\
		assert(a->m_size == b->m_size);\
		for (i = 0; i < a->m_size; ++i) S(&a->m_data[i], &b->m_data[i]);\
	}

#define XSLISTSWAP2(C, B)	XSLISTSWAP3(C, B, C##_swap)

#define XSLISTSWAP(C)	XSLISTSWAP2(C, C##List)

#endif // file guard

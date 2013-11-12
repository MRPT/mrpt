#include "xsoutputconfiguration.h"

void XsOutputConfiguration_swap(struct XsOutputConfiguration* a, struct XsOutputConfiguration* b)
{
	{
		XsDataIdentifier t = a->m_dataIdentifier;
		a->m_dataIdentifier = b->m_dataIdentifier;
		b->m_dataIdentifier = t;
	}
	
	{
		uint16_t t = a->m_frequency;
		a->m_frequency = b->m_frequency;
		b->m_frequency = t;
	}
}

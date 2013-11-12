#ifndef INT_XSDATAPACKET_H
#define INT_XSDATAPACKET_H

struct XsDataPacket;
struct LegacyDataPacket;

#ifdef __cplusplus
extern "C" 
{
#endif

void XsDataPacket_assignFromXsLegacyDataPacket(struct XsDataPacket* thisPtr, struct LegacyDataPacket const* pack, int index);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // file guard

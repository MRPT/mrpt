#ifndef __EXAMPLE_LINUX_H
#define __EXAMPLE_LINUX_H

int doHardwareScan(xsens::Cmt3 &, CmtDeviceId []);
void doMtSettings(xsens::Cmt3 &, CmtOutputMode &, CmtOutputSettings &, CmtDeviceId []);
void getUserInputs(CmtOutputMode &, CmtOutputSettings &);
void writeHeaders(unsigned long, CmtOutputMode &, CmtOutputSettings &, int&, int&);
int calcScreenOffset(CmtOutputMode &, CmtOutputSettings &, int);

#endif

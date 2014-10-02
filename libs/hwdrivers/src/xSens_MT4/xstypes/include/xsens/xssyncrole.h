#ifndef XSSYNCROLE_H
#define XSSYNCROLE_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Synchronization roles
*/
enum XsSyncRole {
	XSR_Slave,
	XSR_None,
	XSR_MasterSlave,
	XSR_Master
};
/*! @} */
typedef enum XsSyncRole XsSyncRole;

#endif // file guard

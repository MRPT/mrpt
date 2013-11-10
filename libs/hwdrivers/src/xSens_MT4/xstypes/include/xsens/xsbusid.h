#ifndef XSBUSID_H
#define XSBUSID_H

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \brief The bus identifier of the master device */
#define XS_BID_MASTER         0xFF

/*! \brief The bus broadcast bus identifier (all devices) */
#define XS_BID_BROADCAST      0x00

/*! \brief The bus identifier for the first MT on the bus */
#define XS_BID_MT             0x01

/*! \brief An invalid bus identifier */
#define XS_BID_INVALID        0xFE

/*! @} */

#endif // file guard

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSDID_H
#define XSDID_H

// DID Type (high nibble)
#define XS_DID_TYPEL_MASK               0x000F0000
#define XS_DID_TYPEH_MASK               0x00F00000
#define XS_DID_GPL_MASK					0x0F000000
#define XS_DID_GPH_MASK					0xF0000000
#define XS_DID_TYPE_MASK                (XS_DID_TYPEH_MASK | XS_DID_TYPEL_MASK)
#define XS_DID_MK4TYPE_MASK             (XS_DID_TYPEH_MASK | XS_DID_GPL_MASK)

#define XS_DID_TYPEL_SHIFT				16
#define XS_DID_TYPEH_SHIFT				20
#define XS_DID_GPL_SHIFT				24
#define XS_DID_GPH_SHIFT				28

#define XS_DID_TYPEH_XM                 0x00100000
#define XS_DID_TYPEH_AWINDAMASTER       0x00200000
#define XS_DID_TYPEH_MTI_MTX            0x00300000
#define XS_DID_TYPEH_MTIG               0x00500000
#define XS_DID_TYPEH_MT_X0			    0x00600000
#define XS_DID_TYPEH_MT_X00				0x00700000
#define XS_DID_TYPEH_MTX2_MTW2			0x00800000

#define XS_DID_TYPEL_STATION			0x00000000
#define XS_DID_TYPEL_DONGLE			    0x00010000
#define XS_DID_TYPEL_OEM			    0x00020000

#define XS_DID_TYPEL_RS232				0x00000000
#define XS_DID_TYPEL_RS422				0x00010000
#define XS_DID_TYPEL_RS485XM			0x00020000
#define XS_DID_TYPEL_RS485				0x00030000
#define XS_DID_TYPEL_WIRELESS			0x00040000

#define XS_DID_GPL_10					0x01000000
#define XS_DID_GPL_20					0x02000000
#define XS_DID_GPL_30					0x03000000
#define XS_DID_GPL_100					0x01000000
#define XS_DID_GPL_200					0x02000000
#define XS_DID_GPL_300					0x03000000
#define XS_DID_GPL_400					0x04000000
#define XS_DID_GPL_500					0x05000000
#define XS_DID_GPL_600					0x06000000
#define XS_DID_GPL_700					0x07000000
#define XS_DID_GPL_800					0x08000000
#define XS_DID_GPL_900					0x09000000
#define XS_DID_GPL_IMU					0x01000000
#define XS_DID_GPL_VRU					0x02000000
#define XS_DID_GPL_AHRS					0x03000000
#define XS_DID_GPL_AHRSGPS				XS_DID_GPL_400
#define XS_DID_GPL_AHRSGPSG				XS_DID_GPL_500
#define XS_DID_GPL_GPSINS				XS_DID_GPL_600
#define XS_DID_GPL_GPSINSG				XS_DID_GPL_700

#define XS_DID_TYPE_AWINDA_STATION		(XS_DID_TYPEH_AWINDAMASTER | XS_DID_TYPEL_STATION)
#define XS_DID_TYPE_AWINDA_DONGLE		(XS_DID_TYPEH_AWINDAMASTER | XS_DID_TYPEL_DONGLE)
#define XS_DID_TYPE_AWINDA_OEM			(XS_DID_TYPEH_AWINDAMASTER | XS_DID_TYPEL_OEM)

#define XS_DID_TYPE_MTW					(XS_DID_TYPEH_MTI_MTX | XS_DID_TYPEL_WIRELESS)	
#define XS_DID_TYPE_MTIX_RS232			(XS_DID_TYPEH_MTI_MTX | XS_DID_TYPEL_RS232)
#define XS_DID_TYPE_MTIX_RS422			(XS_DID_TYPEH_MTI_MTX | XS_DID_TYPEL_RS422)
#define XS_DID_TYPE_MTIX_RS485			(XS_DID_TYPEH_MTI_MTX | XS_DID_TYPEL_RS485)
#define XS_DID_TYPE_MTX_XBUS			(XS_DID_TYPEH_MTI_MTX | XS_DID_TYPEL_RS485XM)
#define XS_DID_TYPE_MTW2				(XS_DID_TYPEH_MTX2_MTW2 | XS_DID_TYPEL_WIRELESS)

#define XS_DID_MK4TYPE_MT_10			(XS_DID_TYPEH_MT_X0 | XS_DID_GPL_10)
#define XS_DID_MK4TYPE_MT_20			(XS_DID_TYPEH_MT_X0 | XS_DID_GPL_20)
#define XS_DID_MK4TYPE_MT_30			(XS_DID_TYPEH_MT_X0 | XS_DID_GPL_30)
#define XS_DID_MK4TYPE_MT_100			(XS_DID_TYPEH_MT_X00 | XS_DID_GPL_100)
#define XS_DID_MK4TYPE_MT_200			(XS_DID_TYPEH_MT_X00 | XS_DID_GPL_200)
#define XS_DID_MK4TYPE_MT_300			(XS_DID_TYPEH_MT_X00 | XS_DID_GPL_300)
#define XS_DID_MK4TYPE_MT_400			(XS_DID_TYPEH_MT_X00 | XS_DID_GPL_400)
#define XS_DID_MK4TYPE_MT_500			(XS_DID_TYPEH_MT_X00 | XS_DID_GPL_500)
#define XS_DID_MK4TYPE_MT_600			(XS_DID_TYPEH_MT_X00 | XS_DID_GPL_600)
#define XS_DID_MK4TYPE_MT_700			(XS_DID_TYPEH_MT_X00 | XS_DID_GPL_700)
#define XS_DID_MK4TYPE_MT_800			(XS_DID_TYPEH_MT_X00 | XS_DID_GPL_800)
#define XS_DID_MK4TYPE_MT_900			(XS_DID_TYPEH_MT_X00 | XS_DID_GPL_900)

#define XS_DID_BROADCAST                0x80000000
#define XS_DID_MASTER					0x00000000

#define XS_DID_XM(did)					((did & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_XM)
#define XS_DID_NOXM(did)				((did & XS_DID_TYPEH_MASK) != XS_DID_TYPEH_XM)
#define XS_DID_WM(did)					((did & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_AWINDAMASTER)
#define XS_DID_NOWM(did)				((did & XS_DID_TYPEH_MASK) != XS_DID_TYPEH_AWINDAMASTER)
#define XS_DID_WMMT(did)				((did & XS_DID_TYPE_MASK) == XS_DID_TYPE_MTW)
#define XS_DID_NOWMMT(did)				((did & XS_DID_TYPE_MASK) != XS_DID_TYPE_MTW)
#define XS_DID_XMMT(did)				((did & XS_DID_TYPE_MASK) == XS_DID_TYPE_MASK_MT_XM)
#define XS_DID_NOXMMT(did)				((did & XS_DID_TYPE_MASK) != XS_DID_TYPE_MASK_MT_XM)
#define XS_DID_MTIX(did)				(((did & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MTI_MTX) && XS_DID_NOWMMT(did))
#define XS_DID_MTIG(did)				(((did & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MTIG) || ((did & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_700))
#define XS_DID_MTW2(did)				((did & XS_DID_TYPE_MASK) == XS_DID_TYPE_MTW2)

#define XS_DID_AWINDA_STATION(did)		((did & XS_DID_TYPE_MASK) == XS_DID_TYPE_AWINDA_STATION)
#define XS_DID_AWINDA_DONGLE(did)		((did & XS_DID_TYPE_MASK) == XS_DID_TYPE_AWINDA_DONGLE)
#define XS_DID_AWINDA_OEM(did)			((did & XS_DID_TYPE_MASK) == XS_DID_TYPE_AWINDA_OEM)

#endif	// file guard

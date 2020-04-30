
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef DATAPACKET_P_H
#define DATAPACKET_P_H

#include "xsmessage.h"
#include "xsdeviceid.h"
#include "xstimestamp.h"
#include <map>
#include <atomic>
#include "xsquaternion.h"
#include "xsushortvector.h"
#include "xsvector3.h"
#include "xsscrdata.h"
#include "xstriggerindicationdata.h"
#include "xseuler.h"
#include "xsmatrix3x3.h"
#include "xsrange.h"
#include "xstimeinfo.h"
#include "xsrawgnsspvtdata.h"
#include "xsrawgnsssatinfo.h"
#include "xsbytearray.h"
#include "xsglovesnapshot.h"
#include "xsglovedata.h"

#include "xssnapshot.h"

/*! \cond XS_INTERNAL */
namespace XsDataPacket_Private {
	/*! \brief Abstract Variant class for handling contents of XsDataPacket */
	class Variant {
	public:
		/*! \brief Constructor, sets the dataId to \a id */
		Variant(XsDataIdentifier id) : m_id(id) {}
		virtual ~Variant() {}
		/*! \brief Read the data from message \a msg at \a offset and optionally using \a dSize */
		virtual XsSize readFromMessage(XsMessage const& msg, XsSize offset, XsSize dSize) = 0;
		/*! \brief Write the data to message \a msg at \a offset */
		virtual void writeToMessage(XsMessage& msg, XsSize offset) const = 0;
		/*! \brief Return the size the Variant would have in a message */
		virtual XsSize sizeInMsg() const = 0;
		/*! \brief Create a copy of the Variant
			\return A pointer to the newly created Variant
			\note This needs to be reimplemented in each subclass!
		*/
		virtual Variant* clone() const = 0;

		/*! \brief Set the dataId to \a id */
		void setDataId(XsDataIdentifier id)
		{
			assert((m_id & XDI_FullTypeMask) == (id & XDI_FullTypeMask));
			m_id = id;
		}
		/*! \brief Return the dataId of the Variant */
		XsDataIdentifier dataId() const { return m_id; }

		/*! \brief Convert the Variant to a derived type if allowed, returns nullptr otherwise (and asserts in debug builds) */
		template <typename U>
		U& toDerived()
		{
			U* ptr = dynamic_cast<U*>(this);
			assert(ptr);
			return *ptr;
		}

		/*! \brief Convert the Variant to a derived type if allowed, returns nullptr otherwise (and asserts in debug builds) */
		template <typename U>
		U const& toDerived() const
		{
			U const* ptr = dynamic_cast<U const*>(this);
			assert(ptr);
			return *ptr;
		}

	private:
		XsDataIdentifier m_id;
	};

	/*! \brief Generic base class template for Variant subclasses */
	template <typename T, int C = 1>
	class GenericVariant : public Variant {
	public:
		/*! \brief Constructor, sets the dataId to \a id */
		GenericVariant(XsDataIdentifier id) : Variant(id) {}

		/*! \brief Read the data from message \a msg at \a offset and optionally using \a dSize */
		XsSize readFromMessage(XsMessage const& msg, XsSize offset, XsSize sz) override
		{
			msg.getData<T>(data(), dataId(), offset, C);
			return sz;
		}
		/*! \brief Write the data to message \a msg at \a offset */
		void writeToMessage(XsMessage& msg, XsSize offset) const override
		{
			msg.setData<T>(constData(), dataId(), offset, C);
		}
		/*! \brief Return a typed pointer to the contained data */
		virtual T* data() = 0;
		/*! \brief Return a typed pointer to the contained data */
		virtual T const* constData() const = 0;

		/*! \brief Return the size the Variant would have in a message */
		XsSize sizeInMsg() const override
		{
			return (XsSize)(ptrdiff_t) XsMessage::sizeInMsg<T>(dataId(), C);
		}
	};

	/*! \brief Read the data from the message \a msg at the given \a offset */
	template<>
	inline XsSize GenericVariant<uint64_t, 1>::readFromMessage(XsMessage const& msg, XsSize offset, XsSize sz)
	{
		*data() = 0;
		*data() = ((uint64_t)XsMessage_getDataLong(&msg, offset)) << 32;
		*data() += ((uint64_t)XsMessage_getDataLong(&msg, offset + 4));
		return sz;
	}

	/*! \brief Write the data to the message \a msg at the given \a offset */
	template<>
	inline void GenericVariant<uint64_t, 1>::writeToMessage(XsMessage& msg, XsSize offset) const
	{
		XsMessage_setDataLong(&msg, (uint32_t) ((*constData()) >> 32), offset);
		XsMessage_setDataLong(&msg, (uint32_t) ((*constData()) & 0xFFFFFFFF), offset);
	}

	/*! \brief Template base class for simple single value Variants. memcpy is assumed to result in valid objects */
	template <typename T>
	struct SimpleVariant : public GenericVariant<T, 1> {
		using GenericVariant<T, 1>::dataId;

		/*! \brief Constructor, sets the dataId to \a id */
		SimpleVariant(XsDataIdentifier id) : GenericVariant<T, 1>(id), m_data() {}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		SimpleVariant(XsDataIdentifier id, T const& val) : GenericVariant<T, 1>(id), m_data(val) {}

		T m_data;			//!< The contained data
		/*! \brief Return a typed pointer to the contained data */
		T* data() override
		{
			return &m_data;
		}
		/*! \brief Return a typed pointer to the contained data */
		T const* constData() const override
		{
			return &m_data;
		}

		/*! \brief Create a copy of the Variant
		\return A pointer to the newly created Variant
		\note This needs to be reimplemented in each subclass!
		*/
		Variant* clone() const override
		{
			return new SimpleVariant<T>(dataId(), m_data);
		}

		/*! \brief Return the size the Variant would have in a message */
		XsSize sizeInMsg() const override
		{
			return XsMessage::sizeInMsg<T>(dataId(), 1);
		}
	};

	/*! \brief Template base class for complex or multi-value Variants. memcpy is NOT assumed to result in valid objects */
	template <typename U, typename T, int C>
	struct ComplexVariant : public GenericVariant<T, C>
	{
		using GenericVariant<T, C>::dataId;
		/*! \brief Constructor, sets the dataId to \a id */
		ComplexVariant(XsDataIdentifier id) : GenericVariant<T, C>(id) {}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		ComplexVariant(XsDataIdentifier id, U const& val) : GenericVariant<T, C>(id), m_data(val) {}

		U m_data;			//!< The contained data
		/*! \brief Return a typed pointer to the contained data */
		T* data() override
		{
			return const_cast<T*>(m_data.data());
		}
		/*! \brief Return a typed pointer to the contained data */
		T const* constData() const override
		{
			return m_data.data();
		}

		/*! \brief Create a copy of the Variant
		\return A pointer to the newly created Variant
		\note This needs to be reimplemented in each subclass!
		*/
		Variant* clone() const override
		{
			return new ComplexVariant<U, T, C>(dataId(), m_data);
		}
	};

	/*! \brief Variant containing an XsQuaternion value */
	struct XsQuaternionVariant : public ComplexVariant<XsQuaternion, XsReal, 4>
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsQuaternionVariant(XsDataIdentifier id) : ComplexVariant<XsQuaternion, XsReal, 4>(id) {}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsQuaternionVariant(XsDataIdentifier id, XsQuaternion const& val) : ComplexVariant<XsQuaternion, XsReal, 4>(id, val) {}

		Variant* clone() const override
		{
			return new XsQuaternionVariant(dataId(), m_data);
		}
	};

	/*! \brief Variant containing an XsUShortVector value */
	struct XsUShortVectorVariant : public ComplexVariant<XsUShortVector, uint16_t, 3>
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsUShortVectorVariant(XsDataIdentifier id) : ComplexVariant<XsUShortVector, unsigned short, 3>(id) {}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsUShortVectorVariant(XsDataIdentifier id, XsUShortVector const& val) : ComplexVariant<XsUShortVector, unsigned short, 3>(id, val) {}
		Variant* clone() const override
		{
			return new XsUShortVectorVariant(dataId(), m_data);
		}
	};

	/*! \brief Variant containing an XsVector3 value */
	struct XsVector3Variant : public ComplexVariant<XsVector3, XsReal, 3>
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsVector3Variant(XsDataIdentifier id) : ComplexVariant<XsVector3, XsReal, 3>(id) {}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsVector3Variant(XsDataIdentifier id, XsVector const& val) : ComplexVariant<XsVector3, XsReal, 3>(id, val)
		{
			assert(val.size() == 3);
		}
		Variant* clone() const override
		{
			return new XsVector3Variant(dataId(), m_data);
		}
	};

	/*! \brief Variant containing an XsVector value */
	struct XsVector2Variant : public ComplexVariant<XsVector, XsReal, 2>
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsVector2Variant(XsDataIdentifier id) : ComplexVariant<XsVector, XsReal, 2>(id, XsVector(2,0)) {}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsVector2Variant(XsDataIdentifier id, XsVector const& val) : ComplexVariant<XsVector, XsReal, 2>(id, val)
		{
			assert(val.size() == 2);
		}
		Variant* clone() const override
		{
			return new XsVector2Variant(dataId(), m_data);
		}
	};

	/*! \brief Variant containing an XsScrData value */
	struct XsScrDataVariant : public Variant
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsScrDataVariant(XsDataIdentifier id) : Variant(id) {}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsScrDataVariant(XsDataIdentifier id, XsScrData const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsScrDataVariant(dataId(), m_data);
		}

		XsScrData m_data;			//!< The contained data
		XsSize readFromMessage(XsMessage const& msg, XsSize offset, XsSize sz) override
		{
			for (XsSize i = 0; i < 3; ++i, offset +=2)
				m_data.m_acc[i] = msg.getDataShort(offset);
			for (XsSize i = 0; i < 3; ++i, offset +=2)
				m_data.m_gyr[i] = msg.getDataShort(offset);
			for (XsSize i = 0; i < 3; ++i, offset +=2)
				m_data.m_mag[i] = msg.getDataShort(offset);
			m_data.m_temp = msg.getDataShort(offset);
			return sz;
		}
		void writeToMessage(XsMessage& msg, XsSize offset) const override
		{
			for (XsSize i = 0; i < 3; ++i, offset +=2)
				msg.setDataShort(m_data.m_acc[i], offset);
			for (XsSize i = 0; i < 3; ++i, offset +=2)
				msg.setDataShort(m_data.m_gyr[i], offset);
			for (XsSize i = 0; i < 3; ++i, offset +=2)
				msg.setDataShort(m_data.m_mag[i], offset);
			msg.setDataShort(m_data.m_temp, offset);
		}

		XsSize sizeInMsg() const override
		{
			return 10*sizeof(uint16_t);
		}
	};

	/*! \brief Variant containing an XsTriggerIndication value */
	struct XsTriggerIndicationDataVariant : public Variant
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsTriggerIndicationDataVariant(XsDataIdentifier id) : Variant(id) {}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsTriggerIndicationDataVariant(XsDataIdentifier id, XsTriggerIndicationData const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsTriggerIndicationDataVariant(dataId(), m_data);
		}

		XsTriggerIndicationData m_data;			//!< The contained data
		XsSize readFromMessage(XsMessage const& msg, XsSize offset, XsSize sz) override
		{
			m_data.m_line        = XsMessage_getDataByte (&msg, offset + 0);
			m_data.m_polarity    = XsMessage_getDataByte (&msg, offset + 1);
			m_data.m_timestamp   = XsMessage_getDataLong (&msg, offset + 2);
			m_data.m_frameNumber = XsMessage_getDataShort(&msg, offset + 6);
			return sz;
		}
		void writeToMessage(XsMessage& msg, XsSize offset) const override
		{
			XsMessage_setDataByte (&msg, m_data.m_line,        offset + 0);
			XsMessage_setDataByte (&msg, m_data.m_polarity,    offset + 1);
			XsMessage_setDataLong (&msg, m_data.m_timestamp,   offset + 2);
			XsMessage_setDataShort(&msg, m_data.m_frameNumber, offset + 6);
		}

		XsSize sizeInMsg() const override
		{
			return 8;
		}
	};

	/*! \brief Variant containing an XsEuler value */
	struct XsEulerVariant : public ComplexVariant<XsEuler, XsReal, 3>
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsEulerVariant(XsDataIdentifier id) : ComplexVariant<XsEuler, XsReal, 3>(id) {}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsEulerVariant(XsDataIdentifier id, XsEuler const& val) : ComplexVariant<XsEuler, XsReal, 3>(id, val) {}
		Variant* clone() const override
		{
			return new XsEulerVariant(dataId(), m_data);
		}
	};

	/*! \brief Variant containing an XsMatrix value */
	struct XsMatrixVariant : public ComplexVariant<XsMatrix3x3, XsReal, 9>
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsMatrixVariant(XsDataIdentifier id) : ComplexVariant<XsMatrix3x3, XsReal, 9>(id) {}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsMatrixVariant(XsDataIdentifier id, XsMatrix const& val) : ComplexVariant<XsMatrix3x3, XsReal, 9>(id, val) {}
		Variant* clone() const override
		{
			return new XsMatrixVariant(dataId(), m_data);
		}

		XsSize readFromMessage(XsMessage const& msg, XsSize offset, XsSize sz) override
		{
			XsSize ds = XsMessage_getFPValueSize(dataId());
			XsSize k = 0;
			for (int i=0 ; i<3 ; ++i)
				for (XsSize j=0 ; j<3 ; ++j, k+=ds)
					XsMessage_getDataRealValuesById(&msg, dataId(), &m_data[j][i], offset+k, 1);
			return sz;
		}
		void writeToMessage(XsMessage& msg, XsSize offset) const override
		{
			XsSize ds = XsMessage_getFPValueSize(dataId());
			XsSize k = 0;
			for (int i=0 ; i<3 ; ++i)
				for (XsSize j=0 ; j<3 ; ++j, k+=ds)
					XsMessage_setDataRealValuesById(&msg, dataId(), &m_data[j][i], offset+k, 1);
		}
	};

	/*! \brief Variant containing an XsRange value */
	struct XsRangeVariant : public Variant
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsRangeVariant(XsDataIdentifier id) : Variant(id) {}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsRangeVariant(XsDataIdentifier id, XsRange const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsRangeVariant(dataId(), m_data);
		}

		XsRange m_data;			//!< The contained data
		XsSize readFromMessage(XsMessage const& msg, XsSize offset, XsSize sz) override
		{
			// unwrap
			uint16_t first = (uint16_t) XsMessage_getDataShort(&msg, offset + 0);
			uint16_t last = (uint16_t) XsMessage_getDataShort(&msg, offset + 2);
			m_data.setRange(first, (int)((uint16_t)(last - first)) + (int)first);
			return sz;
		}
		void writeToMessage(XsMessage& msg, XsSize offset) const override
		{
			XsMessage_setDataShort(&msg, (uint16_t) m_data.first(), offset + 0);
			XsMessage_setDataShort(&msg, (uint16_t) m_data.last(), offset + 2);
		}

		XsSize sizeInMsg() const override
		{
			return 4;
		}
	};

	/*! \brief Variant containing an XsTimeInfo value */
	struct XsTimeInfoVariant : public Variant
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsTimeInfoVariant(XsDataIdentifier id) : Variant(id), m_data() {}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsTimeInfoVariant(XsDataIdentifier id, XsTimeInfo const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsTimeInfoVariant(dataId(), m_data);
		}

		XsTimeInfo m_data;			//!< The contained data
		XsSize readFromMessage(XsMessage const& msg, XsSize offset, XsSize sz) override
		{
			m_data.m_nano = XsMessage_getDataLong(&msg, offset);
			m_data.m_year = XsMessage_getDataShort(&msg, offset+4);

			// month, day, hour, minute, second and valid
			uint8_t* bareByte = (uint8_t*) &m_data.m_month;
			for (XsSize i=0; i < 6; ++i)
				bareByte[i] = XsMessage_getDataByte(&msg, offset + 6 + i);

			m_data.m_utcOffset = 0;
			return sz;
		}
		void writeToMessage(XsMessage& msg, XsSize offset) const override
		{
			// update
			XsMessage_setDataLong(&msg, m_data.m_nano, offset);
			XsMessage_setDataShort(&msg, m_data.m_year, offset + 4);

			// month, day, hour, minute, second and valid
			uint8_t const* bareByte = (uint8_t const*) &m_data.m_month;
			for (XsSize i=0; i<6;++i)
				XsMessage_setDataByte(&msg, bareByte[i], offset + 6 + i);

			// utcOffset is ignored and assumed to be 0, use makeUtc to ensure this if necessary
		}

		XsSize sizeInMsg() const override
		{
			return 12;
		}
	};

	/*! \brief Variant containing an XsRawGnssPvtData value */
	struct XsRawGnssPvtDataVariant : public Variant
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsRawGnssPvtDataVariant(XsDataIdentifier id) : Variant(id), m_data() {}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsRawGnssPvtDataVariant(XsDataIdentifier id, XsRawGnssPvtData const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsRawGnssPvtDataVariant(dataId(), m_data);
		}

		XsRawGnssPvtData m_data;			//!< The contained data
		XsSize readFromMessage(XsMessage const& msg, XsSize offset, XsSize sz) override
		{
			m_data.m_itow    = XsMessage_getDataLong(&msg, offset + 0);
			m_data.m_year   = XsMessage_getDataShort(&msg, offset + 4);
			m_data.m_month   = XsMessage_getDataByte(&msg, offset + 6);
			m_data.m_day     = XsMessage_getDataByte(&msg, offset + 7);
			m_data.m_hour    = XsMessage_getDataByte(&msg, offset + 8);
			m_data.m_min     = XsMessage_getDataByte(&msg, offset + 9);
			m_data.m_sec     = XsMessage_getDataByte(&msg, offset + 10);
			m_data.m_valid   = XsMessage_getDataByte(&msg, offset + 11);
			m_data.m_tAcc    = XsMessage_getDataLong(&msg, offset + 12);
			m_data.m_nano    = XsMessage_getDataLong(&msg, offset + 16);
			m_data.m_fixType = XsMessage_getDataByte(&msg, offset + 20);
			m_data.m_flags   = XsMessage_getDataByte(&msg, offset + 21);
			m_data.m_numSv   = XsMessage_getDataByte(&msg, offset + 22);
			m_data.m_res1    = XsMessage_getDataByte(&msg, offset + 23);
			m_data.m_lon     = XsMessage_getDataLong(&msg, offset + 24);
			m_data.m_lat     = XsMessage_getDataLong(&msg, offset + 28);
			m_data.m_height  = XsMessage_getDataLong(&msg, offset + 32);
			m_data.m_hMsl    = XsMessage_getDataLong(&msg, offset + 36);
			m_data.m_hAcc    = XsMessage_getDataLong(&msg, offset + 40);
			m_data.m_vAcc    = XsMessage_getDataLong(&msg, offset + 44);
			m_data.m_velN    = XsMessage_getDataLong(&msg, offset + 48);
			m_data.m_velE    = XsMessage_getDataLong(&msg, offset + 52);
			m_data.m_velD    = XsMessage_getDataLong(&msg, offset + 56);
			m_data.m_gSpeed  = XsMessage_getDataLong(&msg, offset + 60);
			m_data.m_headMot = XsMessage_getDataLong(&msg, offset + 64);
			m_data.m_sAcc    = XsMessage_getDataLong(&msg, offset + 68);
			m_data.m_headAcc = XsMessage_getDataLong(&msg, offset + 72);
			m_data.m_headVeh = XsMessage_getDataLong(&msg, offset + 76);
			m_data.m_gdop   = XsMessage_getDataShort(&msg, offset + 80);
			m_data.m_pdop   = XsMessage_getDataShort(&msg, offset + 82);
			m_data.m_tdop   = XsMessage_getDataShort(&msg, offset + 84);
			m_data.m_vdop   = XsMessage_getDataShort(&msg, offset + 86);
			m_data.m_hdop   = XsMessage_getDataShort(&msg, offset + 88);
			m_data.m_ndop   = XsMessage_getDataShort(&msg, offset + 90);
			m_data.m_edop   = XsMessage_getDataShort(&msg, offset + 92);
			return sz;
		}
		void writeToMessage(XsMessage& msg, XsSize offset) const override
		{
			XsMessage_setDataLong (&msg, m_data.m_itow   , offset + 0);
			XsMessage_setDataShort(&msg, m_data.m_year   , offset + 4);
			XsMessage_setDataByte (&msg, m_data.m_month  , offset + 6);
			XsMessage_setDataByte (&msg, m_data.m_day    , offset + 7);
			XsMessage_setDataByte (&msg, m_data.m_hour   , offset + 8);
			XsMessage_setDataByte (&msg, m_data.m_min    , offset + 9);
			XsMessage_setDataByte (&msg, m_data.m_sec    , offset + 10);
			XsMessage_setDataByte (&msg, m_data.m_valid  , offset + 11);
			XsMessage_setDataLong (&msg, m_data.m_tAcc   , offset + 12);
			XsMessage_setDataLong (&msg, m_data.m_nano   , offset + 16);
			XsMessage_setDataByte (&msg, m_data.m_fixType, offset + 20);
			XsMessage_setDataByte (&msg, m_data.m_flags  , offset + 21);
			XsMessage_setDataByte (&msg, m_data.m_numSv  , offset + 22);
			XsMessage_setDataByte (&msg, m_data.m_res1   , offset + 23);
			XsMessage_setDataLong (&msg, m_data.m_lon    , offset + 24);
			XsMessage_setDataLong (&msg, m_data.m_lat    , offset + 28);
			XsMessage_setDataLong (&msg, m_data.m_height , offset + 32);
			XsMessage_setDataLong (&msg, m_data.m_hMsl   , offset + 36);
			XsMessage_setDataLong (&msg, m_data.m_hAcc   , offset + 40);
			XsMessage_setDataLong (&msg, m_data.m_vAcc   , offset + 44);
			XsMessage_setDataLong (&msg, m_data.m_velN   , offset + 48);
			XsMessage_setDataLong (&msg, m_data.m_velE   , offset + 52);
			XsMessage_setDataLong (&msg, m_data.m_velD   , offset + 56);
			XsMessage_setDataLong (&msg, m_data.m_gSpeed , offset + 60);
			XsMessage_setDataLong (&msg, m_data.m_headMot, offset + 64);
			XsMessage_setDataLong (&msg, m_data.m_sAcc   , offset + 68);
			XsMessage_setDataLong (&msg, m_data.m_headAcc, offset + 72);
			XsMessage_setDataLong (&msg, m_data.m_headVeh, offset + 76);
			XsMessage_setDataShort(&msg, m_data.m_gdop   , offset + 80);
			XsMessage_setDataShort(&msg, m_data.m_pdop   , offset + 82);
			XsMessage_setDataShort(&msg, m_data.m_tdop   , offset + 84);
			XsMessage_setDataShort(&msg, m_data.m_vdop   , offset + 86);
			XsMessage_setDataShort(&msg, m_data.m_hdop   , offset + 88);
			XsMessage_setDataShort(&msg, m_data.m_ndop   , offset + 90);
			XsMessage_setDataShort(&msg, m_data.m_edop   , offset + 92);
		}

		XsSize sizeInMsg() const override
		{
			return 94;
		}
	};

	/*! \brief Variant containing an XsRawGnssSatInfo value */
	struct XsRawGnssSatInfoVariant : public Variant
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsRawGnssSatInfoVariant(XsDataIdentifier id) : Variant(id), m_data() {}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsRawGnssSatInfoVariant(XsDataIdentifier id, XsRawGnssSatInfo const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsRawGnssSatInfoVariant(dataId(), m_data);
		}

		XsRawGnssSatInfo m_data;			//!< The contained data
		XsSize readFromMessage(XsMessage const& msg, XsSize offset, XsSize sz) override
		{
			m_data.m_itow   = XsMessage_getDataLong(&msg, offset + 0);
			m_data.m_numSvs = XsMessage_getDataByte(&msg, offset + 4);
			m_data.m_res1   = XsMessage_getDataByte(&msg, offset + 5);
			m_data.m_res2   = XsMessage_getDataByte(&msg, offset + 6);
			m_data.m_res3   = XsMessage_getDataByte(&msg, offset + 7);

			offset = offset + 8;
			for (uint8_t i = 0; i < m_data.m_numSvs; ++i)
			{
				m_data.m_satInfos[i].m_gnssId = XsMessage_getDataByte(&msg, offset + 0);
				m_data.m_satInfos[i].m_svId   = XsMessage_getDataByte(&msg, offset + 1);
				m_data.m_satInfos[i].m_cno    = XsMessage_getDataByte(&msg, offset + 2);
				m_data.m_satInfos[i].m_flags  = XsMessage_getDataByte(&msg, offset + 3);
				offset += 4;
			}
			return sz;
		}
		void writeToMessage(XsMessage& msg, XsSize offset) const override
		{
			XsMessage_setDataLong(&msg, m_data.m_itow   , offset + 0);
			XsMessage_setDataByte(&msg, m_data.m_numSvs , offset + 4);
			XsMessage_setDataByte(&msg, m_data.m_res1   , offset + 5);
			XsMessage_setDataByte(&msg, m_data.m_res2   , offset + 6);
			XsMessage_setDataByte(&msg, m_data.m_res3   , offset + 7);

			offset = offset + 8;
			for (uint8_t i = 0; i < m_data.m_numSvs; ++i)
			{
				XsMessage_setDataByte(&msg, m_data.m_satInfos[i].m_gnssId , offset + 0);
				XsMessage_setDataByte(&msg, m_data.m_satInfos[i].m_svId   , offset + 1);
				XsMessage_setDataByte(&msg, m_data.m_satInfos[i].m_cno    , offset + 2);
				XsMessage_setDataByte(&msg, m_data.m_satInfos[i].m_flags  , offset + 3);
				offset += 4;
			}
		}

		XsSize sizeInMsg() const override
		{
			return 8 + 4*m_data.m_numSvs;
		}
	};

	/*! \brief Variant containing an XsFullSnaphsot value */
	struct XsFullSnapshotVariant : public Variant
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsFullSnapshotVariant(XsDataIdentifier id) : Variant(id) {}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsFullSnapshotVariant(XsDataIdentifier id, XsSnapshot const& val) : Variant(id), m_data(val) {}

		Variant* clone() const override
		{
			return new XsFullSnapshotVariant(dataId(), m_data);
		}

		XsSnapshot m_data;			//!< The contained data
		XsSize readFromMessage(XsMessage const& msg, XsSize offset, XsSize sz) override
		{
			m_data.m_frameNumber = XsMessage_getDataShort(&msg, offset); offset += 2;
			m_data.m_timestamp = XsMessage_getDataLongLong(&msg, offset); offset += 8;
			for (int i = 0; i < 4; ++i, offset += 4)
				m_data.m_iQ[i] = XsMessage_getDataLong(&msg, offset);
			for (int i = 0; i < 3; ++i, offset += 8)
				m_data.m_iV[i] = XsMessage_getDataLongLong(&msg, offset);
			for (int i = 0; i < 3; ++i, offset += 4)
				m_data.m_mag[i] = XsMessage_getDataLong(&msg, offset);

			m_data.m_baro = XsMessage_getDataLong(&msg, offset); offset += 4;
			m_data.m_accClippingCounter = XsMessage_getDataByte(&msg, offset); offset += 1;
			m_data.m_gyrClippingCounter = XsMessage_getDataByte(&msg, offset); offset += 1;
			m_data.m_status = XsMessage_getDataShort(&msg, offset);
			m_data.m_type = ST_Full;
			return sz;
		}

		void writeToMessage(XsMessage& msg, XsSize offset) const override
		{
			assert(m_data.m_type == ST_Full);
			XsMessage_setDataShort(&msg, (uint16_t) m_data.m_frameNumber, offset);	offset += 2;
			XsMessage_setDataLongLong(&msg, m_data.m_timestamp, offset); offset += 8;
			for (int i = 0; i < 4; ++i, offset += 4)
				XsMessage_setDataLong(&msg, m_data.m_iQ[i], offset);
			for (int i = 0; i < 3; ++i, offset += 8)
				XsMessage_setDataLongLong(&msg, m_data.m_iV[i], offset);
			for (int i = 0; i < 3; ++i, offset += 4)
				XsMessage_setDataLong(&msg, m_data.m_mag[i], offset);

			XsMessage_setDataLong(&msg, m_data.m_baro, offset);	offset += 4;
			XsMessage_setDataByte(&msg, m_data.m_accClippingCounter, offset); offset += 1;
			XsMessage_setDataByte(&msg, m_data.m_gyrClippingCounter, offset); offset += 1;
			XsMessage_setDataShort(&msg, m_data.m_status, offset);
		}

		XsSize sizeInMsg() const override
		{
			return 70;
		}
	};

	/*! \brief Variant containing an XsAwindaSnapshot value */
	struct XsAwindaSnapshotVariant : public Variant
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsAwindaSnapshotVariant(XsDataIdentifier id) : Variant(id)
		{
			m_data.m_type = ST_Awinda;
		}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsAwindaSnapshotVariant(XsDataIdentifier id, XsSnapshot const& val) : Variant(id), m_data(val)
		{
			m_data.m_type = ST_Awinda;
		}
		Variant* clone() const override
		{
			return new XsAwindaSnapshotVariant(dataId(), m_data);
		}

		XsSnapshot m_data;			//!< The contained data
		XsSize readFromMessage(XsMessage const& msg, XsSize offset, XsSize sz) override
		{
			m_data.m_deviceId    = XsMessage_getDataLong(&msg, offset);	offset += 4;
			m_data.m_frameNumber = XsMessage_getDataLong(&msg, offset);	offset += 4;
			for (int i = 0; i < 3; ++i, offset += 4)
				m_data.m_iQ[i] = XsMessage_getDataLong(&msg, offset);
			for (int i = 0; i < 3; ++i, offset += 4)
				m_data.m_iV[i] = (int64_t)(int32_t) XsMessage_getDataLong(&msg, offset);
			for (int i = 0; i < 3; ++i, offset += 2)
				m_data.m_mag[i] = (int32_t)(int16_t) XsMessage_getDataShort(&msg, offset);
			m_data.m_baro = XsMessage_getDataLong(&msg, offset);	offset += 4;
			m_data.m_status = XsMessage_getDataShort(&msg, offset);	offset += 2;
			m_data.m_accClippingCounter = XsMessage_getDataByte(&msg, offset);	offset += 1;
			m_data.m_gyrClippingCounter = XsMessage_getDataByte(&msg, offset);
			m_data.m_type = ST_Awinda;
			return sz;
		}
		void writeToMessage(XsMessage& msg, XsSize offset) const override
		{
			assert(m_data.m_type == ST_Awinda);
			XsMessage_setDataLong(&msg, m_data.m_deviceId.legacyDeviceId(), offset);	offset += 4;
			XsMessage_setDataLong(&msg, m_data.m_frameNumber, offset);	offset += 4;
			for (int i = 0; i < 3; ++i, offset += 4)
				XsMessage_setDataLong(&msg, m_data.m_iQ[i], offset);
			for (int i = 0; i < 3; ++i, offset += 4)
				XsMessage_setDataLong(&msg, (uint32_t)(int32_t) m_data.m_iV[i], offset);
			for (int i = 0; i < 3; ++i, offset += 2)
				XsMessage_setDataShort(&msg, (uint16_t)(int16_t) m_data.m_mag[i], offset);
			XsMessage_setDataLong(&msg, m_data.m_baro, offset);	offset += 4;
			XsMessage_setDataShort(&msg, m_data.m_status, offset);	offset += 2;
			XsMessage_setDataByte(&msg, m_data.m_accClippingCounter, offset);	offset += 1;
			XsMessage_setDataByte(&msg, m_data.m_gyrClippingCounter, offset);
		}

		XsSize sizeInMsg() const override
		{
			return 46;
		}
	};

	/*! \brief Variant containing an XsByteArray value */
	struct XsByteArrayVariant : public Variant
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsByteArrayVariant(XsDataIdentifier id) : Variant(id)
		{
		}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsByteArrayVariant(XsDataIdentifier id, XsByteArray const& val) : Variant(id), m_data(val)
		{
		}
		Variant* clone() const override
		{
			return new XsByteArrayVariant(dataId(), m_data);
		}

		XsByteArray m_data;			//!< The contained data
		XsSize readFromMessage(XsMessage const& msg, XsSize offset, XsSize dSize) override
		{
			if (dSize)
			{
				if (dSize < 255)
				{
					m_data.assign(dSize, XsMessage_getDataBuffer(&msg, offset));
					return dSize;
				}
				else
				{
					const uint16_t exactId = XsMessage_getDataShort(&msg, offset-3);
					XsSize msgSize = msg.getDataSize();
					XsSize total = 0;

					// look ahead to get total size
					XsSize sSize = dSize;
					XsSize sOffset = offset;
					XsSize tSize = 255;
					while (sSize == 255)
					{
						sOffset += 255;
						if (sOffset >= msgSize)
							break;
						uint16_t nextId = XsMessage_getDataShort(&msg, sOffset);
						if (nextId != exactId)
							break;
						sSize = XsMessage_getDataByte(&msg, sOffset+2);
						sOffset += 3;
						tSize += sSize;
					}
					m_data.setSize(tSize);

					sOffset = 0;
					while (dSize == 255)
					{
						memcpy(m_data.data()+sOffset, XsMessage_getDataBuffer(&msg, offset), 255);
						total += 258;
						offset += 255;
						sOffset += 255;
						if (offset >= msgSize)
						{
							dSize = 0;
							break;
						}
						uint16_t nextId = XsMessage_getDataShort(&msg, offset);
						if (nextId != exactId)
						{
							dSize = 0;
							break;
						}
						dSize = XsMessage_getDataByte(&msg, offset+2);
						offset += 3;
					}
					if (dSize)
					{
						memcpy(m_data.data()+sOffset, XsMessage_getDataBuffer(&msg, offset), dSize);
						total += dSize;
					}
					return total;
				}
			}
			else
			{
				m_data.clear();
				return 0;
			}
		}
		void writeToMessage(XsMessage& msg, XsSize offset) const override
		{
			if (m_data.size())
			{
				if (m_data.size() < 255)
					XsMessage_setDataBuffer(&msg, m_data.data(), m_data.size(), offset);
				else
				{
					XsSize dataOffset = 0;
					XsSize remainingSize = m_data.size();
					while (remainingSize >= 255)
					{
						XsMessage_setDataBuffer(&msg, m_data.data() + dataOffset, 255, offset);
						offset += 258;
						dataOffset += 255;
						remainingSize -= 255;
					}
					if (remainingSize)
						XsMessage_setDataBuffer(&msg, m_data.data() + dataOffset, remainingSize, offset);
				}
			}
		}

		XsSize sizeInMsg() const override
		{
			return m_data.size();
		}
	};

	/*! \brief Variant containing an XsGloveSnapshotVariant value */
	struct XsGloveSnapshotVariant : public Variant
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsGloveSnapshotVariant(XsDataIdentifier id) : Variant(id)
		{
		}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsGloveSnapshotVariant(XsDataIdentifier id, XsGloveSnapshot const& val) : Variant(id), m_data(val)
		{
		}
		Variant* clone() const override
		{
			return new XsGloveSnapshotVariant(dataId(), m_data);
		}

		XsGloveSnapshot m_data;			//!< The contained data
		XsSize readFromMessage(XsMessage const& msg, XsSize offset, XsSize dSize) override
		{
			const uint16_t exactId = XsMessage_getDataShort(&msg, offset - 3);
			assert(dSize == 255);
			if (dSize != 255)
				return 0;
			memcpy((uint8_t*)&m_data, XsMessage_getDataBuffer(&msg, offset), 255);
			offset += 255;
			uint16_t nextId = XsMessage_getDataShort(&msg, offset);
			assert(nextId == exactId);
			if (nextId != exactId)
				return 0;
			dSize = XsMessage_getDataByte(&msg, offset + 2);
			assert(dSize == 124);
			if (dSize != 124)
				return 0;
			offset += 3;
			memcpy(((uint8_t*)&m_data)+255, XsMessage_getDataBuffer(&msg, offset), 124);

			// loop over finger / struct and swap bytes
			m_data.m_snapshotCounter = (uint16_t)swapEndian16(m_data.m_snapshotCounter);
			m_data.m_validSampleFlags = (uint16_t)swapEndian16(m_data.m_validSampleFlags);
			m_data.m_timestamp = (uint16_t)swapEndian16(m_data.m_timestamp);

			for (int i = 0; i < 12; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					{
						auto& tmp = m_data.m_fingers[i].m_iV[j];
						m_data.m_fingers[i].m_iV[j] = (uint32_t)swapEndian32(tmp);
					}
					{
						auto& tmp = m_data.m_fingers[i].m_mag[j];
						tmp = (int16_t)swapEndian16(tmp);
					}
				}
				{
					//const auto& tmp = swapEndian16(m_data.m_fingers[i].m_flags);
				}
			}

			return 255+124;
		}
		void writeToMessage(XsMessage& msg, XsSize offset) const override
		{
			assert(0); //not expected to be called
			XsMessage_setDataShort(&msg, m_data.m_snapshotCounter, offset);
		}

		XsSize sizeInMsg() const override
		{
			return sizeof(XsGloveSnapshot);
		}
	};

	/*! \brief Variant containing an XsGloveDataVariant value */
	struct XsGloveDataVariant : public Variant
	{
		/*! \brief Constructor, sets the dataId to \a id */
		XsGloveDataVariant(XsDataIdentifier id) : Variant(id), m_data() {}
		/*! \brief Constructor, sets the dataId to \a id and data value to \a val */
		XsGloveDataVariant(XsDataIdentifier id, XsGloveData const& val) : Variant(id), m_data(val) {}
		Variant* clone() const override
		{
			return new XsGloveDataVariant(dataId(), m_data);
		}

		XsGloveData m_data;			//!< The contained data
		XsSize readFromMessage(XsMessage const& msg, XsSize offset, XsSize sz) override
		{
			assert(0); //not expected to be called
			XsMessage_getDataShort(&msg, offset);
			return sz;
		}
		void writeToMessage(XsMessage& msg, XsSize offset) const override
		{
			assert(0); //not expected to be called
			XsMessage_setDataShort(&msg, m_data.snapshotCounter(), offset);
		}

		XsSize sizeInMsg() const override
		{
			return sizeof(XsGloveData);
		}
	};
}

typedef std::map<XsDataIdentifier, XsDataPacket_Private::Variant*> MapType;

struct DataPacketPrivate : private MapType
{
	DataPacketPrivate() : m_refCount(1) { ++m_created; }
	DataPacketPrivate(DataPacketPrivate const&);
	~DataPacketPrivate();
	DataPacketPrivate& operator = (const DataPacketPrivate& p);
	void erase(XsDataIdentifier id);
	void erase(MapType::const_iterator const& it);
	MapType::iterator insert(XsDataIdentifier id, XsDataPacket_Private::Variant* var);

	void clear();
	void merge(DataPacketPrivate const& other, bool overwrite);

	MapType::const_iterator find(XsDataIdentifier id) const;

	using MapType::begin;
	using MapType::end;
	using MapType::size;
	using MapType::empty;
	mutable volatile std::atomic_int m_refCount;	//!< The reference count for this DataPacketPrivate.
	static volatile std::atomic_int m_created;		//!< The number of DataPacketPrivate objects created so far. \sa creationDiff()
	static volatile std::atomic_int m_destroyed;	//!< The number of DataPacketPrivate objects destroyed so far. \sa creationDiff()

	static int creationDiff();
};

/*! \endcond */

#endif

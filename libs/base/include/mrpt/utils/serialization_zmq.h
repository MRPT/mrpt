/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  SERIALIZATION_ZMQ_H
#define  SERIALIZATION_ZMQ_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CMemoryStream.h>

namespace mrpt
{
	namespace utils
	{
		/** \addtogroup noncstream_serialization_zmq Serialization functions for ZMQ (in #include <mrpt/utils/serialization_zmq.h>)
		  * \ingroup noncstream_serialization
		  * @{ */

		/** Send an MRPT object to a ZMQ socket.
		 * \param[in] obj The object to be serialized and sent to the socket.
		 * \param[in] zmq_socket The zmq socket object.
		 * \param[in] max_packet_len The object will be split into a series of ZMQ "message parts" of this maximum length (in bytes). Default=0, which means do not split in parts.
		 * \note Including `<mrpt/utils/serialization_zmq.h>` requires libzmq to be available in your system and linked
		 *  to your user code. This function can be used even if MRPT was built without ZMQ support, thanks to the use of templates.
		 * \exception std::exception If the object finds any critical error during serialization or on ZMQ errors.
		 */
		template <typename ZMQ_SOCKET_TYPE>
		void mrpt_send_to_zmq(
			ZMQ_SOCKET_TYPE zmq_socket,
			const mrpt::utils::CSerializable *obj,
			const size_t max_packet_len = 0)
		{
			mrpt::utils::internal::TFreeFnDataForZMQ *fd = new mrpt::utils::internal::TFreeFnDataForZMQ();
			if (!fd) throw std::bad_alloc();
			fd->buf=new mrpt::utils::CMemoryStream();
			fd->do_free = true;
			if (!fd->buf) throw std::bad_alloc();

			fd->buf->WriteObject(obj);
			const size_t nBytes = fd->buf->getTotalBytesCount();

			zmq_msg_t message;
			if (0!=zmq_msg_init_data(&message, fd->buf->getRawBufferData(), nBytes, &mrpt::utils::internal::free_fn_for_zmq, fd))
				throw std::runtime_error("[mrpt_send_to_zmq] Error in zmq_msg_init_data()");
			int size = zmq_msg_send (&message, zmq_socket, 0);
			if (0!=zmq_msg_close (&message))
				throw std::runtime_error("[mrpt_send_to_zmq] Error in zmq_msg_close()");
			if (size!=static_cast<int>(nBytes))
				throw std::runtime_error("[mrpt_send_to_zmq] Error in zmq_msg_send()");
		}

		/** Receives an MRPT object from a ZMQ socket, determining the type of the
		 *  object on-the-fly.
		 * \param[in] zmq_socket The zmq socket object.
		 * \param[in] dont_wait If true, will fail if there is no data ready to
		 *  be read. If false (default) this function will block until data arrives.
		 * \param[out] rx_obj_length_in_bytes If non-NULL, the object length will be stored here.
		 * \return An empty smart pointer if there was any error. The received
		 *  object if all went OK.
		 * \note Including `<mrpt/utils/serialization_zmq.h>` requires libzmq to be
		 *  available in your system and linked to your user code. This function
		 *  can be used even if MRPT was built without ZMQ support, thanks to the
		 *  use of templates.
		 * \exception std::exception If the object finds any critical error during de-serialization.
		 * \sa mrpt_recv_from_zmq_into
		 */
		template <typename ZMQ_SOCKET_TYPE>
		mrpt::utils::CSerializablePtr mrpt_recv_from_zmq(ZMQ_SOCKET_TYPE zmq_socket, bool dont_wait = false, size_t * rx_obj_length_in_bytes = NULL)
		{
			mrpt::utils::CSerializablePtr obj;
			if (rx_obj_length_in_bytes) *rx_obj_length_in_bytes = 0;
			// Init rx msg:
			zmq_msg_t msg;
			if (0!=zmq_msg_init (&msg))
				return obj;
			// Recv:
			const int len = zmq_msg_recv(&msg,zmq_socket, dont_wait ? ZMQ_DONTWAIT : 0);
			if (len <=0 ) return obj;
			if (rx_obj_length_in_bytes) *rx_obj_length_in_bytes = len;
			// De-serialize:
			{
				CMemoryStream	tmp;
				tmp.assignMemoryNotOwn(zmq_msg_data(&msg), len);
				obj = tmp.ReadObject();
			}
			zmq_msg_close (&msg); // Free msg
			return obj;
		}
		/** Like mrpt_recv_from_zmq() but without dynamically allocating the received object,
		  * more efficient to use if the type of the received object is known in advance.
			* \param[in] target_object The received object will be stored here. An exception will be raised upon type mismatch.
			* \return true if all was OK, false on any ZMQ error.
			* \sa mrpt_recv_from_zmq() for details on the rest of parameters.
		  */
		template <typename ZMQ_SOCKET_TYPE>
		bool mrpt_recv_from_zmq_into(ZMQ_SOCKET_TYPE zmq_socket, mrpt::utils::CSerializable &target_object, bool dont_wait = false, size_t * rx_obj_length_in_bytes = NULL)
		{
			if (rx_obj_length_in_bytes) *rx_obj_length_in_bytes = 0;
			// Init rx msg:
			zmq_msg_t msg;
			if (0!=zmq_msg_init (&msg))
				return false;
			// Recv:
			const int len = zmq_msg_recv(&msg,zmq_socket, dont_wait ? ZMQ_DONTWAIT : 0);
			if (len <=0 ) return false;
			if (rx_obj_length_in_bytes) *rx_obj_length_in_bytes = len;
			// De-serialize:
			{
				CMemoryStream	tmp;
				tmp.assignMemoryNotOwn(zmq_msg_data(&msg), len);
				tmp.ReadObject(&target_object);
			}
			zmq_msg_close (&msg); // Free msg
			return true;
		}

		/** @} */
	} // End of namespace
} // End of namespace
#endif

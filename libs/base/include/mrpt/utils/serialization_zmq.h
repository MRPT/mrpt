/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  SERIALIZATION_ZMQ_H
#define  SERIALIZATION_ZMQ_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CMemoryStream.h>
#include <cmath> // ceil()

namespace mrpt
{
	namespace utils
	{
		/** \addtogroup noncstream_serialization_zmq Serialization functions for ZMQ (v3 or above) (in #include <mrpt/utils/serialization_zmq.h>)
		  * \ingroup noncstream_serialization
		  * @{ */

		/** Send an MRPT object to a ZMQ socket.
		 * \param[in] obj The object to be serialized and sent to the socket.
		 * \param[in] zmq_socket The zmq socket object.
		 * \param[in] max_packet_len The object will be split into a series of ZMQ "message parts" of this maximum length (in bytes). Default=0, which means do not split in parts.
		 * \note Including `<mrpt/utils/serialization_zmq.h>` requires libzmq to be available in your system and linked
		 *  to your user code. This function can be used even if MRPT was built without ZMQ support, thanks to the use of templates.
		 * \exception std::exception If the object finds any critical error during serialization or on ZMQ errors.
		 * \note See examples of usage in https://github.com/MRPT/mrpt/tree/master/doc/mrpt-zeromq-example
		 */
		template <typename ZMQ_SOCKET_TYPE>
		void mrpt_send_to_zmq(
			ZMQ_SOCKET_TYPE zmq_socket,
			const mrpt::utils::CSerializable &obj,
			const size_t max_packet_len = 0)
		{
			mrpt::utils::CMemoryStream *buf = new mrpt::utils::CMemoryStream();
			if (!buf) throw std::bad_alloc();

			buf->WriteObject(&obj);
			const size_t nBytes = buf->getTotalBytesCount();
			if (!nBytes)
				throw std::runtime_error("[mrpt_send_to_zmq] Serialized object has 0 bytes, which probably means something went wrong...");
			unsigned int nPkts = (!max_packet_len) ? 1U : static_cast<unsigned int>(ceil(double(nBytes)/max_packet_len));
			for (unsigned int iPkt=0;iPkt<nPkts;++iPkt)
			{
				// Prepare a msg part:
				mrpt::utils::internal::TFreeFnDataForZMQ *fd = new mrpt::utils::internal::TFreeFnDataForZMQ();
				if (!fd) throw std::bad_alloc();
				fd->buf = buf;
				fd->do_free = iPkt==(nPkts-1); // Free buffer only after the last part is disposed.
				void *pkt_data = reinterpret_cast<char*>(fd->buf->getRawBufferData()) + max_packet_len*iPkt;
				size_t nBytesThisPkt = nBytes-max_packet_len*iPkt;
				if (max_packet_len!=0 && nBytesThisPkt>max_packet_len) nBytesThisPkt=max_packet_len;
				// Build ZMQ msg:
				zmq_msg_t message;
				if (0!=zmq_msg_init_data(&message,pkt_data , nBytesThisPkt, &mrpt::utils::internal::free_fn_for_zmq, fd))
					throw std::runtime_error("[mrpt_send_to_zmq] Error in zmq_msg_init_data()");
				// Send:
				const int sent_size = zmq_msg_send (&message, zmq_socket, fd->do_free ? 0 : ZMQ_SNDMORE);
				if (0!=zmq_msg_close (&message))
					throw std::runtime_error("[mrpt_send_to_zmq] Error in zmq_msg_close()");
				if (sent_size!=static_cast<int>(nBytesThisPkt))
					throw std::runtime_error("[mrpt_send_to_zmq] Error in zmq_msg_send()");
			}
		}

		/** Users may normally call mrpt_recv_from_zmq() and mrpt_recv_from_zmq_into().
		 * This function just stores the received data into a memory buffer without parsing it into an MRPT object.
		 * \return false on any error */
		template <typename ZMQ_SOCKET_TYPE, typename VECTOR_MSG_T>
		bool mrpt_recv_from_zmq_buf(
			ZMQ_SOCKET_TYPE zmq_socket,
			VECTOR_MSG_T &out_lst_msgs,
			mrpt::utils::CMemoryStream &target_buf,
			bool dont_wait,
			size_t * rx_obj_length_in_bytes)
		{
			if (rx_obj_length_in_bytes) *rx_obj_length_in_bytes = 0;
			out_lst_msgs.clear();
			target_buf.Clear();
			int64_t more;
			size_t more_size = sizeof(more);
			do
			{
				// Init rx msg:
				zmq_msg_t *msg = new zmq_msg_t();
				if (0!=zmq_msg_init (msg))
					return false;
				out_lst_msgs.push_back(msg);
				// Recv:
				int rc = zmq_msg_recv(msg,zmq_socket, dont_wait ? ZMQ_DONTWAIT : 0);
				if (rc==-1) return false;
				// Determine if more message parts are to follow
				rc = zmq_getsockopt (zmq_socket, ZMQ_RCVMORE, &more, &more_size);
				if (rc!=0) return false;
				// Only one part?
				if (out_lst_msgs.size()==1 && !more) {
					target_buf.assignMemoryNotOwn(zmq_msg_data(msg), zmq_msg_size(msg));
					if (rx_obj_length_in_bytes) *rx_obj_length_in_bytes = zmq_msg_size(msg);
				}
			} while (more);
			// More than 1 part?
			if (out_lst_msgs.size()>1) {
				for (size_t i=0;i<out_lst_msgs.size();i++) {
					target_buf.WriteBuffer(zmq_msg_data(out_lst_msgs[i]),zmq_msg_size(out_lst_msgs[i]));
				}
				if (rx_obj_length_in_bytes) *rx_obj_length_in_bytes = target_buf.getTotalBytesCount();
				target_buf.Seek(0);
			}
			return true;
		}

		namespace internal {
			template <typename VECTOR_MSG_T>
			void free_zmq_msg_lst(VECTOR_MSG_T &lst_msgs)
			{
				for (size_t i=0;i<lst_msgs.size();++i) {
					zmq_msg_close (lst_msgs[i]);
					delete lst_msgs[i];
				}
			}
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
		 * \note See examples of usage in https://github.com/MRPT/mrpt/tree/master/doc/mrpt-zeromq-example
		 */
		template <typename ZMQ_SOCKET_TYPE>
		mrpt::utils::CSerializablePtr mrpt_recv_from_zmq(ZMQ_SOCKET_TYPE zmq_socket, bool dont_wait = false, size_t * rx_obj_length_in_bytes = NULL)
		{
			CMemoryStream	target_buf;
			mrpt::utils::CSerializablePtr obj;
			std::vector<zmq_msg_t*> lst_msgs_to_close;
			if (!mrpt_recv_from_zmq_buf(zmq_socket,lst_msgs_to_close, target_buf,dont_wait,rx_obj_length_in_bytes))
				return obj;
			// De-serialize:
			obj = target_buf.ReadObject();
			internal::free_zmq_msg_lst(lst_msgs_to_close); // Free msgs mem
			return obj;
		}
		/** Like mrpt_recv_from_zmq() but without dynamically allocating the received object,
		  * more efficient to use if the type of the received object is known in advance.
			* \param[in] target_object The received object will be stored here. An exception will be raised upon type mismatch.
			* \return true if all was OK, false on any ZMQ error.
			* \sa mrpt_recv_from_zmq() for details on the rest of parameters.
			* \note See examples of usage in https://github.com/MRPT/mrpt/tree/master/doc/mrpt-zeromq-example
		  */
		template <typename ZMQ_SOCKET_TYPE>
		bool mrpt_recv_from_zmq_into(ZMQ_SOCKET_TYPE zmq_socket, mrpt::utils::CSerializable &target_object, bool dont_wait = false, size_t * rx_obj_length_in_bytes = NULL)
		{
			CMemoryStream	target_buf;
			std::vector<zmq_msg_t*> lst_msgs_to_close;
			if (!mrpt_recv_from_zmq_buf(zmq_socket,lst_msgs_to_close, target_buf,dont_wait,rx_obj_length_in_bytes))
				return false;
			// De-serialize:
			target_buf.ReadObject(&target_object);
			internal::free_zmq_msg_lst(lst_msgs_to_close); // Free msgs mem
			return true;
		}

		/** @} */
	} // End of namespace
} // End of namespace
#endif

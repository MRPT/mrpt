/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/containers/MT_buffer.h>

#include <future>
#include <list>

namespace mrpt::hwdrivers
{
/** A client for NTRIP (HTTP) sources of differential GPS corrections from
 *internet servers, or Global navigation satellite system (GNSS) internet
 *radio.
 *  Usage:
 *		- To open the server, invoke "open" with the proper parameters. Then use
 *"stream_data" to read the read data.
 *		- To obtain a list of all the mountpoints available at a given NTRIP
 *Caster, call "retrieveListOfMountpoints" (it's a static method).
 *
 *  It is not neccesary to call "close", the connection is ended at
 *destruction.
 *
 * \note For a good reference of the NTRIP protocol, see
 *http://gnss.itacyl.es/opencms/opencms/system/modules/es.jcyl.ita.site.gnss/resources/documentos_gnss/NtripDocumentation.pdf
 * \ingroup mrpt_hwdrivers_grp
 *
 */
class CNTRIPClient
{
   public:
	/** A descriptor of one stream in an NTRIP Caster - See
	 * CNTRIPClient::retrieveListOfMountpoints
	 */
	struct TMountPoint
	{
		std::string mountpoint_name;
		/** City name */
		std::string id;
		/** RTCM 2.3, RTCM 3, CMR+, etc... */
		std::string format;
		std::string format_details;
		/** 0: No carrier phase, 1: L1, 2: L1+L2 */
		int carrier{0};
		/** GPS, ... */
		std::string nav_system;
		/** IGS, ... */
		std::string network;
		/** ITA, ESP, DEU,... */
		std::string country_code;
		double latitude{0}, longitude{0};
		bool needs_nmea{false};
		bool net_ref_stations{false};
		std::string generator_model;
		/** "none" */
		std::string compr_encryp;
		/** "N": none, "B": basic, "D": digest */
		char authentication{'B'};
		bool pay_service{false};
		int stream_bitspersec{0};
		std::string extra_info;

		TMountPoint() = default;
	};

	/** Used in CNTRIPClient::retrieveListOfMountpoints */
	using TListMountPoints = std::list<TMountPoint>;

	/**  The arguments for connecting to a NTRIP stream, used in
	 * CNTRIPClient::open
	 */
	struct NTRIPArgs
	{
		std::string server{"www.euref-ip.net"};
		int port{2101};
		std::string user;
		std::string password;
		std::string mountpoint;

		/** Default params */
		NTRIPArgs() = default;
	};

   protected:
	/** The working thread */
	void private_ntrip_thread();

	std::thread m_thread;
	std::promise<void> m_sem_sock_closed;
	std::promise<void> m_sem_first_connect_done;

	mutable bool m_thread_exit{false};
	/** Will be "true" between "open" and "close" */
	mutable bool m_thread_do_process{false};
	mutable bool m_waiting_answer_connection{false};

	enum TConnResult
	{
		connOk = 0,
		connError,
		connUnauthorized
	};

	mutable TConnResult m_answer_connection{connError};
	/** All the parameters for the NTRIP connection */
	mutable NTRIPArgs m_args;

	/** Buffer for data to be sent back to the server */
	mrpt::containers::MT_buffer m_upload_data;

   public:
	/** Default constructor */
	CNTRIPClient();
	/** Default destructor */
	virtual ~CNTRIPClient();

	/** Tries to open a given NTRIP stream and, if successful, launches a thread
	 * for continuously reading from it.
	 * \sa close, stream_data
	 *
	 * \return false On any kind of error, with a description of the error in
	 * errmsg, if provided.
	 */
	bool open(const NTRIPArgs& params, std::string& out_errmsg);

	/** Closes the connection.
	 * \sa open
	 */
	void close();

	/** The buffer with all the bytes so-far read from the NTRIP server stream.
	 * Call its "readAndClear" method in a timely fashion to get the stream
	 * contents.
	 * \sa open, close
	 */
	mrpt::containers::MT_buffer stream_data;

	/** Connect to a given NTRIP caster and get the list of all available
	 *mountpoints and their parameters.
	 *  Note that the authentication parameters "auth_user" and "auth_pass"
	 *will be left empty in most situations, since LISTING the Caster normally
	 *doesn't require special rights.
	 *
	 * Example:
	 * \code
	 *	 CNTRIPClient::TListMountPoints	lst;
	 *	 std::string errMsg;
	 *	 bool ret =
	 *CNTRIPClient::retrieveListOfMountpoints(lst,errMsg,"www.euref-ip.net",
	 *2101);
	 * \endcode
	 *
	 * \return False on any error, then "errmsg" holds the reason.
	 */
	static bool retrieveListOfMountpoints(
		TListMountPoints& out_list, std::string& out_errmsg,
		const std::string& server, int port = 2101,
		const std::string& auth_user = std::string(),
		const std::string& auth_pass = std::string());

	/** Enqueues a string to be sent back to the NTRIP server (e.g. GGA frames)
	 */
	void sendBackToServer(const std::string& data);

};  // End of class

}  // namespace mrpt::hwdrivers

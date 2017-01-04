/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// This file contains portions of code from sicklms200.cc from the Player/Stage project.

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/utils/crc.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/system/os.h>
#include <cstdio> // printf

#include <mrpt/hwdrivers/CCANBusReader.h>

IMPLEMENTS_GENERIC_SENSOR(CCANBusReader,mrpt::hwdrivers)

#define RET_ERROR(msg) { cout << "[" << __CURRENT_FUNCTION_NAME__ <<"] " << msg << endl; return false; }

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::hwdrivers;

char hexCharToInt(char n)
{
    if (n >= '0' && n <= '9')
        return (n-'0');
    else

    if (n >= 'A' && n <= 'F')
        return (n-'A'+10);
    else
        return 0;
}

/*-------------------------------------------------------------
						CCANBusReader
-------------------------------------------------------------*/
CCANBusReader::CCANBusReader() :
	mrpt::utils::COutputLogger("CCANBusReader"),
	m_com_port(),
	m_mySerialPort( NULL ),
	m_com_baudRate(57600),
	m_nTries_connect(1),
	m_nTries_current(0),
	m_canbus_speed(250000),
	m_canreader_timestamp(false),
	m_CANBusChannel_isOpen(false)
{
	m_sensorLabel = "CANBusReader";
	memset(m_received_frame_buffer,0,sizeof(m_received_frame_buffer));
}

/*-------------------------------------------------------------
						~CCANBusReader
-------------------------------------------------------------*/
CCANBusReader::~CCANBusReader()
{
    if( m_CANBusChannel_isOpen )
	{
		try
		{
			CANBusCloseChannel();
		}
		catch(...) {}
	}

	if (m_mySerialPort)
	{
		delete m_mySerialPort;
		m_mySerialPort = NULL;
	}
}

void  CCANBusReader::doProcess()
{
    mrpt::obs::CObservationCANBusJ1939Ptr	obs = mrpt::obs::CObservationCANBusJ1939::Create();
    bool thereIsObservation;
    bool hardwareError;

    doProcessSimple( thereIsObservation, *obs, hardwareError );
    if( thereIsObservation )
        appendObservation(obs);
    else
        cout << "No frame received" << endl;
}

/*-------------------------------------------------------------
						doProcess
-------------------------------------------------------------*/
void  CCANBusReader::doProcessSimple(
	bool							    &outThereIsObservation,
	mrpt::obs::CObservationCANBusJ1939	&outObservation,
	bool							    &hardwareError )
{
	outThereIsObservation	= false;
	hardwareError			= false;

	if ( !tryToOpenComms() )
	{
		hardwareError = true;
		return;
	}

	m_state = ssWorking;

	// Wait for a scan:
	uint8_t out_prio,out_pdu_format,out_pdu_spec,out_src_address,out_data_length;
	uint16_t out_pgn;
	vector<uint8_t> out_data;
	vector<char> out_raw_frame;
	if (!waitContinuousSampleFrame(
				out_prio,
				out_pdu_format,
				out_pdu_spec,
				out_src_address,
				out_data_length,
				out_pgn,
				out_data,
				out_raw_frame ))
		return;

	// Yes, we have a new scan:
	//    cout << "we've got a frame" << endl;
	// -----------------------------------------------
	//   Extract the observation:
	// -----------------------------------------------
	outObservation.timestamp        = mrpt::system::now();
	outObservation.sensorLabel      = m_sensorLabel;	// Set label

	// And the scan ranges:
	outObservation.m_priority       = out_prio;
	outObservation.m_pdu_spec       = out_pdu_spec;
	outObservation.m_pdu_format     = out_pdu_format;
	outObservation.m_src_address    = out_src_address;
	outObservation.m_pgn 	        = out_pgn;
	outObservation.m_data_length    = out_data_length;
	outObservation.m_data.resize( out_data.size() );
	for(uint8_t k = 0; k < out_data.size(); ++k)
		outObservation.m_data[k] = out_data[k];
	outObservation.m_raw_frame.resize( out_raw_frame.size() );
	for(uint8_t k = 0; k < out_raw_frame.size(); ++k)
		outObservation.m_raw_frame[k] = out_raw_frame[k];

	// we've got a new observation
	outThereIsObservation = true;
}

/*-------------------------------------------------------------
						loadConfig_sensorSpecific
-------------------------------------------------------------*/
void  CCANBusReader::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string	  &iniSection )
{
//	m_sensorPose = CPose3D(
//		configSource.read_float(iniSection,"pose_x",0),
//		configSource.read_float(iniSection,"pose_y",0),
//		configSource.read_float(iniSection,"pose_z",0),
//		DEG2RAD( configSource.read_float(iniSection,"pose_yaw",0) ),
//		DEG2RAD( configSource.read_float(iniSection,"pose_pitch",0) ),
//		DEG2RAD( configSource.read_float(iniSection,"pose_roll",0) )
//		);  // irrelevant

	m_canbus_speed          = configSource.read_int(iniSection,"CANBusSpeed",m_canbus_speed);
	m_canreader_timestamp   = configSource.read_bool(iniSection,"useCANReaderTimestamp",m_canreader_timestamp);

#ifdef MRPT_OS_WINDOWS
	m_com_port = configSource.read_string(iniSection, "COM_port_WIN", m_com_port, true );
#else
	m_com_port = configSource.read_string(iniSection, "COM_port_LIN", m_com_port, true );
#endif

    m_com_baudRate = configSource.read_int(iniSection, "COM_baudRate", m_com_baudRate );
    m_nTries_connect = configSource.read_int(iniSection, "nTries_connect", m_nTries_connect );
}

/*-------------------------------------------------------------
 Tries to open the com port and setup
 all the LMS protocol. Returns true if OK or already open.
-------------------------------------------------------------*/
bool CCANBusReader::tryToOpenComms(std::string *err_msg)
{
	if (err_msg) *err_msg="";
	try
	{
		if (!m_mySerialPort)
		{
			// There is no COMMS port open yet...
			if (!m_com_port.empty())
			{
//			    cout << "Creating port" << endl;
				m_mySerialPort = new CSerialPort(); 				// Create the port myself:
			}
			else
			throw std::logic_error("ERROR: No serial port attached with bindIO, neither it set with 'setSerialPort'");
		}

		// We assure now we have a stream... try to open it, if it's not done yet.
		bool just_open = false;
//		CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
		if (m_mySerialPort!=NULL)
		{
			if (!m_mySerialPort->isOpen())
			{
				// Try to open it now:
				m_mySerialPort->setSerialPortName(m_com_port);
				m_mySerialPort->open(); // will raise an exception on error.

				// Set basic params:
				m_mySerialPort->setConfig(9600);
				m_mySerialPort->setTimeouts(100,0,10,0,50);

				just_open = true;
			}
		}

		// It seems the port was open and working so we are done here.
		if (!just_open)
			return true;

		// ==================================================================
		// Otherwise, it was just opened now, we must send the
		//  ** initialization commands **
		// and put the CAN Converter in recording mode:
		// ==================================================================
		cout << "Setting up serial comms in port " << m_com_port;
		if (!setupSerialComms())  RET_ERROR("error");
        cout << " ... done" << endl;

		// initialize
        // set CAN Bus speed
        /**/
        bool res;
        cout << "Setting up CAN BUS Speed at: " << m_canbus_speed << endl;
        for (int nTry=0;nTry<250000/*4*/;nTry++)
            if (true == (res=sendCANBusReaderSpeed()))
                break;
        if(!res) return false;
        cout << " ... done" << endl;

        // open the CAN channel. If true, at this point, frames should be poping out the CAN Bus
        cout << "Opening CAN BUS and starting to receive." << endl;
        for (int nTry=0;nTry<250000/*4*/;nTry++)
            if (true==(res=CANBusOpenChannel()))
                break;
        if(!res) return false;
        cout << " ... done" << endl;

//        cout << "Autopoll" << endl;
//        for (int nTry=0;nTry<250000/*4*/;nTry++)
//            if (true==(res=CANBusAutoPoll()))
//                break;
//        if(!res) return false;
//        cout << " ... done" << endl;

		return res;
		/**/
	}
	catch(std::exception &e)
	{
		std::string s = "[CCANBusReader] Error trying to open CANBusReader at port ";
		s+= e.what();
		if (err_msg) *err_msg=s;
		MRPT_LOG_ERROR_STREAM << s;
		return false;
	}
}

/*-------------------------------------------------------------
 Tries to send the command to set up the speed of the
 CAN Bus reader -> tractor link
-------------------------------------------------------------*/
bool CCANBusReader::sendCANBusReaderSpeed()
{
    // command: S0 --> S8 according to the selected speed
    unsigned char cmd[2];

    cmd[0] = 'S';
    switch( m_canbus_speed )
    {
        case 10000:     cmd[1] = '0'; break;
        case 20000:     cmd[1] = '1'; break;
        case 50000:     cmd[1] = '2'; break;
        case 100000:    cmd[1] = '3'; break;
        case 125000:    cmd[1] = '4'; break;
        case 250000:    cmd[1] = '5'; break;
        case 500000:    cmd[1] = '6'; break;
        case 800000:    cmd[1] = '7'; break;
        case 1000000:   cmd[1] = '8'; break;
        default: RET_ERROR("Incorrect CAN Bus speed"); break;
    }
    sendCommandToCANReader(cmd,2);
    return waitACK(50);
}

bool CCANBusReader::CANBusOpenChannel()
{
    unsigned char cmd[1];
    cmd[0] = 'O';
    sendCommandToCANReader(cmd,1);
    m_CANBusChannel_isOpen = waitACK(50);
    return m_CANBusChannel_isOpen;
}

bool CCANBusReader::CANBusCloseChannel()
{
    unsigned char cmd[1];
    cmd[0] = 'C';
    sendCommandToCANReader(cmd,1);
//    m_CANBusChannel_isOpen = !waitACK(50);
    m_CANBusChannel_isOpen = false;
//    return !m_CANBusChannel_isOpen;
    return true;
}

bool CCANBusReader::CANBusAutoPoll()
{
    unsigned char cmd[1];
    cmd[0] = 'A';
    sendCommandToCANReader(cmd,1);
    return waitACK(50);
}

bool CCANBusReader::CANBusX1()
{
    unsigned char cmd[2];
    cmd[0] = 'X';
    cmd[1] = '1';
    sendCommandToCANReader(cmd,2);
    return waitACK(50);
}

bool CCANBusReader::CANBusPoll()
{
    unsigned char cmd[1];
    cmd[0] = 'P';
    sendCommandToCANReader(cmd,1);
    return waitACK(50);
}

/*-------------------------------------------------------------
					waitContinuousSampleFrame
-------------------------------------------------------------*/
bool CCANBusReader::waitContinuousSampleFrame(
    uint8_t &out_prio,
    uint8_t &out_pdu_format,
    uint8_t &out_pdu_spec,
    uint8_t &out_src_address,
    uint8_t &out_data_length,
    uint16_t &out_pgn,
    vector<uint8_t> &out_data,
    vector<char> &out_raw_frame)
{
//	CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
//	ASSERTMSG_(COM!=NULL,"No I/O channel bound to this object");

	size_t 	nRead,nBytesToRead;
	size_t	nFrameBytes = 0;
	size_t	lengthField;
	unsigned char	buf[40];

    // clear buffer
	for(uint8_t k = 0; k < 40; ++k) buf[k]=0;

	uint8_t dlc = 0;
    while( nFrameBytes < (lengthField=(10U+dlc+1U) )  )
	{
//	    cout << "trying to receive" << endl;
		if(lengthField > 30)
		{
			cout << "#" << int(dlc) << " ";
			nFrameBytes = 0;	// No es cabecera de trama correcta
            for(uint8_t k = 0; k < 40; ++k) buf[k]=0;
            dlc = 0;
		}

		if (nFrameBytes < 10)
			nBytesToRead = 1;
		else
		{
            dlc = 2*uint8_t(hexCharToInt(buf[9]));
//		    cout << "dlc: " << int(dlc) << endl;
		    nBytesToRead = (lengthField) - nFrameBytes;
		}

		try
		{
			nRead = m_mySerialPort->Read( buf+nFrameBytes,nBytesToRead );
//			cout << "to read: " << nBytesToRead << " received: " << nRead << " -> ";
//			for( uint8_t k = 0; k < nRead; ++k )
//                cout << int(buf[k+nFrameBytes]);
//            cout << endl;
		}
		catch (std::exception &e)
		{
			// Disconnected?
			MRPT_LOG_ERROR_STREAM << "[waitContinuousSampleFrame] Disconnecting due to comms error: " << e.what();
			return false;
		}

		if ( !nRead )
			return false;

		if (nRead<nBytesToRead)
			mrpt::system::sleep(30);

		// Reading OK:
		// Was it the first one?
		if (nFrameBytes>0 || (nFrameBytes == 0 && buf[0]==0x54 /*T*/))
            nFrameBytes+=nRead;
		else
		{
			nFrameBytes = 0;	// No es cabecera de trama correcta
			for(uint8_t k = 0; k < 40; ++k) buf[k]=0;
		}
	} // end while

    // Process frame
    // convert ASCII text into integer
    vector<uint8_t> aux;
    out_raw_frame.resize(nFrameBytes);
    for(uint8_t k = 0; k < nFrameBytes; ++k)
    {
        aux.push_back( hexCharToInt(buf[k]) );
        out_raw_frame[k] = buf[k];
    }

    out_prio        = (aux[1] << 2) | (aux[2] >> 2);
    out_pdu_format  = (aux[3] << 4) | aux[4];
    out_pdu_spec    = (aux[5] << 4) | aux[6];
    out_src_address = (aux[7] << 4) | aux[8];
    out_data_length = aux[9];
    out_pgn         = uint16_t(out_pdu_format) << 8 | uint16_t(out_pdu_spec);
    out_data.resize(out_data_length);
    for(uint8_t k = 0, k2 = 0; k < 2*out_data_length; k+=2, k2++)
        out_data[k2] = (aux[10+k] << 4) | aux[11+k];

    if( buf[nFrameBytes-1] != 0x0D )
    {
        cout << format("[CCANBusReader::waitContinuousSampleFrame] expected 0x0D ending flag, 0x%X found instead", buf[nFrameBytes]) << endl;
		return false; // Bad ending flag
    }

	// All OK
	return true;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
void CCANBusReader::initialize()
{
	string err_str;
	memset(m_received_frame_buffer,0,sizeof(m_received_frame_buffer));
	if (!tryToOpenComms(&err_str))
    {
        cerr << err_str << endl;
		throw std::logic_error(err_str);
    }
}

/*-----------------------------------------------------------------
	Assures laser is connected and operating at 38400, in
	 its case returns true.
  -----------------------------------------------------------------*/
bool CCANBusReader::setupSerialComms()
{
    ASSERT_(m_com_baudRate==9600 || m_com_baudRate==38400 || m_com_baudRate == 57600 || m_com_baudRate==500000);

//	CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
	if (m_mySerialPort==NULL) return true;

    int detected_rate = 0;
	for (size_t reps=0;!detected_rate && reps<m_nTries_connect;reps++)
	{
        m_nTries_current=reps;

        int rates[] = {0, 9600,38400,57600,500000};

        // Try first the desired rate to speed up the process, just in case
        //  the converter is already setup from a previous run:
        rates[0] = m_com_baudRate;

        detected_rate = 0;

        for (size_t i=0;!detected_rate && i<sizeof(rates)/sizeof(rates[0]);i++)
        {
            // Are we already receiving at 500k?
            // ------------------------------------------------
            m_mySerialPort->setConfig( rates[i] );
            mrpt::system::sleep(100);
            m_mySerialPort->purgeBuffers();

            // close the connection
            /**/
            cout << endl << "Closing CAN Channel " << endl;
            for (int nTry=0;nTry<250000/*4*/;nTry++)
                if( true==CANBusCloseChannel() )
                    break;
            cout << " ... done" << endl;
            /**/

            mrpt::system::sleep(100);
            m_mySerialPort->purgeBuffers();

            for (int nTry=0;nTry<250000/*4*/ && !detected_rate;nTry++)
            {
                m_mySerialPort->purgeBuffers();

                // Ask for the laser version at the current rate:
                if( queryVersion(true) )
                {
                    detected_rate = rates[i];
                    break;
                }
                mrpt::system::sleep(20);
            } // for tries
            // There is no link, or the baudrate is wrong...
        }

        // Try again in a while:
        if (!detected_rate && reps!=(m_nTries_connect-1))
            mrpt::system::sleep(5000);
	}

	// Switch "this" serial port to the detected baudrate
    setBaudRate( detected_rate );

	m_mySerialPort->setConfig(m_com_baudRate);
	m_mySerialPort->purgeBuffers();

	// Wait...
	mrpt::system::sleep(500);

	// And check comms at the new baud rate:
	return true;
}

/*-----------------------------------------------------------------
  Query to LMS a status query.
   Returns true if response is read ok.
  -----------------------------------------------------------------*/
bool CCANBusReader::queryVersion(bool printOutVersion)
{
	ASSERT_(m_mySerialPort);

	uint8_t cmd[1];
	cmd[0] = 'V';
    uint16_t cmd_len = 1;

    if (!sendCommandToCANReader(cmd,cmd_len,false)) return false;
	return waitForVersion(500, printOutVersion);
}

// Returns false if timeout
bool CCANBusReader::waitACK(uint16_t timeout_ms)
{
//	CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
//	ASSERT_(COM);

	uint8_t b = 0;
	CTicTac tictac;
	tictac.Tic();

	do
	{
		if ( m_mySerialPort->Read(&b,1) )
		{
		    // Byte rx:
			if(b==0x0D/*0x30*/)
			{
			    cout << int(b) << endl;
			    return true;       // [CR]
			}

		}
	} while(tictac.Tac()<timeout_ms*1e-3 );

	if (b==0x07)                            // [BELL]
        RET_ERROR(format("ERROR received."))
	else if (b!=0)
         RET_ERROR(format("Unexpected code received: 0x%02X",b))
    else
        return false; //RET_ERROR("Timeout")
}

bool CCANBusReader::waitForVersion(uint16_t timeout, bool printOutVersion )
{
//    CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
//	ASSERT_(COM);

	uint8_t b;
	unsigned int nBytes=0;

	CTicTac tictac;
	tictac.Tic();
	const double maxTime = timeout*1e-3;

	while( nBytes<6 )
	{
		if ( m_mySerialPort->Read(&b,1) )
		{
//		    cout << "received " << nBytes << " bytes: " << char(b) << endl;
			// First byte must be STX:
			if ( nBytes > 0 || (nBytes == 0 && b == 'V') )
			{
				// Store in frame:
				m_received_frame_buffer[nBytes] = b;
				nBytes++;
			}
		}
		if (tictac.Tac()>=maxTime)
		{
		    cout << "Version timeout" << endl;
		    return false;	// Timeout
		}
	}

	// Check len:
	if (m_received_frame_buffer[nBytes-1] != 0x0D )
	{
        printf("[CCANBusReader::waitForVersion] Error: expected 0x0D final byte, received %x\n", m_received_frame_buffer[nBytes-1]);
        return false;
	}

	if( printOutVersion )
	{
	    cout << "Version: ";
        for(uint8_t k = 0; k < nBytes; ++k)
            cout << char(m_received_frame_buffer[k]);
        cout << endl;
	}
	return true;
}
// Returns false if timeout
bool CCANBusReader::waitIncomingFrame(uint16_t timeout)
{
//	CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
//	ASSERT_(COM);

	uint8_t b;
	unsigned int nBytes=0;

	CTicTac tictac;
	tictac.Tic();
	const double maxTime = timeout*1e-3;
    uint8_t dlc = 0;
	while( nBytes<10 || (nBytes<(10U+dlc+1U/*CR*/)) )
	{
		if(m_mySerialPort->Read(&b,1) )
		{
			// First byte must be STX:
			if( nBytes>1 || (!nBytes && b==0x54 /*'T'*/) )
			{
				// Store in frame:
				m_received_frame_buffer[nBytes] = b;
				nBytes++;
			}
			if( nBytes == 10 )
                dlc = 2*uint8_t(hexCharToInt(m_received_frame_buffer[9]));   // here is the number of BYTES of data -> 2 hex values for byte
		}
		if (tictac.Tac()>=maxTime)
			return false;	// Timeout
	}
    // Check final flag
	if (m_received_frame_buffer[10U+dlc] != 0x0D)
	{
        printf("[CCANBusReader::waitIncomingFrame] Error: expected 0x0D as final flag, received %x\n",m_received_frame_buffer[10U+dlc]);
        return false;
	}

#if 0
    printf("RX: ");
    for (unsigned int i=0;i<nBytes;i++)
        printf("%02X ",m_received_frame_buffer[i]);
    printf("\n");
#endif

	// OK
	return true;
}

bool CCANBusReader::sendCommandToCANReader(const uint8_t *cmd,const uint16_t cmd_len, bool wait)
{
	MRPT_UNUSED_PARAM(wait);
    uint8_t cmd_full[1024];
    ASSERT_(sizeof(cmd_full)>cmd_len);

//	CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
//	ASSERT_(COM);

	// command is just plain text so no frame header nor CRC is needed
	memcpy(cmd_full,cmd,cmd_len);
	cmd_full[cmd_len] = 0x0D;       // [CR] at the end

    const size_t toWrite = cmd_len+1;

#if 1
    printf("TX: ");
    for (unsigned int i=0;i<toWrite;i++)
        printf("%02X ",cmd_full[i]);
    printf("\n");
#endif

//    const int NTRIES = 3;

//    for (int k=0;k<NTRIES;k++)
//    {
        if (toWrite!=m_mySerialPort->Write( cmd_full, toWrite ))
        {
            cout << "[CCANBusReader::SendCommandToCANReader] Error writing data to serial port." << endl;
            return false;
        }
        return true;
//        mrpt::system::sleep(15);
//        if(wait)
//        {
//            if(waitACK(5000))
//                return true;
//            mrpt::system::sleep(10);
//        }
//        else
//            return true; // perform special wait outside this method
//    }

    return false;
}


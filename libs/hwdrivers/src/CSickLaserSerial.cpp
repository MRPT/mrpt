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

#include <mrpt/hwdrivers/CSickLaserSerial.h>


IMPLEMENTS_GENERIC_SENSOR(CSickLaserSerial,mrpt::hwdrivers)

#define RET_ERROR(msg) { cout << "[" << __CURRENT_FUNCTION_NAME__ <<"] " << msg << endl; return false; }

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::hwdrivers;


int CSickLaserSerial::CRC16_GEN_POL = 0x8005;

/*-------------------------------------------------------------
						CSickLaserSerial
-------------------------------------------------------------*/
CSickLaserSerial::CSickLaserSerial() :
	m_mm_mode( false ),
	m_scans_FOV(180),
	m_scans_res(50),
	m_com_port(),
	m_mySerialPort( NULL ),
	m_com_baudRate(38400),
	m_nTries_connect(1),
	m_nTries_current(0),
	m_skip_laser_config(false)
{
	m_sensorLabel = "SICKLMS";
	memset(m_received_frame_buffer,0,sizeof(m_received_frame_buffer));
}

/*-------------------------------------------------------------
						~CSickLaserSerial
-------------------------------------------------------------*/
CSickLaserSerial::~CSickLaserSerial()
{
	if (m_stream)
	{
		try
		{
            if (!m_skip_laser_config)
		    {
                LMS_endContinuousMode();
		    }
		}
		catch(...) {}
	}

	if (m_mySerialPort)
	{
		delete m_mySerialPort;
		m_mySerialPort = NULL;
	}
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
void  CSickLaserSerial::doProcessSimple(
	bool							&outThereIsObservation,
	mrpt::obs::CObservation2DRangeScan	&outObservation,
	bool							&hardwareError )
{
	outThereIsObservation	= false;
	hardwareError			= false;

	if ( !tryToOpenComms() )
	{
		hardwareError = true;
		return;
	}

	vector<float> 	ranges;
	unsigned char	LMS_stat;
	bool			is_mm_mode;

    m_state = ssWorking;

	// Wait for a scan:
	if (!waitContinuousSampleFrame( ranges,LMS_stat, is_mm_mode ))
		return;

	// Yes, we have a new scan:

	// -----------------------------------------------
	//   Extract the observation:
	// -----------------------------------------------
	outObservation.timestamp = mrpt::system::now();

	outObservation.sensorLabel  = m_sensorLabel;	// Set label

	// Extract the timestamp of the sensor:

	// And the scan ranges:
	outObservation.rightToLeft = true;
	outObservation.aperture = M_PIf;
	outObservation.maxRange	= is_mm_mode ? 32.7 : 81.0;
	outObservation.stdError = 0.003f;
	outObservation.sensorPose = m_sensorPose;

	outObservation.resizeScan(ranges.size());

	for (size_t i=0;i<ranges.size();i++) {
		outObservation.setScanRange( i, ranges[i] );
		outObservation.setScanRangeValidity(i,  (outObservation.scan[i] <= outObservation.maxRange) );
	}

	// Do filter:
	C2DRangeFinderAbstract::filterByExclusionAreas( outObservation );
	C2DRangeFinderAbstract::filterByExclusionAngles( outObservation );
	// Do show preview:
	C2DRangeFinderAbstract::processPreview(outObservation);

	outThereIsObservation = true;
}

/*-------------------------------------------------------------
						loadConfig_sensorSpecific
-------------------------------------------------------------*/
void  CSickLaserSerial::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string	  &iniSection )
{
	m_sensorPose = CPose3D(
		configSource.read_float(iniSection,"pose_x",0),
		configSource.read_float(iniSection,"pose_y",0),
		configSource.read_float(iniSection,"pose_z",0),
		DEG2RAD( configSource.read_float(iniSection,"pose_yaw",0) ),
		DEG2RAD( configSource.read_float(iniSection,"pose_pitch",0) ),
		DEG2RAD( configSource.read_float(iniSection,"pose_roll",0) )
		);

	m_mm_mode = configSource.read_bool(iniSection,"mm_mode",m_mm_mode);

#ifdef MRPT_OS_WINDOWS
	m_com_port = configSource.read_string(iniSection, "COM_port_WIN", m_com_port, true );
#else
	m_com_port = configSource.read_string(iniSection, "COM_port_LIN", m_com_port, true );
#endif

    m_com_baudRate = configSource.read_int(iniSection, "COM_baudRate", m_com_baudRate );
    m_nTries_connect = configSource.read_int(iniSection, "nTries_connect", m_nTries_connect );

    m_scans_FOV = configSource.read_int(iniSection, "FOV", m_scans_FOV );
    m_scans_res = configSource.read_int(iniSection, "resolution", m_scans_res );

    m_skip_laser_config = configSource.read_bool(iniSection, "skip_laser_config", m_skip_laser_config );

	// Parent options:
	C2DRangeFinderAbstract::loadCommonParams(configSource, iniSection);
}

/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CSickLaserSerial::turnOn()
{
	return true;
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CSickLaserSerial::turnOff()
{
	return true;
}

/*-------------------------------------------------------------
 Tries to open the com port and setup
 all the LMS protocol. Returns true if OK or already open.
-------------------------------------------------------------*/
bool CSickLaserSerial::tryToOpenComms(std::string *err_msg)
{
	if (err_msg) *err_msg="";
	try
	{
		if (!m_stream)
		{
			ASSERT_(m_mySerialPort==NULL);

			// There is no COMMS port open yet...
			if (!m_com_port.empty())
			{
				// Create the port myself:
				m_mySerialPort = new CSerialPort();
				m_stream = m_mySerialPort;
			}
			else
			throw std::logic_error("ERROR: No serial port attached with bindIO, neither it set with 'setSerialPort'");
		}

		// We assure now we have a stream... try to open it, if it's not done yet.
		bool just_open = false;
		CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
		if (COM!=NULL)
		{
			if (!COM->isOpen())
			{
				// Try to open it now:
				COM->setSerialPortName(m_com_port);
				COM->open(); // will raise an exception on error.

				// Set basic params:
				COM->setConfig(9600);
				COM->setTimeouts(100,0,10,0,50);

				just_open = true;
			}
		}

		// It seems the port was open and working so we are done here.
		if (!just_open)
			return true;

		// ==================================================================
		// Otherwise, it was just opened now, we must send the
		//  ** initialization commands **
		// and put the laser in continuous measuring mode:
		// ==================================================================
		if (!m_skip_laser_config)
		{
            if (!LMS_setupSerialComms())  RET_ERROR("error");

            bool res;
            for (int nTry=0;nTry<4;nTry++)
                if (true==(res=LMS_sendMeasuringMode_cm_mm()))
                    break;

            if (!res) return false;

            for (int nTry=0;nTry<4;nTry++)
                if (true==(res=LMS_startContinuousMode()))
                    break;

            return res;
		}
		else
		{
		    // Skip setup:
		    return true;
		}
	}
	catch(std::exception &e)
	{
		std::string s = "[CSickLaserSerial] Error trying to open SICK at port ";
		s+= e.what();
		if (err_msg) *err_msg=s;
		MRPT_LOG_ERROR(s);
		return false;
	}
}

/*-------------------------------------------------------------
					waitContinuousSampleFrame
-------------------------------------------------------------*/
bool  CSickLaserSerial::waitContinuousSampleFrame(
	vector<float> 	&out_ranges_meters,
	unsigned char 	&LMS_status,
    bool 			&is_mm_mode )
{
	CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
	ASSERTMSG_(COM!=NULL,"No I/O channel bound to this object");

	size_t 	nRead,nBytesToRead;
	size_t	nFrameBytes = 0;
	size_t	lengthField;
	unsigned char	buf[2000];
	buf[2]=buf[3]=buf[4]=0;

	while ( nFrameBytes < (lengthField=( 6 + (buf[2] | (buf[3] << 8))) )  )
	{
		if (lengthField>800)
		{
			cout << "#";
			nFrameBytes = 0;	// No es cabecera de trama correcta
			buf[2]=buf[3]=0;
		}

		if (nFrameBytes<4)
			nBytesToRead = 1;
		else
			nBytesToRead = (lengthField) - nFrameBytes;

		try
		{
			nRead = COM->Read( buf+nFrameBytes,nBytesToRead );
		}
		catch (std::exception &e)
		{
			// Disconnected?
			MRPT_LOG_ERROR_FMT("[CSickLaserSerial::waitContinuousSampleFrame] Disconnecting due to comms error: %s\n", e.what());
			//m_usbConnection->Close();
			return false;
		}

		if ( !nRead && !nFrameBytes )
			return false;

		if (nRead<nBytesToRead)
			mrpt::system::sleep(1);

		// Lectura OK:
		// Era la primera?
		if (nFrameBytes>1 || (!nFrameBytes && buf[0]==0x02) || (nFrameBytes==1 && buf[1]==0x80))
		{
			nFrameBytes+=nRead;
		}
		else
		{
			nFrameBytes = 0;	// No es cabecera de trama correcta
			buf[2]=buf[3]=0;
			//cerr << "."; //"[CSickLaserSerial] Skipping non-header..." << endl;
		}
	}

	// Frame received
	// --------------------------------------------------------------------------
	// | STX | ADDR | L1 | L2 | COM | INF1 | INF2 |	DATA	| STA | CRC1 | CRC2 |
	// --------------------------------------------------------------------------

	// Trama completa:
	//  Checkear que el byte de comando es 0xB0:
	if ( buf[4]!=0xB0 )	return false;

	// GET FRAME INFO
	int  info	 = buf[5] | (buf[6] << 8);	// Little Endian
	int  n_points = info & 0x01FF;
	is_mm_mode = 0 != ((info & 0xC000) >> 14);	// 0x00: cm 0x01: mm

	out_ranges_meters.resize(n_points);

	// Copiar rangos:
	short mask = is_mm_mode ? 0x7FFF : 0x1FFF;
	float meters_scale = is_mm_mode ? 0.001f : 0.01f;

	for (int i=0;i<n_points;i++)
		out_ranges_meters[i] = ( (buf[7+i*2] | (buf[8+i*2] << 8)) & mask ) * meters_scale;

	// Status
	LMS_status = buf[lengthField-3];

	// CRC:
	uint16_t CRC = mrpt::utils::compute_CRC16(buf,lengthField-2,  CRC16_GEN_POL);
	uint16_t CRC_packet = buf[lengthField-2] | ( buf[lengthField-1] << 8);
	if (CRC_packet!=CRC)
	{
		cerr << format("[CSickLaserSerial::waitContinuousSampleFrame] bad CRC len=%u nptns=%u: %i != %i", unsigned(lengthField),unsigned(n_points), CRC_packet, CRC) << endl;
		return false; // Bad CRC
	}

	// All OK
	return true;
}


/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
void CSickLaserSerial::initialize()
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
bool CSickLaserSerial::LMS_setupSerialComms()
{
    ASSERT_(m_com_baudRate==9600 || m_com_baudRate==38400 || m_com_baudRate==500000);

	CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
	if (COM==NULL) return true;

    int detected_rate = 0;
	for (size_t reps=0;!detected_rate && reps<m_nTries_connect;reps++)
	{
        m_nTries_current=reps;

        int rates[] = {0, 9600,38400,500000};

        // Try first the desired rate to speed up the process, just in case
        //  the laser is already setup from a previous run:
        rates[0] = m_com_baudRate;

        detected_rate = 0;

        for (size_t i=0;!detected_rate && i<sizeof(rates)/sizeof(rates[0]);i++)
        {
            // Are we already receiving at 500k?
            // ------------------------------------------------
            COM->setConfig( rates[i] );

            LMS_endContinuousMode(); // Stop continuous mode.
            mrpt::system::sleep(100);
            COM->purgeBuffers();

            for (int nTry=0;nTry<4 && !detected_rate;nTry++)
            {
                COM->purgeBuffers();
                // Ask for the laser status at the current rate:
                if ( LMS_statusQuery() )
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

	// Are we connected at the right rate?
	if (detected_rate==m_com_baudRate)
        return true;

	// Switch to the desired rate now
	if ( !this->LMS_setupBaudrate(m_com_baudRate) )	RET_ERROR("error");

	// Check response is OK:
	if (!(m_received_frame_buffer[2]==0x03 && m_received_frame_buffer[4]==0xA0 && m_received_frame_buffer[6]==0x10))
		return false;

	COM->setConfig(m_com_baudRate);
	COM->purgeBuffers();

	// Wait...
	mrpt::system::sleep(500);

	// And check comms at the new baud rate:
	return LMS_statusQuery();
}


/*-----------------------------------------------------------------
 Query to LMS a baudrate change command.
   Returns true if response is read ok.
  -----------------------------------------------------------------*/
bool CSickLaserSerial::LMS_setupBaudrate(int baud)
{
	ASSERT_(m_stream);

	uint8_t cmd[4];
	cmd[0] = 0x20;
	switch (baud)
	{
		case 9600:  cmd[1]=0x42; break;
		case 19200:	cmd[1]=0x41; break;
		case 38400:	cmd[1]=0x40; break;
		case 500000:cmd[1]=0x48; break;
		default:
			THROW_EXCEPTION("Invalid baud rate value");
	}

	uint16_t cmd_len = 2;

    if (!SendCommandToSICK(cmd,cmd_len)) return false;
	return LMS_waitIncomingFrame(500);
}


/*-----------------------------------------------------------------
  Query to LMS a status query.
   Returns true if response is read ok.
  -----------------------------------------------------------------*/
bool CSickLaserSerial::LMS_statusQuery()
{
	ASSERT_(m_stream);

	uint8_t cmd[1];
	cmd[0] = 0x31;
    uint16_t cmd_len = 1;

    if (!SendCommandToSICK(cmd,cmd_len)) return false;
	return LMS_waitIncomingFrame(500);
}


// Returns false if timeout
bool CSickLaserSerial::LMS_waitACK(uint16_t timeout_ms)
{
	CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
	ASSERT_(COM);

	uint8_t b = 0;
	CTicTac  tictac;
	tictac.Tic();

	do
	{
		if ( COM->Read(&b,1) )
		{	// Byte rx:
			if (b==0x06) return true;
		}
	}
	while ( tictac.Tac()< timeout_ms*1e-3  );

	if (b==0x15)
        RET_ERROR(format("NACK received."))
	else if (b!=0)
         RET_ERROR(format("Unexpected code received: 0x%02X",b))
    else return false; //RET_ERROR("Timeout")
}


// Returns false if timeout
bool CSickLaserSerial::LMS_waitIncomingFrame(uint16_t timeout)
{
	CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
	ASSERT_(COM);

	uint8_t b;
	unsigned int nBytes=0;

	CTicTac tictac;
	tictac.Tic();
	const double maxTime = timeout*1e-3;

	while (nBytes<6 || (nBytes<(6U+m_received_frame_buffer[2]+(uint16_t)(m_received_frame_buffer[3]<<8))) )
	{
		if ( COM->Read(&b,1) )
		{
			// First byte must be STX:
			if (nBytes>1 || (!nBytes && b==0x02) || (nBytes==1 && b==0x80))
			{
				// Store in frame:
				m_received_frame_buffer[nBytes] = b;
				nBytes++;
			}
		}
		if (tictac.Tac()>=maxTime)
			return false;	// Timeout
	}

    const uint16_t lengthField = m_received_frame_buffer[2] + (m_received_frame_buffer[3]<<8);
	// Check len:
	if (4U+lengthField+2U != nBytes)
	{
        printf("[CSickLaserSerial::LMS_waitIncomingFrame] Error: expected %u bytes, received %u\n",4U+lengthField+2U, nBytes);
        return false;
	}

	// Check CRC
    uint16_t CRC = mrpt::utils::compute_CRC16(m_received_frame_buffer,4+lengthField,  CRC16_GEN_POL);
    uint16_t CRC_packet = m_received_frame_buffer[4+lengthField+0] | ( m_received_frame_buffer[4+lengthField+1] << 8);
    if (CRC_packet!=CRC)
    {
        printf("[CSickLaserSerial::LMS_waitIncomingFrame] Error in CRC: rx: 0x%04X, computed: 0x%04X\n",CRC_packet,CRC);
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


bool CSickLaserSerial::LMS_sendMeasuringMode_cm_mm()
{
	CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
	ASSERT_(COM);

	// **************************
	// Send command: Switch to Installation mode
	// **************************
	uint8_t cmd[128]; // = {0x02,0x00,0x0A,0x00,0x20,0x00,0x53,0x49,0x43,0x4B,0x5F,0x4C,0x4D,0x53,0xBE,0xC5};
    cmd[0] = 0x20; /* mode change command */
    cmd[1] = 0x00; /* configuration mode */
    cmd[2] = 0x53; // S - the password
    cmd[3] = 0x49; // I
    cmd[4] = 0x43; // C
    cmd[5] = 0x4B; // K
    cmd[6] = 0x5F; // _
    cmd[7] = 0x4C; // L
    cmd[8] = 0x4D; // M
    cmd[9] = 0x53; // S

    uint16_t cmd_len = 10;
    if (!SendCommandToSICK(cmd,cmd_len)) RET_ERROR("Error waiting ACK to installation mode");
	if (!LMS_waitIncomingFrame(500)) RET_ERROR("Error in response to installation mode");

	// Check response
	if(!(m_received_frame_buffer[4] == 0xA0 && m_received_frame_buffer[5] == 0x00) )
		RET_ERROR("Wrong response to installation mode");

	// **************************
	// Request LMS Configuration
	// **************************
	cmd[0] = 0x74;
	cmd_len = 1;

    if (!SendCommandToSICK(cmd,cmd_len)) RET_ERROR("No ACK to 0x74 (req. config)");
	if (!LMS_waitIncomingFrame(500)) RET_ERROR("No answer to 0x74 (req. config)");

	// 2. Check response
	if(m_received_frame_buffer[4] != 0xF4)
		RET_ERROR("No expected 0xF4 in response to 0x74 (req. config)");

	// ***********************************************************************
	// Configure 1/2: Measuremente Range, Measurement Unit, Master/Slave Role
	// ***********************************************************************
	// 4.a Modify some values...

	// Measuring mode: Measurement range 32m in mm mode, or 80m+reflectance info in cm mode.
	// See page 98 in LMS2xx_list_datagrams.pdf.
	m_received_frame_buffer[10] = this->m_mm_mode ? 0x06 : 0x00;
	m_received_frame_buffer[11] = this->m_mm_mode ? 0x01 : 0x00;

	// 4.2 Build the output command
	m_received_frame_buffer[1]	= 0x00;		// Address
	m_received_frame_buffer[2]	= 0x23;		// Length (low byte)
	m_received_frame_buffer[3]	= 0x00;		// Length (high byte)
	m_received_frame_buffer[4]	= 0x77;		// Configure command

	memcpy(cmd, m_received_frame_buffer+4, 0x23);
	cmd_len = 0x23;

	// 4.4 Send to the LMS
    if (!SendCommandToSICK(cmd,cmd_len)) RET_ERROR("No ACK for config command (0x77)");
	if(!LMS_waitIncomingFrame(600)) RET_ERROR("No answer for config command (0x77)");

	if(!(m_received_frame_buffer[4] == 0xF7 && m_received_frame_buffer[5] == 0x01))
		RET_ERROR("Wrong answer for config command (0x77)");

	// **************************
	// Switch to Monitoring mode
	// **************************
	cmd[0] = 0x20;
	cmd[1] = 0x25;
	cmd_len = 2;
    if (!SendCommandToSICK(cmd,cmd_len)) RET_ERROR("No ACK for set monitoring mode");
	if (!LMS_waitIncomingFrame(500)) RET_ERROR("No answer for set monitoring mode");

	if(!(m_received_frame_buffer[4] == 0xA0 && m_received_frame_buffer[5] == 0x00) )
		RET_ERROR("Wrong answer for set monitoring mode");

	// All ok.
	return true;
}

/*-----------------------------------------------------------------
  Start continuous mode measurements.
   Returns true if response is read ok.
  -----------------------------------------------------------------*/
bool CSickLaserSerial::LMS_startContinuousMode()
{
	ASSERT_(m_scans_FOV==100 || m_scans_FOV==180);
	ASSERT_(m_scans_res==25 || m_scans_res==50 || m_scans_res==100);

	CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
	ASSERT_(COM);

    uint8_t cmd[40];

    // Config angle/resolution
    cmd[0] = 0x3B;
    cmd[1] = m_scans_FOV;
    cmd[2] = 0x00;
    cmd[3] = m_scans_res; // 25,50 or 100 -  1/100th of deg
    cmd[4] = 0x00;
    uint16_t cmd_len = 5;
    if (!SendCommandToSICK(cmd,cmd_len)) RET_ERROR("Error waiting ack for change angle/resolution");
    if (!LMS_waitIncomingFrame(500)) RET_ERROR("Error waiting answer for change angle/resolution");

    // Start continuous mode:
    cmd[0] = 0x20;
    cmd[1] = 0x24;
    cmd_len = 2;
    if (!SendCommandToSICK(cmd,cmd_len)) RET_ERROR("Error waiting ack for start scanning");
    if (!LMS_waitIncomingFrame(500)) RET_ERROR("Error waiting answer for start scanning");

    return true;
}

bool CSickLaserSerial::LMS_endContinuousMode()
{
	CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
	ASSERT_(COM);

    uint8_t cmd[40];

    // End continuous mode:
    cmd[0] = 0x20;
    cmd[1] = 0x25;
    uint16_t cmd_len = 2;
    if (!SendCommandToSICK(cmd,cmd_len)) return false;
    return LMS_waitIncomingFrame(50);
}

bool CSickLaserSerial::SendCommandToSICK(const uint8_t *cmd,const uint16_t cmd_len)
{
    uint8_t cmd_full[1024];
    ASSERT_(sizeof(cmd_full)>cmd_len+4U+2U);

	CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
	ASSERT_(COM);

    // Create header
    cmd_full[0] = 0x02; // STX
    cmd_full[1] = 0;    // ADDR
    cmd_full[2] = cmd_len & 0xFF;
    cmd_full[3] = cmd_len >> 8;

	memcpy(cmd_full+4,cmd,cmd_len);

	const uint16_t crc = mrpt::utils::compute_CRC16(cmd_full, 4+cmd_len,  CRC16_GEN_POL);
	cmd_full[4+cmd_len+0] = crc & 0xFF;
	cmd_full[4+cmd_len+1] = crc >> 8;

    const size_t toWrite = 4+cmd_len+2;

#if 0
    printf("TX: ");
    for (unsigned int i=0;i<toWrite;i++)
        printf("%02X ",cmd_full[i]);
    printf("\n");
#endif

    const int NTRIES = 3;

    for (int k=0;k<NTRIES;k++)
    {
        if (toWrite!=COM->Write( cmd_full, toWrite ))
        {
            cout << "[CSickLaserSerial::SendCommandToSICK] Error writing data to serial port." << endl;
            return false;
        }
        mrpt::system::sleep(15);
        if (LMS_waitACK(50)) return true;
        mrpt::system::sleep(10);
    }

    return false;
}


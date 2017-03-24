/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/hwdrivers/CRoboticHeadInterface.h>

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::hwdrivers;

/*-------------------------------------------------------------
						CRoboticHeadInterface
-------------------------------------------------------------*/
CRoboticHeadInterface::CRoboticHeadInterface() :
	mrpt::utils::COutputLogger("CRoboticHeadInterface")
{
	MRPT_START

	m_serialNumber = "OREJA001";
	gain.resize(3);
	gain[0]=127;	//0x7F (maximo)
	gain[1]=127;	//0x7F (maximo)
	gain[2]=127;	//0x7F (maximo)
	head_yaw = 0;
	head_pitch = 0;

	MRPT_END
}

/*-------------------------------------------------------------
						loadConfig_sensorSpecific
-------------------------------------------------------------*/
void  CRoboticHeadInterface::loadConfig_sensorSpecific(	const mrpt::utils::CConfigFileBase *configSource,
								const std::string	  &iniSection )
{
	configSource->read_vector(iniSection.c_str(),"gain0",gain,gain);
	m_serialNumber = configSource->read_string(iniSection.c_str(),"OREJA_serialNumber",m_serialNumber);
	head_yaw = configSource->read_int(iniSection.c_str(),"HeadYaw",head_yaw);
	head_pitch = configSource->read_int(iniSection.c_str(),"HeadPitch",head_pitch);
}

/*-------------------------------------------------------------
							GetGain
-------------------------------------------------------------*/
void CRoboticHeadInterface::GetGain(int &_gain,int &channel)
{
	msg.type=0x58;
	msg.content.resize(1);
	msg.content[0]=(unsigned char)channel;
	m_usbConnection.sendMessage(msg);
	while (!m_usbConnection.receiveMessage(msg));
	_gain = msg.content[0];
	if (msg.content[0])
		THROW_EXCEPTION("ERROR LEYENDO LA GANANCIA DEL AMPLIFICADOR DE LA OREJA \n");
}


/*-------------------------------------------------------------
							SetGain
-------------------------------------------------------------*/
bool CRoboticHeadInterface::SetGain(int &new_gain,int &channel)
{
	msg.type = 0x57;
	msg.content.resize(2);
	msg.content[0]=(unsigned char)channel;
	msg.content[1]=(unsigned char)new_gain;
	m_usbConnection.sendMessage(msg);
	while (!m_usbConnection.receiveMessage(msg));
	if (msg.content[0]==0)
		return 0;
	else
		return 1;
}

/*-------------------------------------------------------------
						GetSoundLocation
-------------------------------------------------------------*/
void CRoboticHeadInterface::GetSoundLocation(int &ang)
{
	msg.type = 0x59;
	msg.content.resize(0);
	m_usbConnection.sendMessage(msg);
	while (!m_usbConnection.receiveMessage(msg));
	ang = 256*(int)msg.content[1]+(int)msg.content[0];
}

/*-------------------------------------------------------------
						Get3SoundBuffer
-------------------------------------------------------------*/
void CRoboticHeadInterface::Get3SoundBuffer(CMatrixTemplate<int>	&buf)
{
	buf.setSize(3,500);	//3 channel, 500 samples per channel
	msg.type = 0x51;
	msg.content.resize(0);
	m_usbConnection.sendMessage(msg);

	//Las lecturas se haran de 100 en 100 datos y hay 3 buffers de 500 muestras cada uno
	for (size_t k = 0; k < 3; k++)	//Lectura de cada canal
	{
		for (size_t j = 0; j < 500/100; j++)	//Lectura de un canal completo
		{
			while (!m_usbConnection.receiveMessage(msg));
			for (size_t i = 0; i < 100; i++)	//Lectura de un envio
				buf(k,100*j+i) = 256*(int)msg.content[2*i+1]+(int)msg.content[2*i];
		}
	}
}

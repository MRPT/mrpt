/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers.h> // Precompiled headers

#include <mrpt/utils/utils_defs.h>
#include <mrpt/system/os.h>

using namespace mrpt::utils;
using namespace mrpt::hwdrivers;

/*-------------------------------------------------------------
						CRoboticHeadInterface
-------------------------------------------------------------*/
CRoboticHeadInterface::CRoboticHeadInterface()
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

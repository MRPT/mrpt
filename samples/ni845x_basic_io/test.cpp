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

#include <mrpt/hwdrivers/CInterfaceNI845x.h>
#include <mrpt/system.h>
#include <mrpt/gui.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::hwdrivers;


// ------------------------------------------------------
//				TestNI_USB_845x
// ------------------------------------------------------
void TestNI_USB_845x()
{
	CInterfaceNI845x  ni_usb;

	// Open first connected device:
	cout << "Openning device...\n";
	ni_usb.open(); 
	cout << "Done! Connected to: " << ni_usb.getDeviceDescriptor() << endl;

	ni_usb.setIOVoltageLevel( 25 ); // 2.5 volts
	
#if 0
	ni_usb.setIOPortDirection(0, 0xFF);
	while (!mrpt::system::os::kbhit())
	{
		ni_usb.writeIOPort(0, 0xFF); 
		mrpt::system::sleep(500);
		ni_usb.writeIOPort(0, 0x00); 
		mrpt::system::sleep(500);
	}
#endif

#if 0
	const size_t N=1000;
	std::vector<double> d0(N),d1(N),d2(N);

	ni_usb.setIOPortDirection(0, 0x00);
	for (size_t i=0;i<N;i++)
	{
		uint8_t d = ni_usb.readIOPort(0);
		mrpt::system::sleep(1);
		d0[i]= (d & 0x01) ? 1.0 : 0.0;
		d1[i]= (d & 0x02) ? 3.0 : 2.0;
		d2[i]= (d & 0x04) ? 5.0 : 4.0;
	}

	CDisplayWindowPlots win("Signals",640,480);

	win.hold_on();
	win.plot(d0, "b-");
	win.plot(d1, "r-");
	win.plot(d2, "k-");
	win.axis_fit();
	win.waitForKey();
#endif

#if 0
	ni_usb.create_SPI_configurations(1);
	ni_usb.set_SPI_configuration(0 /*idx*/, 0 /* CS */, 1000 /* Khz */, false /* clock_polarity_idle_high */, false /* clock_phase_first_edge */ );

	// Powerdown OFF
	const uint8_t write1[2] = { 0x20, 0x0F };
	uint8_t read[2];
	size_t nRead;
	ni_usb.read_write_SPI(0 /* config idx */, 2, write1, nRead, read );

	// High pass filter:
	//const uint8_t writeHF[2] = { 0x21, 0x09 };
	//ni_usb.read_write_SPI(0 /* config idx */, 2, writeHF, nRead, read );

	//const double K = 250 / ((1<<16) -1);
	const double K = 8.75e-3;


	std::vector<double> Zs;

	// Read Gyro X,Y,Z
	while (!mrpt::system::os::kbhit())
	{
		const uint8_t write2[7] = { 0xC0 | 0x28, 0x00,0x00, 0x00,0x00, 0x00,0x00 };
		uint8_t       read2[7];

		ni_usb.read_write_SPI(0 /* config idx */, sizeof(write2), write2, nRead, read2);

		double acc[3];
		acc[0] = K*int16_t( read2[1] | (read2[2]<<8) );
		acc[1] = K*int16_t( read2[3] | (read2[4]<<8) );
		acc[2] = K*int16_t( read2[5] | (read2[6]<<8) );

		printf("Gyros: X=%05.02f Y=%05.02f Z=%05.02f     \r", acc[0], acc[1],acc[2] );

		Zs.push_back(acc[2]);

		//mrpt::system::sleep(300);
	}
#endif

#if 1
	ni_usb.create_I2C_configurations(1);
	ni_usb.set_I2C_configuration(0 /*config_idx*/, 0x69 /*address*/, 0 /*timeout*/, 7 /*addr_size*/, 100 /* clock_rate_khz*/, 0 /*I2C_port*/ );


#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23

	// writeI2C(CTRL_REG1, 0x1F);  // Turn on all axes, disable power down
	{
		const uint8_t write[2] = { CTRL_REG1, 0x1F };
		ni_usb.write_I2C(0, sizeof(write), write);
	}

	//writeI2C(CTRL_REG3, 0x08);    // Enable control ready signal
	{
		const uint8_t write[2] = {CTRL_REG3, 0x08 };
		ni_usb.write_I2C(0, sizeof(write), write);
	}

	//writeI2C(CTRL_REG4, 0x80);    // Set scale (500 deg/sec)
	{
		const uint8_t write[2] = { CTRL_REG4, 0x80 };
		ni_usb.write_I2C(0, sizeof(write), write);
	}
	mrpt::system::sleep(10);
	
	//const double K = 250 / ((1<<16) -1);
	const double K = 500.0 / ((1<<16) -1);

	std::vector<double> Zs;

	// Read Gyro X,Y,Z
	while (!mrpt::system::os::kbhit())
	{
		uint8_t gx[2],gy[2],gz[2];
		size_t num_read;

		uint8_t wr_x_MSB[1] = {0x29}, wr_x_LSB[1] = {0x28};
		uint8_t wr_y_MSB[1] = {0x2B}, wr_y_LSB[1] = {0x2A};
		uint8_t wr_z_MSB[1] = {0x2D}, wr_z_LSB[1] = {0x2C};

		ni_usb.write_read_I2C(0, 1, wr_x_MSB, 1, num_read,&gx[1] );  ASSERT_(num_read==1)
		ni_usb.write_read_I2C(0, 1, wr_x_LSB, 1, num_read,&gx[0] ); ASSERT_(num_read==1)

		ni_usb.write_read_I2C(0, 1, wr_y_MSB, 1, num_read,&gy[1] );  ASSERT_(num_read==1)
		ni_usb.write_read_I2C(0, 1, wr_y_LSB, 1, num_read,&gy[0] ); ASSERT_(num_read==1)

		ni_usb.write_read_I2C(0, 1, wr_z_MSB, 1, num_read,&gz[1] );  ASSERT_(num_read==1)
		ni_usb.write_read_I2C(0, 1, wr_z_LSB, 1, num_read,&gz[0] ); ASSERT_(num_read==1)

		double acc[3];
		acc[0] = K*int16_t( gx[0] | (gx[1]<<8) );
		acc[1] = K*int16_t( gy[0] | (gy[1]<<8) );
		acc[2] = K*int16_t( gz[0] | (gz[1]<<8) );

		printf("Gyros: X=%05.02f Y=%05.02f Z=%05.02f     \r", acc[0], acc[1],acc[2] );

		Zs.push_back(acc[2]);

		mrpt::system::sleep(500);
	}


	CDisplayWindowPlots win;

	win.plot(Zs);
	win.axis_fit();
	win.waitForKey();

#endif
	mrpt::system::pause();
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestNI_USB_845x();
		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
}


/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CBoardENoses.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace std;

//Global VARs

bool firstTime =true;
std::map<int, string> colors;
std::map<int, CDisplayWindowPlots*> Win_map;

float Hbaselines[] ={3.5, 1.44, 0.877, 0.71, 0.20, 0.18};
float Vbaselines[] ={30,45,300};
std::map<float, std::vector<float> > Baseline_map;
std::map<float, std::vector<float> > Vertical_lines_map;


//FUNCTIONS
void myOnMenu(int menuID,float x,float y, void*param)
{
	cout << "Menu: " << menuID << endl << " x=" << x << " y="<< y << endl;
}

/** Destroy the windows created.
*/
void clear_memory()
{
	for( map< int, CDisplayWindowPlots* >::iterator iter = Win_map.begin(); iter != Win_map.end(); ++iter )
	{
		delete iter->second;
		iter->second = NULL;
	}
}


void add_VLines()
{
	std::vector <float> Vline;
	for (float inc=0; inc<=5; inc+=0.05)
	{
		Vline.push_back(inc);
		//Vertical lines.
		for (size_t i=0; i<sizeof(Vbaselines)/4; i++)
		{
			Vertical_lines_map[Vbaselines[i]].push_back(Vbaselines[i]);
			//plot it
			char sensor_name [50];
			sprintf(sensor_name,"%f",Vbaselines[i]);
			string name(sensor_name);
			Win_map[1]->plot(Vertical_lines_map[Vbaselines[i]],Vline,"k.1", sensor_name);
		}
	}
}

/** Plot each sensor reading over time
*/
void plot_diff_windows(std::map<int, vector<float> > &sensors, bool together)
{
	//Create windows if first time.
	try
	{
		if (together)	//All sensor on same window
		{
			if (firstTime)
			{
				//Create windows
				firstTime=false;
				Win_map[0] = new CDisplayWindowPlots("Temperature :");
				Win_map[0]->resize(800,900);
				Win_map[0]->axis(-15,1800,-5,50,false);
				Win_map[0]->axis_equal(false);
				Win_map[0]->setPos(0,0);

				Win_map[1] = new CDisplayWindowPlots("Sensor Readings :");
				Win_map[1]->resize(800,900);
				Win_map[1]->axis(-5,300,-1,6,false);
				Win_map[1]->axis_equal(false);
				Win_map[1]->setPos(810,0);

				//Create color palet
				colors[0x2600]="r-3";
				colors[0x2611]="b-3";
				colors[0x2620]="m-3";
				colors[0x2442]="g-3";
				colors[0x6810]="k-3";
				colors[0x5135]="r-2";
				colors[0x5521]="b-2";
				colors[0x5042]="m-2";

				colors[0x3]="g.0";
				colors[0x41]="g.0";
				colors[0x549]="g.0";
				colors[0x1]="g.0";

				add_VLines();

			}
			// Plot
			for( map<int, std::vector<float> >::iterator iter = sensors.begin(); iter != sensors.end(); ++iter )
			{
				if (iter->first == 0xFFFF){ //Temperature
					Win_map[0]->plot(sensors[0x0000], iter->second,"b-3");
				}
				else if (iter->first != (0x0000) ){	//Is not Time
					Win_map[1]->plot(sensors[0x0000], iter->second,colors[iter->first], format("%i",int(iter->first)) );
				}
			}

		}
		else	//Eah sensor has his own window
		{
			if (firstTime)
			{
				firstTime=false;
				int posx=0;
				int posy=0;
				for( map<int, std::vector<float> >::iterator iter = sensors.begin(); iter != sensors.end(); ++iter )
				{
					if (iter->first != (0x0000) ){	//Is not Time
						Win_map[iter->first] = new CDisplayWindowPlots( format("Sensor: %i",int(iter->first)) );
						Win_map[iter->first]->resize(800,100);
						Win_map[iter->first]->axis(-1,240,-1,6,false);
						Win_map[iter->first]->axis_equal(false);
						Win_map[iter->first]->setPos(posx*810,posy*160);
						if(posy==5){posx++; posy=0;}
						else{posy++;}
					}
				}
			}

			for( map<int, std::vector<float> >::iterator iter = sensors.begin(); iter != sensors.end(); ++iter )
			{
				if (iter->first != (0x0000) ){	//Is not Time
					Win_map[iter->first]->plot(sensors[0x0000], sensors[iter->first],"b-3");
				}
			}
		}//End-if(together)

	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
	}

}
//ADD_BASELINE
/**
	If configured, add to the plot window different lines as references.
*/



void add_baseline(std::vector<float> time)
{
	try
	{
		//Horizontal lines.
		for (size_t i=0; i<sizeof(Hbaselines)/4; i++)
		{
			Baseline_map[Hbaselines[i]].push_back(Hbaselines[i]);
			//plot it
			char sensor_name [10];
			sprintf(sensor_name,"%f",Hbaselines[i]);
			string name(sensor_name);
			Win_map[1]->plot(time, Baseline_map[Hbaselines[i]], "k.1", sensor_name);
		}
	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
	}

}


int main()
{
	try
	{
		CBoardENoses			eNoses;
		std::string				firmVers;
		CObservationGasSensors	obs;
		FILE					*f_log = os::fopen(format("./log_%s.txt", fileNameStripInvalidChars(mrpt::system::dateTimeLocalToString(now())).c_str() ),"wt");
		TTimeStamp				timStart = mrpt::system::getCurrentTime();
		std::map<int, std::vector<float> > Sensor_array;
		bool together = true;	//True -> Shows all sensor on same window. False -> Each sensor has his own window

		/*
		// Load configuration:
		ASSERT_( mrpt::system::fileExists("_CONFIG_eNoses.ini") );
		CConfigFile conf("_CONFIG_eNoses.ini");
		eNoses.loadConfig( conf, "eNoses" );

		if (!eNoses.queryFirmwareVersion( firmVers ) )
		{
			printf("Error!!\n");
			return -1;
		}
		else
			std::cout << "FIRMWARE VERSION: " << firmVers << std::endl;
		*/



		while ( !mrpt::system::os::kbhit() )
		{
			if (! eNoses.getObservation( obs ) )	//NEW OBSERVATION
			{
				cout << "- Could not retrieve an observation from the eNoses..." << endl;
			}
			else
			{
				//Push_back timeStamp (seconds)
				Sensor_array[0x0000].push_back( mrpt::system::timeDifference(timStart,obs.timestamp) );
				if (f_log) fprintf(f_log,"%f ", mrpt::system::timeDifference(timStart,obs.timestamp) );

				for (size_t i=0;i<obs.m_readings.size();i++)
				{
					//Push_back Temperature
					Sensor_array[0xFFFF].push_back( obs.m_readings[i].temperature );
					if (f_log) fprintf(f_log,"%f ", obs.m_readings[i].temperature );

					//E-Nose Sensor's Data
					for (size_t j=0;j<obs.m_readings[i].sensorTypes.size();j++)
					{

						if ( j<(obs.m_readings[i].sensorTypes.size() -1) ){	//Not the las item
							if( obs.m_readings[i].sensorTypes[j]==obs.m_readings[i].sensorTypes[j+1] ){
								Sensor_array[ obs.m_readings[i].sensorTypes[j] ].push_back(obs.m_readings[i].readingsVoltage[j+1]-obs.m_readings[i].readingsVoltage[j]);
								if (f_log) fprintf(f_log,"%f ",obs.m_readings[i].readingsVoltage[j+1]-obs.m_readings[i].readingsVoltage[j] );
								j++;	//skip the next sensor as it has been used already.
							}else{
								Sensor_array[ obs.m_readings[i].sensorTypes[j] ].push_back( obs.m_readings[i].readingsVoltage[j] );
								if (f_log) fprintf(f_log,"%f ",obs.m_readings[i].readingsVoltage[j]);	//To file
							}
						}else{
								Sensor_array[ obs.m_readings[i].sensorTypes[j] ].push_back( obs.m_readings[i].readingsVoltage[j] );
								if (f_log) fprintf(f_log,"%f ",obs.m_readings[i].readingsVoltage[j]);	//To file
						}
					}
				}
				if (f_log) fprintf(f_log,"\n");


				//Plotting information
				plot_diff_windows(Sensor_array,together);

				//Generate the Baseline
				add_baseline(Sensor_array[0x0000]);
			} //end-if getObs

			mrpt::system::sleep(40);
		}//end-While

		clear_memory();
		if (f_log) os::fclose(f_log);
	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
		return -1;
	}

	return 0;
}

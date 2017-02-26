/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/random.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSphere.h>

using namespace mrpt;
using namespace std;
using namespace mrpt::gui;
using namespace mrpt::random;
using namespace mrpt::opengl;
using namespace mrpt::utils;

const size_t  	N_MASSES = 750;

const double    BOX     = 500;
const double	V0		= 100;
const double    MASS_MIN = log(40.0), MASS_MAX = log(100.0);
const double    M2R      = 2.0;
const double  	LARGEST_STEP = 0.001;
const double 	G = 300;
const double  	COLLIS_LOSS = 0.98;

struct TMass
{
	TMass() : x(0),y(0),z(0),vx(0),vy(0),vz(0), mass(1), obj3d()
	{ }

	double	x,y,z;
	double	vx,vy,vz;
	double  mass;
	double  radius;
	opengl::CSpherePtr	obj3d;
};

void simulateGravity( vector<TMass> &objs, double At);

// ------------------------------------------------------
//				GravityDemo
// ------------------------------------------------------
void GravityDemo()
{
	CDisplayWindow3D	win("MRPT example: 3D gravity simulator- JLBC 2008",1000,700);


	randomGenerator.randomize();

	win.setCameraElevationDeg( 50.0f );
	win.setCameraZoom( 1000 );

	COpenGLScenePtr &theScene = win.get3DSceneAndLock();

	// Modify the scene:
	// ------------------------------------------------------
	{
		opengl::CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-2000,2000,-2000,2000,0,100);
		obj->setColor(0.3,0.3,0.3);
		theScene->insert( obj );
	}

	// Create the masses:
	// ----------------------------------------------------

	vector<TMass>	masses(N_MASSES);

	// Init at random poses & create opengl objects:
	for (size_t i=0;i<N_MASSES;i++)
	{
		masses[i].x = randomGenerator.drawUniform(-BOX,BOX);
		masses[i].y = randomGenerator.drawUniform(-BOX,BOX);
		masses[i].z = randomGenerator.drawUniform(-BOX,BOX)/10;

		double a=atan2(masses[i].y,masses[i].x);

		masses[i].vx = -V0*sin(a) + randomGenerator.drawUniform(-V0*0.01,V0*0.01);
		masses[i].vy =  V0*cos(a) + randomGenerator.drawUniform(-V0*0.01,V0*0.01);
		masses[i].vz =  0; //randomGenerator.drawUniform(-V0,V0);

		masses[i].mass = exp( randomGenerator.drawUniform(MASS_MIN,MASS_MAX) );
		opengl::CSpherePtr & obj = masses[i].obj3d = opengl::CSphere::Create();

		obj->setColor(
			randomGenerator.drawUniform(0.1,0.9),
			randomGenerator.drawUniform(0.1,0.9),
			randomGenerator.drawUniform(0.1,0.9)  );

		masses[i].radius = M2R * pow( masses[i].mass, 1.0/3.0);
		obj->setRadius( masses[i].radius ); // Guess why ^(1/3) ;-)

		obj->setLocation( masses[i].x, masses[i].y, masses[i].z );
		theScene->insert( obj );
	}

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();

	mrpt::utils::CTicTac	tictac;
	tictac.Tic();

	double t0 = tictac.Tac();

	while (!mrpt::system::os::kbhit() && win.isOpen() )
	{
		// Move the scene:
		double t1 = tictac.Tac();
		double At = t1-t0;
		t0 = t1;

		// Simulate a At, possibly in several small steps:
		// Update the 3D scene:
		win.get3DSceneAndLock();

		size_t  n_steps = ceil(At/LARGEST_STEP)+1;
		double At_steps = At / n_steps;
		n_steps = min(n_steps,size_t(3));
		for (size_t j=0;j<n_steps;j++)
			simulateGravity( masses, At_steps);


		for (size_t i=0;i<masses.size();i++)
		{
			opengl::CSpherePtr & obj = masses[i].obj3d;
			obj->setLocation( masses[i].x, masses[i].y, masses[i].z );
		}
		// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
		win.unlockAccess3DScene();

		// Update window:
		win.forceRepaint();
		mrpt::system::sleep(1);
	};
}

struct TForce
{
	TForce() { f[0]=f[1]=f[2]=0; }
	double f[3];
};


void simulateGravity( vector<TMass> &objs, double At)
{
	const size_t N=objs.size();

//	typedef vector<double> TForce;

	vector< TForce >	forces(N);

	// Index in the array must be larger than its content!!
	vector<pair<size_t,double> >  lstMass_i_joins_j(N, pair<size_t,double>(string::npos,10000.0));

	for (size_t i=0;i<(N-1);i++)
	{
		const double Ri =  objs[i].radius;

		// Compute overall gravity force:
		for (size_t j=i+1;j<N;j++)
		{
			double Ax = objs[j].x - objs[i].x;
			double Ay = objs[j].y - objs[i].y;
			double Az = objs[j].z - objs[i].z;
			double D2 = square(Ax)+square(Ay)+square(Az);

			double D = sqrt(D2);
			if (D==0) continue;

			const double Rj =  objs[j].radius;

			if (D<(Ri+Rj)) // Collission!!
			{
				// Index in the array must be larger than its content!!
				if (D<lstMass_i_joins_j[j].second)
				{
					lstMass_i_joins_j[j].first=i;
					lstMass_i_joins_j[j].second=D;
				}
			}
			else
			{
				double K = G * objs[i].mass * objs[j].mass / square( max(D,1.0) );
				double D_1 = 1.0/D;
				Ax *= D_1;
				Ay *= D_1;
				Az *= D_1;

				const double fx= Ax * K;
				const double fy= Ay * K;
				const double fz= Az * K;

				forces[i].f[0] += fx;
				forces[i].f[1] += fy;
				forces[i].f[2] += fz;

				forces[j].f[0] -= fx;
				forces[j].f[1] -= fy;
				forces[j].f[2] -= fz;
			}
		}
	}

	// F = m a
	for (size_t i=0;i<N;i++)
	{
		const double M_1=1.0/objs[i].mass;

		forces[i].f[0] *= M_1;
		forces[i].f[1] *= M_1;
		forces[i].f[2] *= M_1;

		objs[i].vx += forces[i].f[0]*At;
		objs[i].vy += forces[i].f[1]*At;
		objs[i].vz += forces[i].f[2]*At;

		objs[i].x += objs[i].vx*At;
		objs[i].y += objs[i].vy*At;
		objs[i].z += objs[i].vz*At;
	}

//	return;
	// Join masses that collided:
	for (int i=N-1;i>=0;i--)
	{
		const size_t newObj = lstMass_i_joins_j[i].first;
		if (newObj==string::npos) continue;

		const double Mi = objs[i].mass;
		const double Mj = objs[newObj].mass;
		const double newMass = Mi+Mj;
		const double newMass_1 = 1.0/newMass;

		objs[newObj].vx = COLLIS_LOSS*newMass_1*( Mj*objs[newObj].vx+Mi*objs[i].vx );
		objs[newObj].vy = COLLIS_LOSS*newMass_1*( Mj*objs[newObj].vy+Mi*objs[i].vy );
		objs[newObj].vz = COLLIS_LOSS*newMass_1*( Mj*objs[newObj].vz+Mi*objs[i].vz );

		objs[newObj].x = newMass_1*( Mj*objs[newObj].x+Mi*objs[i].x );
		objs[newObj].y = newMass_1*( Mj*objs[newObj].y+Mi*objs[i].y );
		objs[newObj].z = newMass_1*( Mj*objs[newObj].z+Mi*objs[i].z );

		objs[newObj].mass = newMass;
		objs[newObj].radius = M2R * pow( newMass, 1.0/3.0);
		objs[newObj].obj3d->setRadius( objs[newObj].radius );

		objs[i].obj3d.clear(); // Delete Sphere
		objs.erase(objs.begin()+i);
	}

}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		GravityDemo();
		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}

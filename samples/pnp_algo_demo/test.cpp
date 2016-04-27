#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/utils/CTicTac.h>

using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

int main()
{
	cout << "Starting Program"<<endl;
	
	double val=sqrtf(0.1f*0.1f+0.25f*0.25f+0.75f*0.75f+0.5f*0.5f);
	CMatrixDouble44 T;
	
	// {G} - Global Frame
	// {C} - Camera Frame
	// Given Transformation Matrix of {G} w.r.t. {C} 
	
	CPose3DQuat q(10,0,0, CQuaternionDouble(0.75f/val,-0.5f/val,0.25f/val,-0.1f/val));
	cout << "q=" << q <<endl;
	
	q.getHomogeneousMatrix(T);
	
	cout<<"T="<<T<<endl;
	
	// Given 4 points in {G} for P4P 
	CPoint3D pt1(1.0,2.0,3.0), pt2(-2.0, 3.0,4.0), pt3(-2.0, -1.5, 0.25), pt4(1.2, -3.2, 1.3);
	
	cout << "pts=" << pt1 << endl << pt2 << endl << pt3 << endl << pt4 << endl << endl;
	
	return 0;
	
}

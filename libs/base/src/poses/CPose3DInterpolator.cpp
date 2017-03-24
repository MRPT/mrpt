/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/stl_serialization.h>
#include <mrpt/math/slerp.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/math/interp_fit.h>
#include <mrpt/math/CMatrixD.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CPose3DInterpolator, CSerializable, mrpt::poses)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CPose3DInterpolator::CPose3DInterpolator() : m_method( CPose3DInterpolator::imLinearSlerp )
{
	maxTimeInterpolation = -1.0;

}


/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CPose3DInterpolator::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << m_path;
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CPose3DInterpolator::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			in >> m_path;
		}
	break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}


/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void CPose3DInterpolator::clear()
{
	m_path.clear();
}

/*---------------------------------------------------------------
						insert
  ---------------------------------------------------------------*/
void CPose3DInterpolator::insert( mrpt::system::TTimeStamp t, const CPose3D &p)
{
	m_path[t] = p;
}

/*---------------------------------------------------------------
						interpolate
  ---------------------------------------------------------------*/
CPose3D & CPose3DInterpolator::interpolate( mrpt::system::TTimeStamp t, CPose3D &out_interp, bool &out_valid_interp ) const
{
	TTimePosePair p1, p2, p3, p4;

	// Invalid?
	if (t==INVALID_TIMESTAMP)
	{
		out_valid_interp = false;
		return out_interp;
	}

	// We'll look for 4 consecutive time points.
	// Check if the selected method needs all 4 points or just the central 2 of them:
	bool interp_method_requires_4pts;
	switch (m_method)
	{
	case imLinear2Neig:
	case imSplineSlerp:
	case imLinearSlerp:
		interp_method_requires_4pts = false;
		break;
	default:
		interp_method_requires_4pts = true;
		break;
	};


	// Out of range?
	const_iterator it_ge1 = m_path.lower_bound( t );

	// Exact match?
	if( it_ge1 != m_path.end() && it_ge1->first == t )
	{
		out_interp = it_ge1->second;
		out_valid_interp = true;
		return out_interp;
	}

	// Are we in the beginning or the end of the path?
	if( it_ge1 == m_path.end() || it_ge1 == m_path.begin() )
	{
		out_valid_interp = false;
		out_interp.setFromValues(0,0,0);
		return out_interp;
	} // end

	p3 = *it_ge1;		// Third pair
	const_iterator it_ge2 = it_ge1; ++it_ge2;
	if(it_ge2 == m_path.end() )
	{
		if (interp_method_requires_4pts) {
			out_valid_interp = false;
			out_interp.setFromValues(0,0,0);
			return out_interp;
		}
	}
	else {
		p4 = *(it_ge2);		// Fourth pair
	}

	p2 = *(--it_ge1);	// Second pair

	if( it_ge1 == m_path.begin() )
	{
		if (interp_method_requires_4pts) {
			out_valid_interp = false;
			out_interp.setFromValues(0, 0, 0);
			return out_interp;
		}
	}
	else {
		p1 = *(--it_ge1);	// First pair
	}

	// Test if the difference between the desired timestamp and the next timestamp is lower than a certain (configurable) value
	const double dt12 = interp_method_requires_4pts ? (p2.first - p1.first) / 1e7 : .0;
	const double dt23 = (p3.first - p2.first) / 1e7;
	const double dt34 = interp_method_requires_4pts ? (p4.first - p3.first) / 1e7 : .0;

	if( maxTimeInterpolation > 0 &&
	  (dt12 > maxTimeInterpolation ||
	   dt23 > maxTimeInterpolation ||
	   dt34 > maxTimeInterpolation ))
	{
		out_valid_interp = false;
		out_interp.setFromValues(0,0,0);
		return out_interp;
	}

	// Do interpolation:
	// ------------------------------------------
	// First Previous point:  p1
	// Second Previous point: p2
	// First Next point:	  p3
	// Second Next point:     p4
	// Time where to interpolate:  t
	double td     = mrpt::system::timestampTotime_t(t);

	CVectorDouble	ts;
	ts.resize(4);
	ts[0] = mrpt::system::timestampTotime_t(p1.first);
	ts[1] = mrpt::system::timestampTotime_t(p2.first);
	ts[2] = mrpt::system::timestampTotime_t(p3.first);
	ts[3] = mrpt::system::timestampTotime_t(p4.first);

	CVectorDouble	X,Y,Z,yaw,pitch,roll;
	X.resize(4);						Y.resize(4);							Z.resize(4);
	X[0]	= p1.second.x();				Y[0]	= p1.second.y();					Z[0]	= p1.second.z();
	X[1]	= p2.second.x();				Y[1]	= p2.second.y();					Z[1]	= p2.second.z();
	X[2]	= p3.second.x();				Y[2]	= p3.second.y();					Z[2]	= p3.second.z();
	X[3]	= p4.second.x();				Y[3]	= p4.second.y();					Z[3]	= p4.second.z();

	yaw.resize(4);						pitch.resize(4);						roll.resize(4);
	yaw[0]  = p1.second.yaw();			pitch[0]  = p1.second.pitch();			roll[0]  = p1.second.roll();
	yaw[1]  = p2.second.yaw();			pitch[1]  = p2.second.pitch();			roll[1]  = p2.second.roll();
	yaw[2]  = p3.second.yaw();			pitch[2]  = p3.second.pitch();			roll[2]  = p3.second.roll();
	yaw[3]  = p4.second.yaw();			pitch[3]  = p4.second.pitch();			roll[3]  = p4.second.roll();


	unwrap2PiSequence(yaw);
	unwrap2PiSequence(pitch);
	unwrap2PiSequence(roll);

	// Target interpolated values:
	double int_x,int_y,int_z,int_yaw,int_pitch,int_roll;

	switch (m_method)
	{
	case imSpline:
		{
		// ---------------------------------------
		//    SPLINE INTERPOLATION
		// ---------------------------------------
		int_x		= math::spline(td, ts, X);
		int_y		= math::spline(td, ts, Y);
		int_z		= math::spline(td, ts, Z);
		int_yaw		= math::spline(td, ts, yaw,		true );	// Wrap 2pi
		int_pitch	= math::spline(td, ts, pitch,	true );
		int_roll	= math::spline(td, ts, roll,	true );

		}
		break;

	case imLinear2Neig:
		{
		int_x		= math::interpolate2points(td, ts[1],X[1],ts[2],X[2]);
		int_y		= math::interpolate2points(td, ts[1],Y[1],ts[2],Y[2]);
		int_z		= math::interpolate2points(td, ts[1],Z[1],ts[2],Z[2]);
		int_yaw		= math::interpolate2points(td, ts[1],yaw[1],ts[2],yaw[2],	true );	// Wrap 2pi
		int_pitch	= math::interpolate2points(td, ts[1],pitch[1],ts[2],pitch[2],	true );
		int_roll	= math::interpolate2points(td, ts[1],roll[1],ts[2],roll[2],	true );
		}
		break;

	case imLinear4Neig:
		{
		int_x		= math::leastSquareLinearFit(td, ts, X);
		int_y		= math::leastSquareLinearFit(td, ts, Y);
		int_z		= math::leastSquareLinearFit(td, ts, Z);
		int_yaw		= math::leastSquareLinearFit(td, ts, yaw,	true );	// Wrap 2pi
		int_pitch	= math::leastSquareLinearFit(td, ts, pitch,	true );
		int_roll	= math::leastSquareLinearFit(td, ts, roll,	true );
		}
		break;

	case imSSLLLL:
		{
		int_x		= math::spline(td, ts, X);
		int_y		= math::spline(td, ts, Y);
		int_z		= math::leastSquareLinearFit(td, ts, Z);
		int_yaw		= math::leastSquareLinearFit(td, ts, yaw,	true );	// Wrap 2pi
		int_pitch	= math::leastSquareLinearFit(td, ts, pitch,	true );
		int_roll	= math::leastSquareLinearFit(td, ts, roll,	true );
		}
		break;

	case imSSLSLL:
		{
		int_x		= math::spline(td, ts, X);
		int_y		= math::spline(td, ts, Y);
		int_z		= math::leastSquareLinearFit(td, ts, Z);
		int_yaw		= math::spline(td, ts, yaw,	true );					// Wrap 2pi
		int_pitch	= math::leastSquareLinearFit(td, ts, pitch,	true );
		int_roll	= math::leastSquareLinearFit(td, ts, roll,	true );
		}
		break;

	case imLinearSlerp:
		{
		int_x		= math::interpolate2points(td, ts[1],X[1],ts[2],X[2]);
		int_y		= math::interpolate2points(td, ts[1],Y[1],ts[2],Y[2]);
		int_z		= math::interpolate2points(td, ts[1],Z[1],ts[2],Z[2]);

		const CPose3D aux1(0,0,0,yaw[1],pitch[1],roll[1]);
		const CPose3D aux2(0,0,0,yaw[2],pitch[2],roll[2]);
		CPose3D  q_interp;

		const double ratio = (td-ts[1])/(ts[2]-ts[1]);
		mrpt::math::slerp(aux1,aux2, ratio, q_interp);

		q_interp.getYawPitchRoll(int_yaw,int_pitch,int_roll);
		}
		break;

	case imSplineSlerp:
		{
		int_x		= math::spline(td, ts, X);
		int_y		= math::spline(td, ts, Y);
		int_z		= math::spline(td, ts, Z);

		const CPose3D aux1(0,0,0,yaw[1],pitch[1],roll[1]);
		const CPose3D aux2(0,0,0,yaw[2],pitch[2],roll[2]);
		CPose3D  q_interp;

		const double ratio = (td-ts[1])/(ts[2]-ts[1]);
		mrpt::math::slerp(aux1,aux2, ratio, q_interp);

		q_interp.getYawPitchRoll(int_yaw,int_pitch,int_roll);
		}
		break;

	default: THROW_EXCEPTION("Unknown value for interpolation method!");
	}; // end switch

	out_interp.setFromValues(int_x, int_y, int_z, int_yaw, int_pitch, int_roll);
	out_valid_interp = true;
	return out_interp;

} // end interpolate

/*---------------------------------------------------------------
					    getPreviousPose
  ---------------------------------------------------------------*/

bool CPose3DInterpolator::getPreviousPoseWithMinDistance(  const mrpt::system::TTimeStamp &t, double distance, CPose3D &out_pose )
{
	if( m_path.size() == 0 || distance <=0 )
		return false;

	CPose3D			myPose;

	// Search for the desired timestamp
	iterator  it = m_path.find(t);
	if( it != m_path.end() && it != m_path.begin() )
		myPose = it->second;
	else
		return false;


	double d = 0.0;
	do
	{
		--it;
		d = myPose.distance2DTo( it->second.x(), it->second.y());
	} while( d < distance && it != m_path.begin() );

	if( d >= distance )
	{
		out_pose = it->second;
		return true;
	}
	else
		return false;
} // end getPreviousPose

/*---------------------------------------------------------------
					setMaxTimeInterpolation
  ---------------------------------------------------------------*/
void CPose3DInterpolator::setMaxTimeInterpolation( double time )
{
	ASSERT_( time > 0 );
	maxTimeInterpolation = time;
} // end setMaxTimeInterpolation

/*---------------------------------------------------------------
					getMaxTimeInterpolation
  ---------------------------------------------------------------*/
double CPose3DInterpolator::getMaxTimeInterpolation( )
{
	return maxTimeInterpolation;
} // end getMaxTimeInterpolation


/*---------------------------------------------------------------
					saveToTextFile
  ---------------------------------------------------------------*/
bool CPose3DInterpolator::saveToTextFile(const std::string &s) const
{
	try
	{
		CFileOutputStream	f(s);

		for (const_iterator i=m_path.begin();i!=m_path.end();++i)
		{
			const double	t  = timestampTotime_t(i->first);
			const CPose3D	&p = i->second;
			int r = f.printf("%.06f %.06f %.06f %.06f %.06f %.06f %.06f\n",
				t,
				p.x(),p.y(),p.z(),
				p.yaw(),p.pitch(),p.roll());
			ASSERT_(r>0);
		}

		return true;
	}
	catch(...)
	{
		return false;
	}
}

/*---------------------------------------------------------------
					saveInterpolatedToTextFile
  ---------------------------------------------------------------*/
bool CPose3DInterpolator::saveInterpolatedToTextFile(const std::string &s, double period) const
{
	try
	{
		CFileOutputStream	f(s);

		if (m_path.empty()) return true;


		const TTimeStamp t_ini = m_path.begin()->first;
		const TTimeStamp t_end = m_path.rbegin()->first;

		TTimeStamp At = mrpt::system::secondsToTimestamp(period);

		//cout << "Interp: " << t_ini << " " << t_end << " " << At << endl;

		CPose3D	   p;
		bool       valid;

		for (TTimeStamp t=t_ini;t<=t_end;t+=At)
		{
			this->interpolate( t, p, valid );
			if (!valid) continue;

			int r = f.printf("%.06f %.06f %.06f %.06f %.06f %.06f %.06f\n",
				mrpt::system::timestampTotime_t(t),
				p.x(),p.y(),p.z(),
				p.yaw(),p.pitch(),p.roll());
			ASSERT_(r>0);
		}

		return true;
	}
	catch(...)
	{
		return false;
	}
}


/*---------------------------------------------------------------
					loadFromTextFile
  ---------------------------------------------------------------*/
bool CPose3DInterpolator::loadFromTextFile(const std::string &s)
{
	MRPT_START

	clear();
	CMatrixD	M;

	try
	{
		M.loadFromTextFile(s);
	}
	catch(std::exception &)
	{
		return false;	// error loading file
	}

	// Check valid format:
	if (M.getRowCount()==0) return false;
	ASSERT_(M.getColCount()==7);

	// load into the path:
	const size_t N = M.getColCount();
	for (size_t i=0;i<N;i++)
		insert(time_tToTimestamp( M(i,0) ), CPose3D( M(i,1),M(i,2),M(i,3),M(i,4),M(i,5),M(i,6) ) );

	return true;
	MRPT_END
}


void CPose3DInterpolator::getBoundingBox(mrpt::math::TPoint3D &Min, mrpt::math::TPoint3D &Max) const
{
	MRPT_START
	ASSERT_( !m_path.empty() );

	Min.x=Min.y=Min.z = std::numeric_limits<double>::max();
	Max.x=Max.y=Max.z =-std::numeric_limits<double>::max();

	for (const_iterator p=m_path.begin();p!=m_path.end();++p)
	{
		Min.x = min( Min.x, p->second.x());
		Min.y = min( Min.y, p->second.y());
		Min.z = min( Min.z, p->second.z());
		Max.x = max( Max.x, p->second.x());
		Max.y = max( Max.y, p->second.y());
		Max.z = max( Max.z, p->second.z());
	}

	MRPT_END
}

/*---------------------------------------------------------------
					getBoundingBox
  ---------------------------------------------------------------*/
void CPose3DInterpolator::getBoundingBox(CPoint3D &Min_, CPoint3D &Max_) const
{
	TPoint3D Min,Max;
	getBoundingBox(Min,Max);
	Min_ = CPoint3D(Min);
	Max_ = CPoint3D(Max);
}

/*---------------------------------------------------------------
					setInterpolationMethod
  ---------------------------------------------------------------*/
void CPose3DInterpolator::setInterpolationMethod( CPose3DInterpolator::TInterpolatorMethod method)
{
	m_method = method;
}

/*---------------------------------------------------------------
					getInterpolationMethod
  ---------------------------------------------------------------*/
CPose3DInterpolator::TInterpolatorMethod CPose3DInterpolator::getInterpolationMethod() const
{
	return m_method;
}

/*---------------------------------------------------------------
							filter
  ---------------------------------------------------------------*/
void CPose3DInterpolator::filter( unsigned int component, unsigned int samples )
{
	if (m_path.empty())
		return;

	TPath aux;

	int		ant, post;
	size_t	nitems = size();

	post	= (samples%2) ? (unsigned int)(samples/2) : samples/2;
	ant		= (unsigned int)(samples/2);

	int		k = 0;
	CPose3DInterpolator::iterator it1, it2, it3;

	//int asamples;
	for( it1 = m_path.begin(); it1 != m_path.end(); ++it1, ++k )
	{
		//asamples = samples;

		it2 = m_path.begin();
		if( k-ant > 0 )
			advance( it2, k-ant );
		/*else
			asamples = samples+k-ant;*/


		if( k+post < (int)nitems )
		{
			it3 = m_path.begin();
			advance( it3, k+post+1 );
		}
		else
		{
			it3 = m_path.end();
			//asamples = post+nitems-k;
		}

		CPose3D auxPose;

		unsigned int nsamples = distance(it2,it3);
		CPose3DPDFParticles particles(nsamples);
		for( unsigned int i = 0; it2 != it3; ++it2, ++i )
		{
		    particles.m_particles[i].log_w = 0;
		    particles.m_particles[i].d->setFromValues(
                        it1->second.x(), it1->second.y(), it1->second.z(),
                        it1->second.yaw(), it1->second.pitch(), it1->second.roll() );
			switch( component )
			{
				case 0:	particles.m_particles[i].d->x(it2->second.x());		break;
				case 1:	particles.m_particles[i].d->y(it2->second.y());		break;
				case 2:	particles.m_particles[i].d->z(it2->second.z());		break;
				case 3:	particles.m_particles[i].d->setYawPitchRoll(it2->second.yaw(),it1->second.pitch(),it1->second.roll()); break;
				case 4:	particles.m_particles[i].d->setYawPitchRoll(it1->second.yaw(),it2->second.pitch(),it1->second.roll()); 	break;
				case 5:	particles.m_particles[i].d->setYawPitchRoll(it1->second.yaw(),it1->second.pitch(),it2->second.roll()); 	break;
			} // end switch
		} // end for it2
        particles.getMean( auxPose );
		aux[it1->first] = auxPose;
	} // end for it1
	m_path = aux;
} // end filter

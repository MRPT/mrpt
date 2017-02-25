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
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DRotVec.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/geometry.h>  // skew_symmetric3()
#include <mrpt/math/ops_matrices.h>
#include <mrpt/utils/CStream.h>
#include <iomanip>
#include <limits>


using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;

IMPLEMENTS_SERIALIZABLE(CPose3DRotVec, CSerializable ,mrpt::poses)

MRPT_TODO("Complete missing methods")

/*---------------------------------------------------------------
	Constructors
  ---------------------------------------------------------------*/
CPose3DRotVec::CPose3DRotVec(const math::CMatrixDouble44 &m)
{
    m_coords[0] = m(0,3);
    m_coords[1] = m(1,3);
    m_coords[2] = m(2,3);

    m_rotvec = rotVecFromRotMat( m );
}

CPose3DRotVec::CPose3DRotVec(const CPose3D &m)
{
    m_coords[0] = m.x();
    m_coords[1] = m.y();
    m_coords[2] = m.z();
    CMatrixDouble44 R;
    m.getHomogeneousMatrix( R );
    m_rotvec = rotVecFromRotMat( R );
}

/** Constructor from a quaternion (which only represents the 3D rotation part) and a 3D displacement. */
CPose3DRotVec::CPose3DRotVec(const mrpt::math::CQuaternionDouble &q, const double _x, const double _y, const double _z )
{
    m_coords[0]=_x; m_coords[1]=_y; m_coords[2]=_z;
    const double a = sqrt( q.x()*q.x() + q.y()*q.y() + q.z()*q.z() );
    const double TH = 0.001;
    const double k = a < TH ? 2 : 2*acos(q.r())/sqrt(1-q.r()*q.r());
    m_rotvec[0] = k*q.x();
    m_rotvec[1] = k*q.y();
    m_rotvec[2] = k*q.z();
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CPose3DRotVec::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << m_coords[0] << m_coords[1] << m_coords[2] << m_rotvec[0] << m_rotvec[1] << m_rotvec[2];
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CPose3DRotVec::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
            in >> m_coords[0] >> m_coords[1] >> m_coords[2] >> m_rotvec[0] >> m_rotvec[1] >> m_rotvec[2];
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CPose3DRotVec::setFromXYZAndAngles( const double x,const double  y,const double  z,const double  yaw, const double  pitch, const double roll)
{
    CPose3D aux(x,y,z,yaw,pitch,roll);
    this->m_coords[0] = aux.m_coords[0];
    this->m_coords[1] = aux.m_coords[1];
    this->m_coords[2] = aux.m_coords[2];
    this->m_rotvec    = aux.ln_rotation();
}

CArrayDouble<3> CPose3DRotVec::rotVecFromRotMat( const math::CMatrixDouble44 &m )
{
    // go through cpose3d
    CArrayDouble<3> out;
    CPose3D aux(m);
    out = aux.ln_rotation();
    return out;
} // end-rotVecFromRotMat

/**  Textual output stream function.
 */
std::ostream& mrpt::poses::operator << (std::ostream& o, const CPose3DRotVec& p)
{
	const std::streamsize old_pre = o.precision();
	const std::ios_base::fmtflags old_flags = o.flags();
	o << "(x,y,z,vx,vy,vz)=(" << std::fixed << std::setprecision(4) << p.m_coords[0] << "," << p.m_coords[1] << "," << p.m_coords[2] <<  ","
	  << p.m_rotvec[0] << "," << p.m_rotvec[1] << "," << p.m_rotvec[2] << ")";
	o.flags(old_flags);
	o.precision(old_pre);
	return o;
}

/** Get the 3x3 rotation matrix \sa getHomogeneousMatrix  */
void CPose3DRotVec::getRotationMatrix( mrpt::math::CMatrixDouble33 & ROT ) const
{
    // through cpose3D
    ROT = CPose3D().exp_rotation( this->m_rotvec );
//    cout << "CPOSE_3D: " << ROT;
}

/*---------------------------------------------------------------
		sphericalCoordinates
---------------------------------------------------------------*/
void CPose3DRotVec::sphericalCoordinates(
    const TPoint3D &point,
    double &out_range,
    double &out_yaw,
    double &out_pitch ) const
{
    // Pass to coordinates as seen from this 6D pose:
	TPoint3D local;
	this->inverseComposePoint(point.x,point.y,point.z, local.x,local.y,local.z);

    // Range:
	out_range = local.norm();

    // Yaw:
    if (local.y!=0 || local.x!=0)
         out_yaw = atan2(local.y,local.x);
    else out_yaw = 0;

    // Pitch:
    if (out_range!=0)
         out_pitch = -asin( local.z / out_range );
    else out_pitch = 0;
}

/*---------------------------------------------------------------
		composePoint
---------------------------------------------------------------*/
void CPose3DRotVec::composePoint(double lx,double ly,double lz, double &gx, double &gy, double &gz,
	mrpt::math::CMatrixFixedNumeric<double,3,3>  *out_jacobian_df_dpoint,
	mrpt::math::CMatrixFixedNumeric<double,3,6>  *out_jacobian_df_dpose) const
{
    const double angle  = this->m_rotvec.norm();
    const double K1     = sin(angle)/angle;
    const double K2     = (1-cos(angle))/(angle*angle);

    const double tx     = this->m_coords[0];
    const double ty     = this->m_coords[1];
    const double tz     = this->m_coords[2];

    const double w1     = this->m_rotvec[0];
    const double w2     = this->m_rotvec[1];
    const double w3     = this->m_rotvec[2];

    const double w1_2   = w1*w1;
    const double w2_2   = w2*w2;
    const double w3_2   = w3*w3;

    gx = lx*(1-K2*(w2_2+w3_2)) + ly*(K2*w1*w2-K1*w3)   + lz*(K1*w2+K2*w1*w3)   + tx;
    gy = lx*(K1*w3+K2*w1*w2)   + ly*(1-K2*(w1_2+w3_2)) + lz*(K2*w2*w3-K1*w1)   + ty;
    gz = lx*(K2*w1*w3-K1*w2)   + ly*(K1*w1+K2*w2*w3)   + lz*(1-K2*(w1_2+w2_2)) + tz;

	if (out_jacobian_df_dpoint || out_jacobian_df_dpose)
	{
		MRPT_TODO("Jacobians")
		THROW_EXCEPTION("Jacobians not implemented yet")
	}
}

/*---------------------------------------------------------------
		unary -
---------------------------------------------------------------*/
CPose3DRotVec mrpt::poses::operator -(const CPose3DRotVec &b)
{
	CMatrixDouble44 B_INV(UNINITIALIZED_MATRIX);
	b.getInverseHomogeneousMatrix(B_INV);
	return CPose3DRotVec(B_INV);
}

bool mrpt::poses::operator==(const CPose3DRotVec &p1,const CPose3DRotVec &p2)
{
	return (p1.m_coords==p2.m_coords)&&(p1.m_rotvec==p2.m_rotvec);
}

bool mrpt::poses::operator!=(const CPose3DRotVec &p1,const CPose3DRotVec &p2)
{
	return (p1.m_coords!=p2.m_coords)||(p1.m_rotvec!=p2.m_rotvec);
}

/*---------------------------------------------------------------
				point3D = pose3D + point3D
  ---------------------------------------------------------------*/
CPoint3D  CPose3DRotVec::operator + (const CPoint3D& b) const
{
    CPoint3D outPoint;

    this->composePoint(b.m_coords[0], b.m_coords[1], b.m_coords[2],
                       outPoint.m_coords[0],
                       outPoint.m_coords[1],
                       outPoint.m_coords[2]);

    return outPoint;
}

/*---------------------------------------------------------------
				point3D = pose3D + point2D
  ---------------------------------------------------------------*/
CPoint3D  CPose3DRotVec::operator + (const CPoint2D& b) const
{
    CPoint3D outPoint;

    this->composePoint( b.m_coords[0],
                        b.m_coords[1],
                        0,
                        outPoint.m_coords[0],
                        outPoint.m_coords[1],
                        outPoint.m_coords[2] );

    return outPoint;

}

void CPose3DRotVec::toQuatXYZ( CPose3DQuat &q ) const
{
    q.m_coords[0] = this->m_coords[0];
    q.m_coords[1] = this->m_coords[1];
    q.m_coords[2] = this->m_coords[2];

    const double a = sqrt( this->m_rotvec[0]*this->m_rotvec[0]+this->m_rotvec[1]*this->m_rotvec[1]+this->m_rotvec[2]*this->m_rotvec[2] );
    if(a < 0.001)
    {
        q.m_quat.r(1);
        q.m_quat.x(0.5*this->m_rotvec[0]);
        q.m_quat.y(0.5*this->m_rotvec[1]);
        q.m_quat.z(0.5*this->m_rotvec[2]);
        // TO DO: output of the jacobian
        // df_dr   = 0.25*[-r';2*eye(3)];
    }
    else
    {
        q.m_quat.fromRodriguesVector( this->m_rotvec );

        // TO DO: output of the jacobian
//        a2      = a*a;
//        a3      = a2*a;
//
//        r1      = r(1);
//        r2      = r(2);
//        r3      = r(3);
//        s       = sin(a/2);
//        c       = cos(a/2);
//
//        A       = a*c-2*s;
//
//        df_dr   = 1/a3*[-r1*a2*s -r2*a2*s -r3*a2*s; ...
//            2*a2*s+r1*r1*A r1*r2*A r1*r3*A; ...
//            r1*r2*A 2*a2*s+r2*r2*A r2*r3*A; ...
//            r1*r3*A r2*r3*A 2*a2*s+r3*r3*A];        % jacobian of transf.
//
    }
}

/*---------------------------------------------------------------
				this = A + B
  ---------------------------------------------------------------*/
void CPose3DRotVec::composeFrom(const CPose3DRotVec& A,
                                const CPose3DRotVec& B,
                                mrpt::math::CMatrixFixedNumeric<double,6,6>  *out_jacobian_drvtC_drvtA,
                                mrpt::math::CMatrixFixedNumeric<double,6,6>  *out_jacobian_drvtC_drvtB)

{
    const double a1 = A.m_rotvec.norm();    // angles
    const double a2 = B.m_rotvec.norm();

    // if the angles are small, we can compute the approximate composition easily
    if( a1 < 0.01 && a2 < 0.01 )
    {
        const CMatrixDouble33 Ra  = Eigen::MatrixXd::Identity(3,3) + mrpt::math::skew_symmetric3(A.m_rotvec);

        this->m_rotvec = A.m_rotvec + B.m_rotvec;
        this->m_coords = A.m_coords + Ra*B.m_coords;

        if( out_jacobian_drvtC_drvtA || out_jacobian_drvtC_drvtB )
        {
            if( out_jacobian_drvtC_drvtA )      // jacobian wrt A
            {
                out_jacobian_drvtC_drvtA->setIdentity(6,6);
                out_jacobian_drvtC_drvtA->insertMatrix(3,0,mrpt::math::skew_symmetric3_neg(B.m_coords));
            }

            if( out_jacobian_drvtC_drvtB )      // jacobian wrt B
            {
                out_jacobian_drvtC_drvtB->setIdentity(6,6);
                out_jacobian_drvtC_drvtB->insertMatrix(3,3,mrpt::math::skew_symmetric3(A.m_rotvec));
                (*out_jacobian_drvtC_drvtB)(3,3) =
                (*out_jacobian_drvtC_drvtB)(4,4) =
                (*out_jacobian_drvtC_drvtB)(5,5) = 1;
            }
        }
        return;
    }

    CMatrixDouble33 RA,RB;
    A.getRotationMatrix(RA);
    B.getRotationMatrix(RB);

    // Translation part
    CArrayDouble<3> coords = RA*B.m_coords + A.m_coords;
    this->m_coords[0] = coords[0];
    this->m_coords[1] = coords[1];
    this->m_coords[2] = coords[2];

    // Rotation part:
#if 0
    else if (A_is_small)            this->m_rotvec = B.m_rotvec;
    else if (B_is_small)            this->m_rotvec = A.m_rotvec;
    else
    {
        const double a1_inv = 1/a1;
        const double a2_inv = 1/a2;

        const double sin_a1_2 = sin(0.5*a1);
        const double cos_a1_2 = cos(0.5*a1);
        const double sin_a2_2 = sin(0.5*a2);
        const double cos_a2_2 = cos(0.5*a2);

        const double KA = sin_a1_2*sin_a2_2;
        const double KB = sin_a1_2*cos_a2_2;
        const double KC = cos_a1_2*sin_a2_2;
        const double KD = cos_a1_2*cos_a2_2;

        const double r11 = a1_inv*A.m_rotvec[0];
        const double r12 = a1_inv*A.m_rotvec[1];
        const double r13 = a1_inv*A.m_rotvec[2];

        const double r21 = a2_inv*B.m_rotvec[0];
        const double r22 = a2_inv*B.m_rotvec[1];
        const double r23 = a2_inv*B.m_rotvec[2];

        const double q3[] = {
            KD - KA*(r11*r21+r12*r22+r13*r23),
            KC*r21 + KB*r11 + KA*(r22*r13-r23*r12),
            KC*r22 + KB*r12 + KA*(r23*r11-r21*r13),
            KC*r23 + KB*r13 + KA*(r21*r12-r22*r11)
        };

        const double param = 2*acos(q3[0])/sqrt(1-q3[0]*q3[0]);
        this->m_rotvec[0] = param*q3[1];
        this->m_rotvec[1] = param*q3[2];
        this->m_rotvec[2] = param*q3[3];
    }
#endif

    /* */

    // Rotation part
    CPose3D aux(UNINITIALIZED_POSE);
	aux.setRotationMatrix( RA*RB );
    this->m_rotvec = aux.ln_rotation();

//    cout << "WO Approx: " << *this << endl;

    /* */

    // Output Jacobians (if desired)
    if( out_jacobian_drvtC_drvtA || out_jacobian_drvtC_drvtB )
    {
        CPose3DQuat qA,qB,qC;
        this->toQuatXYZ(qC);

        const double &qCr = qC.m_quat[0];
        const double &qCx = qC.m_quat[1];
        const double &qCy = qC.m_quat[2];
        const double &qCz = qC.m_quat[3];

        const double &r1 = this->m_rotvec[0];
        const double &r2 = this->m_rotvec[1];
        const double &r3 = this->m_rotvec[2];

        const double C = 1/(1-qCr*qCr);
        const double D = acos(qCr)/sqrt(1-qCr*qCr);

		MRPT_ALIGN16 const double aux_valsH[] = {
             2*C*qCx*(D*qCr-1), 2*D,    0,      0,
             2*C*qCy*(D*qCr-1), 0,      2*D,    0,
             2*C*qCz*(D*qCr-1), 0,      0,      2*D
		};

        CMatrixFixedNumeric<double,3,4> H(aux_valsH);

        const double alpha  = sqrt(r1*r1+r2*r2+r3*r3);
        const double alpha2 = alpha*alpha;
        const double KA     = alpha*cos(alpha/2)-2*sin(alpha/2);

		MRPT_ALIGN16 const double aux_valsG[] = {
             -r1*alpha2*sin(alpha/2),           -r2*alpha2*sin(alpha/2),            -r3*alpha2*sin(alpha/2),
             2*alpha2*sin(alpha/2)+r1*r1*KA,    r1*r2*KA,                           r1*r3*KA,
             r1*r2*KA,                          2*alpha2*sin(alpha/2)+r2*r2*KA,     r2*r3*KA,
             r1*r3*KA,                          r2*r3*KA,                           2*alpha2*sin(alpha/2)+r3*r3*KA
		};

        CMatrixFixedNumeric<double,4,3> G(aux_valsG);

        if( out_jacobian_drvtC_drvtB )
        {
            A.toQuatXYZ(qA);

            const double &qAr = qA.m_quat[0];
            const double &qAx = qA.m_quat[1];
            const double &qAy = qA.m_quat[2];
            const double &qAz = qA.m_quat[3];

            MRPT_ALIGN16 const double aux_valsQA[] = {qAr, -qAx, -qAy, -qAz, qAx, qAr, qAz, -qAy, qAy, -qAz, qAr, qAx, qAz, qAy, -qAx, qAr};
            CMatrixDouble44 QA(aux_valsQA);

            CMatrixDouble33 jac_rot_B;
            jac_rot_B.multiply_ABC(H,QA,G);

            out_jacobian_drvtC_drvtB->fill(0);
            out_jacobian_drvtC_drvtB->insertMatrix(0,0,jac_rot_B);
            out_jacobian_drvtC_drvtB->insertMatrix(3,3,RA);
        }
        if( out_jacobian_drvtC_drvtA )
        {
            B.toQuatXYZ(qB);

            const double &qBr = qB.m_quat[0];
            const double &qBx = qB.m_quat[1];
            const double &qBy = qB.m_quat[2];
            const double &qBz = qB.m_quat[3];

            MRPT_ALIGN16 const double aux_valsQB[] = {qBr, -qBx, -qBy, -qBz, qBx, qBr, -qBz, qBy, qBy, qBz, qBr, -qBx, qBz, -qBy, qBx, qBr};
            CMatrixDouble44 QB(aux_valsQB);

            CMatrixDouble33 jac_rot_A, id3;
            jac_rot_A.multiply_ABC(H,QB,G);

            id3.eye();
            out_jacobian_drvtC_drvtA->fill(0);
            out_jacobian_drvtC_drvtA->insertMatrix(0,0,jac_rot_A);
            out_jacobian_drvtC_drvtB->insertMatrix(3,3,id3);
        }
    }
} // end composeFrom

/** Convert this pose into its inverse, saving the result in itself. */
void CPose3DRotVec::inverse()
{
    CMatrixDouble44 B_INV(UNINITIALIZED_MATRIX);
	this->getInverseHomogeneousMatrix(B_INV);
	this->setFromTransformationMatrix(B_INV);
}

/** Compute the inverse of this pose and return the result. */
CPose3DRotVec CPose3DRotVec::getInverse() const
{
    CPose3DRotVec inv_rvt(UNINITIALIZED_POSE);
    CMatrixDouble44 B_INV(UNINITIALIZED_MATRIX);
	this->getInverseHomogeneousMatrix(B_INV);
	inv_rvt.setFromTransformationMatrix(B_INV);

	return inv_rvt;
}

/**  Makes \f$ this = A \ominus B \f$ this method is slightly more efficient than "this= A - B;" since it avoids the temporary object.
  *  \note A or B can be "this" without problems.
  * \sa composeFrom, composePoint
  */
void CPose3DRotVec::inverseComposeFrom(const CPose3DRotVec& A, const CPose3DRotVec& B )
{
	// this    =    A  (-)  B
	// HM_this = inv(HM_B) * HM_A
	//
	// [  R_b  | t_b ] -1   [  R_a  | t_a ]    [ R_b^t * Ra |    ..    ]
	// [ ------+-----]    * [ ------+-----]  = [ ---------- +----------]
	// [ 0 0 0 |  1  ]      [ 0 0 0 |  1  ]    [  0  0   0  |      1   ]
	//
    CMatrixDouble44 HM_A(UNINITIALIZED_MATRIX),
                    HM_B_inv(UNINITIALIZED_MATRIX),
                    HM_C(UNINITIALIZED_MATRIX);

    A.getHomogeneousMatrix(HM_A);
    B.getInverseHomogeneousMatrix(HM_B_inv);

    HM_C.multiply_AB(HM_B_inv,HM_A);

    this->m_rotvec      = this->rotVecFromRotMat(HM_C);

    this->m_coords[0]   = HM_C(0,3);
    this->m_coords[1]   = HM_C(1,3);
    this->m_coords[2]   = HM_C(2,3);
}

/**  Computes the 3D point L such as \f$ L = G \ominus this \f$.
  * \sa composePoint, composeFrom
  */
void CPose3DRotVec::inverseComposePoint(const double gx,const double gy,const double gz,double &lx,double &ly,double &lz,
	mrpt::math::CMatrixFixedNumeric<double,3,3>  *out_jacobian_df_dpoint,
	mrpt::math::CMatrixFixedNumeric<double,3,6>  *out_jacobian_df_dpose ) const
{
	MRPT_UNUSED_PARAM(out_jacobian_df_dpoint); MRPT_UNUSED_PARAM(out_jacobian_df_dpose);
    CPose3DRotVec rvt = this->getInverse();
    rvt.composePoint( gx, gy, gz, lx, ly, lz );
    MRPT_TODO("Jacobians");
}

/** Exponentiate a Vector in the SE3 Lie Algebra to generate a new CPose3DRotVec.
  */
CPose3DRotVec CPose3DRotVec::exp(const mrpt::math::CArrayDouble<6> & mu)
{
    return CPose3DRotVec(mu);
}

/** Take the logarithm of the 3x3 rotation matrix, generating the corresponding vector in the Lie Algebra.
  */
CArrayDouble<3> CPose3DRotVec::ln_rotation() const
{
	return m_rotvec;
}


/** Take the logarithm of the 3x4 matrix defined by this pose, generating the corresponding vector in the SE3 Lie Algebra.
  */
void CPose3DRotVec::ln(CArrayDouble<6> &result) const
{
    result.head<3>() = m_coords;
    result.tail<3>() = m_rotvec;
}

void CPose3DRotVec::setToNaN()
{
	for (int i=0;i<3;i++) {
		m_coords[i] = std::numeric_limits<double>::quiet_NaN();
		m_rotvec[i] = std::numeric_limits<double>::quiet_NaN();
	}
}

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers
#include <mrpt/vision/CCamModel.h>
#include <mrpt/vision/pinhole.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/types_math.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;


/**	Constructor */
CCamModel::CCamModel() : cam()
{
}

/**********************************************************************************************************************/
void  CCamModel::jacob_undistor_fm(const mrpt::utils::TPixelCoordf &p, math::CMatrixDouble &J_undist)
{
	// JL: CHECK!!!!
	const double Cx = cam.cx();
	const double Cy = cam.cy();
	const double k1 = cam.k1();
	const double k2 = cam.k2();
	const double dx = 1.0 / cam.fx();   // JL: Check if formulas hold with this change!!!
	const double dy = 1.0 / cam.fy();

	double xd = (p.x-Cx)*dx;
	double yd = (p.y-Cy)*dy;
	double rd2=xd*xd+yd*yd;
	double rd4=rd2*rd2;

	J_undist.setSize(2,2);
	J_undist(0,0) = (1+k1*rd2+k2*rd4) + (p.x-Cx) * (k1+2*k2*rd2) * (2*(p.x-Cx)*dx*dx);
	J_undist(1,1) = (1+k1*rd2+k2*rd4) + (p.y-Cy) * (k1+2*k2*rd2) * (2*(p.y-Cy)*dy*dy);
	J_undist(0,1) = (p.x-Cx) * (k1+2*k2*rd2) * (2*(p.y-Cy)*dy*dy);
	J_undist(1,0) = (p.y-Cy) * (k1+2*k2*rd2) * (2*(p.x-Cx)*dx*dx);

}
/******************************************************************************************************************************/

void CCamModel::jacob_undistor(const mrpt::utils::TPixelCoordf &p, mrpt::math::CMatrixDouble &J_undistor)
{

	//J_undistor.setSize(2,2);
	const double dx = p.x-cam.cx();
	const double dy = p.y-cam.cy();
	const double f = 1 - 2*cam.k1()*square(dx)*square(dy);
	const double inv_f_15 = 1.0/pow(f,1.5);

	//// dy/du
	//CMatrixDouble dy_du(2,2);  // Default is all zeroes
	//dy_du(0,0) = 1.0/cam.fx;
	//dy_du(1,1) = 1.0/cam.fy;

	// du/dh
	//CMatrixDouble du_dh(2,2);

	J_undistor(0,0) = ( 1 - 2*cam.k1()*square(dy) ) * inv_f_15;
	J_undistor(0,1) =
	J_undistor(1,0) = ( 2*cam.k1()*dx*dy ) * inv_f_15;
	J_undistor(1,1) = ( 1 - 2*cam.k1()*square(dx) ) * inv_f_15;


	//// JL: TODO: CHECK!
	//const double Cx = cam.cx;
	//const double Cy = cam.cy;
	//const double k1 = cam.k1();
	//const double k2 = cam.k2();
	//const double dx = 1.0 / cam.fx;   // JL: Check if formulas hold with this change!!!
	//const double dy = 1.0 / cam.fy;

	//double xd = (p.x-Cx)*dx;
	//double yd = (p.y-Cy)*dy;

	//double rd2 = xd*xd + yd*yd;
	//double rd4 = rd2 * rd2;

	//J_undistor.setSize(2,2);

	//J_undistor(0,0) = (1+k1*rd2+k2*rd4) + (p.x-Cx) * (k1+2*k2*rd2) * (2*(p.x-Cx)*dx*dx);
	//J_undistor(1,1) = (1+k1*rd2+k2*rd4) + (p.y-Cy) * (k1+2*k2*rd2) * (2*(p.y-Cy)*dy*dy);
	//J_undistor(0,1) = (p.x-Cx) * (k1+2*k2*rd2) * (2*(p.y-Cy)*dy*dy);
	//J_undistor(1,0) = (p.y-Cy) * (k1+2*k2*rd2) * (2*(p.x-Cx)*dx*dx);



}
/**********************************************************************************************************************/

void  CCamModel::distort_a_point(const mrpt::utils::TPixelCoordf &p, mrpt::utils::TPixelCoordf &distorted_p)
{
	// JLBC: Added from Davison's SceneLib:
	//
	// 1 distortion coefficient model
	//
	const double dx = (p.x-cam.cx()); // / cam.fx;   //JL: commented out cam.fxy... (dx,dy) units must be pixels
	const double dy = (p.y-cam.cy()); // / cam.fy;

	const double r2 = square(dx) + square(dy);

	const double fact = 1.0/sqrt(1+2*cam.k1()*r2);

	distorted_p.x = cam.cx() + dx*fact;
	distorted_p.y = cam.cy() + dy*fact;
	return;
}
/*************************************************************************************************************************/
// Removes distortion of a pair of pixel coordinates x,y.
void  CCamModel::undistort_point(const mrpt::utils::TPixelCoordf &p, mrpt::utils::TPixelCoordf &undistorted_p)
{
	std::vector<TPixelCoordf> in_p(1), out_p;
	in_p[0] = p;

	mrpt::vision::pinhole::undistort_points(
		in_p,
		out_p,
		cam.intrinsicParams,
		cam.getDistortionParamsAsVector()
		);

	ASSERT_(out_p.size()==1);
	undistorted_p = out_p[0];

	// It's explained fine in page 3, "A visual compass based on SLAM"
}


/*************************************************************************************************************************/
/*************************************************************************************************************************/
/**************************************************Davison Style**********************************************************/
/*************************************************************************************************************************/


/**	Return the (distorted) pixel position of a 3D point given in coordinates relative to the camera (+Z pointing forward, +X to the right)
 */
void  CCamModel::project_3D_point(const mrpt::math::TPoint3D &p3D, mrpt::utils::TPixelCoordf &distorted_p) const
{
	// JLBC: From Davison's SceneLib:
	//
	// 1 distortion coefficient model (+ projection)
	//

	// Offsets from the image center for the undistorted projection, in units of pixels:

	ASSERT_(p3D.z!=0)
	const double dx = (p3D.x / p3D.z)*cam.fx();
	const double dy = (p3D.y / p3D.z)*cam.fy();

	// 1 distortion coeff. model:
	const double r2 = square(dx) + square(dy);

	const double fact = 1.0/sqrt(1+2*cam.k1()*r2);   // Note the "+2" sign

	distorted_p.x = cam.cx() + dx*fact;
	distorted_p.y = cam.cy() + dy*fact;
}

/**	Return the 3D location of a point (at a fixed distance z=1), for the given (distorted) pixel position
  */
void  CCamModel::unproject_3D_point(const mrpt::utils::TPixelCoordf &distorted_p, mrpt::math::TPoint3D &p3D) const
{
	// JLBC: From Davison's SceneLib:
	//
	// 1 distortion coefficient model (+ projection)
	//
	const double dx = distorted_p.x - cam.cx();
	const double dy = distorted_p.y - cam.cy();
	const double r2 = square(dx)+square(dy);
	double factor = 1.0/sqrt(1 - 2*cam.k1()*r2);   // Note the "-2" sign

	p3D.x = dx * factor / cam.fx();
	p3D.y = dy * factor / cam.fy();
	p3D.z = 1.0;
}


// Jacobian of the projection of 3D points (with distortion), as done in project_3D_point \f$ \frac{\partial \vct{h}}{\partial \vct{y}} \f$
// JL: See .h file for all the formulas
void CCamModel::jacobian_project_with_distortion(const mrpt::math::TPoint3D &p3D, math::CMatrixDouble & dh_dy ) const
{
	/*
	\frac{\partial \vct{u}}{\partial \vct{y}} =
	\left( \begin{array}{ccc}
	 \frac{f_x}{y_z} &  0 & - y \frac f_x}{y_z^2} \\
	 0 & \frac{f_y}{y_z} & - y \frac f_y}{y_z^2} \\
	\end{array} \right)
	*/
	ASSERT_(p3D.z!=0)

	CMatrixDouble  du_dy(2,3); // Default all to zeroes.

	du_dy(0,0) = cam.fx() / p3D.z;  du_dy(0,2) = - p3D.x * cam.fx() / square(p3D.z);
	du_dy(1,1) = cam.fy() / p3D.z;  du_dy(1,2) = - p3D.y * cam.fy() / square(p3D.z);

	/*
	 f = 1+ 2  k_1  (u_x^2+u_y^2),  then:

	\frac{\partial \vct{h}}{\partial \vct{u}} =
	\left( \begin{array}{cc}
	 \frac{ 1+2 k_1 u_y^2 }{f^{3/2}}  &  -\frac{2 u_x u_y k_1 }{f^{3/2}} \\
	 -\frac{2 u_x u_y k_1 }{f^{3/2}}  & \frac{ 1+2 k_1 u_x^2 }{f^{3/2}}
	\end{array} \right)
	*/
	const double ux = (p3D.x / p3D.z)*cam.fx();  // coordinates with (0,0) at the image center
	const double uy = (p3D.y / p3D.z)*cam.fy();

	const double ux_sqr = square(ux);
	const double uy_sqr = square(uy);

	const double r2 = ux_sqr + uy_sqr;

	const double f = 1+2*cam.k1()*r2;
    const double f1_2 = sqrt(f);
    const double f3_2 = f1_2 * f;

  // Now form the proper dh_by_du by manipulating the outer product matrix

	CMatrixDouble  dh_du(2,2);
	dh_du *= -2 * cam.k1() / f3_2;
	dh_du(0,0) += (1/f1_2);
	dh_du(1,1) += (1/f1_2);

	//dh_du(0,0) = 1+2*cam.k1()*uy_sqr;
	//dh_du(1,1) = 1+2*cam.k1()*ux_sqr;

	//dh_du(0,1) =
	//dh_du(1,0) = -2*ux*uy*cam.k1()/pow(f,1.5);

	// Jacobian dh_dy = dh_du * du_dy   (Result is 2x3)
	dh_dy.multiply( dh_du, du_dy );
}

/* Jacobian of the unprojection of a pixel (with distortion) back into a 3D point, as done in unproject_3D_point \f$ \frac{\partial \vct{y}}{\partial \vct{h}} \f$, evaluated at the pixel p
\note JLBC: Added in March, 2009. Should be equivalent to Davison's WideCamera::ProjectionJacobian
\sa unproject_3D_point
*/
void CCamModel::jacobian_unproject_with_distortion(const mrpt::utils::TPixelCoordf &p, math::CMatrixDouble & dy_dh ) const
{
	// dy/du
	CMatrixDouble dy_du(3,2);  // Default is all zeroes
	dy_du(0,0) = 1.0/cam.fx();
	dy_du(1,1) = 1.0/cam.fy();

	//MAAA:
	//// du/dh
	const double dx = p.x-cam.cx();
	const double dy = p.y-cam.cy();
	const double radi2 = square(dx)+square(dy);

	double f = 1 - 2 * cam.k1() * radi2;
	double f1_2 = sqrt(f);
	double f3_2 = f1_2 * f;

	CMatrixDouble du_dh(2,3);

	//const double f = 1 - 2*cam.k1()*radi2;
	//const double inv_f_15 = 1.0f/powf(f,1.5f);
	//du_dh(0,0) = ( 1 - 2*cam.k1()*square(dy) ) * inv_f_15;
	//du_dh(0,1) =
	//du_dh(1,0) = ( 2*cam.k1()*dx*dy ) * inv_f_15;
	//du_dh(1,1) = ( 1 - 2*cam.k1()*square(dx) ) * inv_f_15;

    du_dh *= 2 * cam.k1() / f3_2;
    du_dh(0,0) += (1/f1_2);
    du_dh(1,1) += (1/f1_2);

	// Jacobian dy_dh = dy_du * du_dh   (Result is 3x2)
	dy_dh.multiply( dy_du, du_dh );

}

void  CCamModel::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase	&source,
	const std::string		&section)
{
	MRPT_START

    // Read camera parameters: They are mandatory, we'll raise an exception if not found:
	double	cx = 0.0f, cy = 0.0f, fx = 0.0f, fy = 0.0f;

	//MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(nrows,int,cam.nrows,    source, section)
	//MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(ncols,int,cam.ncols,    source, section)

	MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(cx,double,cx,    source, section)
	MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(cy,double,cy,    source, section)
	MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(fx,double,fx,    source, section)
	MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(fy,double,fy,    source, section)

	cam.setIntrinsicParamsFromValues( fx, fy, cx, cy );

	CVectorDouble DD;
	source.read_vector(section,"dist_params",CVectorDouble(),DD,true);
	ASSERT_( DD.size()==4 || DD.size()==5 )

	this->cam.setDistortionParamsVector(DD);

	MRPT_END
}

/** This method displays clearly all the contents of the structure in textual form, sending it to a CStream. */
void  CCamModel::dumpToTextStream( CStream		&out) const
{
	MRPT_UNUSED_PARAM(out);
}


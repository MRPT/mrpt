/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/vision.h>  // Precompiled headers
#include <mrpt/vision/CCamModel.h>
#include <mrpt/vision/pinhole.h>
#include <mrpt/utils/CFileOutputStream.h>

using namespace mrpt;
using namespace mrpt::vision;


/**	Constructor */
CCamModel::CCamModel() : cam()
{
}

/**********************************************************************************************************************/
void  CCamModel::jacob_undistor_fm(const mrpt::vision::TPixelCoordf &p, math::CMatrixDouble &J_undist)
{
	// JL: CHECK!!!!
	//THROW_EXCEPTION("TO CHECK")

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


	/*
	double xd=(p.x-cam.cx)*cam.d;
	double yd=(p.y-cam.cy)*cam.d;
	double rd2=xd*xd+yd*yd;
	double rd4=rd2*rd2;

	double uu_ud=(1+cam.k1*rd2+cam.k2*rd4)+(p.x-cam.cx)*(cam.k1+2*cam.k2*rd2)*(2*(p.x-cam.cx)*cam.d*cam.d);
	double vu_vd=(1+cam.k1*rd2+cam.k2*rd4)+(p.y-cam.cy)*(cam.k1+2*cam.k2*rd2)*(2*(p.y-cam.cy)*cam.d*cam.d);

	double uu_vd=(p.x-cam.cx)*(cam.k1+2*cam.k2*rd2)*(2*(p.y-cam.cy)*cam.d*cam.d);
	double vu_ud=(p.y-cam.cy)*(cam.k1+2*cam.k2*rd2)*(2*(p.x-cam.cx)*cam.d*cam.d);

	J_undist.setSize(2,2);
	J_undist(0,0)=uu_ud;	J_undist(0,1)=uu_vd;
	J_undist(1,0)=vu_ud;	J_undist(1,1)=vu_vd;
	*/

}
/******************************************************************************************************************************/

void CCamModel::jacob_undistor(const mrpt::vision::TPixelCoordf &p, mrpt::math::CMatrixDouble &J_undistor)
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

void  CCamModel::distort_a_point(const mrpt::vision::TPixelCoordf &p, mrpt::vision::TPixelCoordf &distorted_p)
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

/*	const double r4 = square(r2);
	const double a1 = 2*x*y;
	const double a2 = r2 + 2*x*x;
	const double a3 = r2 + 2*y*y;
	const double cdist = 1 + cam.k1()*r2 + cam.k2()*r4;
	const double xd = x*cdist + cam.dist[2]*a1 + cam.dist[3]*a2;
	const double yd = y*cdist + cam.dist[2]*a3 + cam.dist[3]*a1;

	distorted_p.x = xd*cam.fx + cam.cx;
	distorted_p.y = yd*cam.fy + cam.cy;
*/

/*    double ru = sqrt(xu*xu + yu*yu);
	double rd = ru / (1 + cam.k1 * ru*ru + cam.k2 * ru*ru*ru*ru);	//initial value for iteration

    //Newton-Rapson. 100 iterations
	double f=0.0, f_p=1.0;
	for (int k=0 ; k<100 ; k++)
	{
        f = rd + cam.k1 * rd*rd*rd + cam.k2 * rd*rd*rd*rd*rd - ru;
		f_p= 1+ 3 * cam.k1 * rd*rd + 5 * cam.k2 * rd*rd*rd*rd;
		rd = rd - f / f_p;
	}

    double D = 1 + cam.k1 * rd*rd + cam.k2*rd*rd*rd*rd;

	uvd.resize(2);
	uvd[0] = ( cam.cx + (xu/D) / cam.d );
	uvd[1] = ( cam.cy + (yu/D) / cam.d );
	*/
}
/*************************************************************************************************************************/
// Removes distortion of a pair of pixel coordinates x,y.
void  CCamModel::undistort_point(const mrpt::vision::TPixelCoordf &p, mrpt::vision::TPixelCoordf &undistorted_p)
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
/*	double Cx = cam.cx;
	double Cy = cam.cy;
	double k1 = cam.k1;
	double k2 = cam.k2;
	double dx = cam.d;
	double dy = cam.d;

	double ud = col;
	double vd = row;
	double rd = sqrt( (dx*(ud-Cx))*(dx*(ud-Cx)) + (dy*(vd-Cy))*(dy*(vd-Cy)) );

	double uu = Cx + ( ud - Cx )*( 1 + k1*rd*rd + k2*rd*rd*rd*rd );
	double vu = Cy + ( vd - Cy )*( 1 + k1*rd*rd + k2*rd*rd*rd*rd );

	col = uu;
	row = vu;*/
}


/*************************************************************************************************************************/
/*************************************************************************************************************************/
/**************************************************Davison Style**********************************************************/
/*************************************************************************************************************************/


/**	Return the (distorted) pixel position of a 3D point given in coordinates relative to the camera (+Z pointing forward, +X to the right)
 */
void  CCamModel::project_3D_point(const mrpt::math::TPoint3D &p3D, mrpt::vision::TPixelCoordf &distorted_p) const
{
	// JLBC: From Davison's SceneLib:
	//
	// 1 distortion coefficient model (+ projection)
	//

	// Offsets from the image center for the undistorted projection, in units of pixels:


  //VNL::VectorFixed<2> imagepos_centred;
  //imagepos_centred[0] = -m_Fku * camera[0] / camera[2];
  //imagepos_centred[1] = -m_Fkv * camera[1] / camera[2];

  //m_last_image_centred = imagepos_centred;

  //// 1 distortion coefficient model
  //const double radius2 = imagepos_centred.SquaredMagnitude();
  //double factor = sqrt(1 + 2 * m_Kd1 * radius2);
  //return imagepos_centred / factor + m_centre;


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
void  CCamModel::unproject_3D_point(const mrpt::vision::TPixelCoordf &distorted_p, mrpt::math::TPoint3D &p3D) const
{
	// JLBC: From Davison's SceneLib:
	//
	// 1 distortion coefficient model (+ projection)
	//

/* MAAA: original source
  centred = image - m_centre;

  m_last_image_centred = centred;

  const double radius2 = centred.SquaredMagnitude();
  double factor = sqrt(1 - 2 * m_Kd1 * radius2);

  VNL::VectorFixed<2> undistorted = centred / factor;

  VNL::VectorFixed<3> camera;

  camera[0] = undistorted[0] / -m_Fku;
  camera[1] = undistorted[1] / -m_Fkv;
  camera[2] = 1.0;

*/

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
  // Jacobians
  // Normal image measurement
  const double fku_yz = m_Fku/m_last_camera[2];
  const double fkv_yz = m_Fkv/m_last_camera[2];
  const double a[6] =
    {-fku_yz,
      0.0,
      fku_yz * m_last_camera[0] / m_last_camera[2],
      0.0,
     -fkv_yz,
      fkv_yz * m_last_camera[1] / m_last_camera[2]};
  const VNL::MatrixFixed<2,3> du_by_dy(a);

  // Distortion model Jacobians
  // Generate the outer product matrix first
  VNL::MatrixFixed<2,2> dh_by_du =
    m_last_image_centred.AsColumn() * m_last_image_centred.AsRow();

  // this matrix is not yet dh_by_du, it is just
  // [ uc*uc  uc*vc ]
  // [ vc*uc  vc*vc ]
  // The trace of this matrix gives the magnitude of the vector
  const double radius2 = dh_by_du(0,0) + dh_by_du(1,1);
  // Calculate various constants to save typing
  const double distor = 1 + 2 * m_Kd1 * radius2;
  const double distor1_2 = sqrt(distor);
  const double distor3_2 = distor1_2 * distor;

  // Now form the proper dh_by_du by manipulating the outer product matrix
  dh_by_du *= -2 * m_Kd1 / distor3_2;
  dh_by_du(0,0) += (1/distor1_2);
  dh_by_du(1,1) += (1/distor1_2);

  return dh_by_du * du_by_dy;

*/
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
void CCamModel::jacobian_unproject_with_distortion(const mrpt::vision::TPixelCoordf &p, math::CMatrixDouble & dy_dh ) const
{

/*
//MAAA: Original Sourcees
 double a[6] = {-1/m_Fku, 0.0,
                 0.0, -1/m_Fkv,
                 0.0, 0.0};
  VNL::MatrixFixed<3,2,double> dy_by_du(a);

  // Generate the outer product matrix first
  VNL::MatrixFixed<2,2> du_by_dh =
    m_last_image_centred.AsColumn() * m_last_image_centred.AsRow();
  // this matrix is not yet du_by_dh, it is just
  // [ uc*uc  uc*vc ]
  // [ vc*uc  vc*vc ]
  // The trace of this matrix gives the magnitude of the vector
  const double radius2 = du_by_dh(0,0) + du_by_dh(1,1);
  // Calculate various constants to save typing
  double distor = 1 - 2 * m_Kd1 * radius2;
  double distor1_2 = sqrt(distor);
  double distor3_2 = distor1_2 * distor;

  // Now form the proper du_by_dh by manipulating the outer product matrix
  du_by_dh *= 2 * m_Kd1 / distor3_2;
  du_by_dh(0,0) += (1/distor1_2);
  du_by_dh(1,1) += (1/distor1_2);

  return dy_by_du * du_by_dh;
*/

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

	vector_double DD;
	source.read_vector(section,"dist_params",vector_double(0),DD,true);
	ASSERT_( DD.size()==4 || DD.size()==5 )

	this->cam.setDistortionParamsVector(DD);

	MRPT_END
}

/** This method displays clearly all the contents of the structure in textual form, sending it to a CStream. */
void  CCamModel::dumpToTextStream( CStream		&out) const
{

}


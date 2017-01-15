/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CCamModel_H
#define CCamModel_H

#include <mrpt/utils/TCamera.h>
#include <mrpt/system/os.h>
#include <mrpt/vision/utils.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/CArrayNumeric.h>

namespace mrpt
{
	namespace vision
	{
		/** This class represent a pinhole camera model for Monocular SLAM and implements some associated Jacobians
		 *
		 *  The camera parameters are accessible in the public member CCamModel::cam
		 *
		 * - Versions:
		 *    - First version: By Antonio J. Ortiz de Galistea.
		 *    - 2009-2010: Rewritten by various authors.
		 *
		 * \sa mrpt::utils::TCamera, CMonoSlam, the application <a href="http://www.mrpt.org/Application:camera-calib-gui" >camera-calib-gui</a> for calibrating a camera
		 * \ingroup mrpt_vision_grp
		 */
		class VISION_IMPEXP  CCamModel : public mrpt::utils::CLoadableOptions
		{
		public:
			mrpt::utils::TCamera cam;  //!< The parameters of a camera

			/** Default Constructor */
			CCamModel();

			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

			/** Constructor from a ini file
			 */
			CCamModel(const mrpt::utils::CConfigFileBase &cfgIni);

			/** Jacobian for undistortion the image coordinates */
			void  jacob_undistor_fm(const mrpt::utils::TPixelCoordf &uvd, math::CMatrixDouble &J_undist);

			/** Calculate the image coordinates undistorted
			 */
			void jacob_undistor(const mrpt::utils::TPixelCoordf &p, mrpt::math::CMatrixDouble &J_undist );

			/**	Return the pixel position distorted by the camera
			 */
			void  distort_a_point(const mrpt::utils::TPixelCoordf &p, mrpt::utils::TPixelCoordf &distorted_p);

			/**	Return the pixel position undistorted by the camera
			 *	The input values 'col' and 'row' will be replace for the new values (undistorted)
			 */
			void  undistort_point(const mrpt::utils::TPixelCoordf &p, mrpt::utils::TPixelCoordf &undistorted_p);

			/**	Return the (distorted) pixel position of a 3D point given in coordinates relative to the camera (+Z pointing forward, +X to the right)
			 * \sa unproject_3D_point
  		     */
			void  project_3D_point(const mrpt::math::TPoint3D &p3D, mrpt::utils::TPixelCoordf &distorted_p) const;

			/**	Return the 3D location of a point (at a fixed distance z=1), for the given (distorted) pixel position
			 * \sa project_3D_point
			 * \note Of course, there is a depth ambiguity, so the returned 3D point must be considered a direction from the camera focus, or a vector, rather than a meaninful physical point.
  		     */
			void  unproject_3D_point(const mrpt::utils::TPixelCoordf &distorted_p, mrpt::math::TPoint3D &p3D) const;

			/** Jacobian of the projection of 3D points (with distortion), as done in project_3D_point \f$ \frac{\partial h}{\partial y} \f$, evaluated at the point p3D (read below the full explanation)

			We define \f$ h = (h_x ~ h_y) \f$ as the projected point in pixels (origin at the top-left corner),
			and \f$ y=( y_x ~ y_y ~ y_z ) \f$ as the 3D point in space, in coordinates relative to the camera (+Z pointing forwards).

			Then this method computes the 2x3 Jacobian:

			\f[
			\frac{\partial h}{\partial y} =  \frac{\partial h}{\partial u} \frac{\partial u}{\partial y}
			\f]

			With:

			\f[
			\frac{\partial u}{\partial y} =
			\left( \begin{array}{ccc}
			 \frac{f_x}{y_z} &  0 & - y \frac{f_x}{y_z^2} \\
			 0 & \frac{f_y}{y_z} & - y \frac{f_y}{y_z^2} \\
			\end{array} \right)
			\f]

			where \f$ f_x, f_y \f$ is the focal length in units of pixel sizes in x and y, respectively.
			And, if we define:

			\f[
			 f = 1+ 2  k_1  (u_x^2+u_y^2)
			\f]

			then:

			\f[
			\frac{\partial h}{\partial u} =
			\left( \begin{array}{cc}
			 \frac{ 1+2 k_1 u_y^2 }{f^{3/2}}  &  -\frac{2 u_x u_y k_1 }{f^{3/2}} \\
			 -\frac{2 u_x u_y k_1 }{f^{3/2}}  & \frac{ 1+2 k_1 u_x^2 }{f^{3/2}}
			\end{array} \right)
			\f]

			\note JLBC: Added in March, 2009. Should be equivalent to Davison's WideCamera::ProjectionJacobian
			\sa project_3D_point
			*/
			void jacobian_project_with_distortion(const mrpt::math::TPoint3D &p3D, math::CMatrixDouble & dh_dy ) const;


			/** Jacobian of the unprojection of a pixel (with distortion) back into a 3D point, as done in unproject_3D_point \f$ \frac{\partial y}{\partial h} \f$, evaluated at the pixel p
			\note JLBC: Added in March, 2009. Should be equivalent to Davison's WideCamera::UnprojectionJacobian
			\sa unproject_3D_point
			*/
			void jacobian_unproject_with_distortion(const mrpt::utils::TPixelCoordf &p, math::CMatrixDouble & dy_dh ) const;

			template<typename T> struct CameraTempVariables	{
				T x_,y_;
				T x_2,y_2;
				T R;
				T K;
				T x__,y__;
			};
			template<typename T,typename POINT> void getTemporaryVariablesForTransform(const POINT &p,CameraTempVariables<T> &v) const	{
				v.x_=p[1]/p[0];
				v.y_=p[2]/p[0];
				v.x_2=square(v.x_);
				v.y_2=square(v.y_);
				v.R=v.x_2+v.y_2;
				v.K=1+v.R*(cam.k1()+v.R*(cam.k2()+v.R*cam.k3()));
				T xy=v.x_*v.y_,p1=cam.p1(),p2=cam.p2();
				v.x__=v.x_*v.K+2*p1*xy+p2*(3*v.x_2+v.y_2);
				v.y__=v.y_*v.K+p1*(v.x_2+3*v.y_2)+2*p2*xy;
			}

			template<typename T,typename POINT,typename PIXEL> inline void getFullProjection(const POINT &pIn,PIXEL &pOut) const	{
				CameraTempVariables<T> tmp;
				getTemporaryVariablesForTransform(pIn,tmp);
				getFullProjectionT(tmp,pOut);
			}

			template<typename T,typename PIXEL> inline void getFullProjectionT(const CameraTempVariables<T> &tmp,PIXEL &pOut) const	{
				pOut[0]=cam.fx()*tmp.x__+cam.cx();
				pOut[1]=cam.fy()*tmp.y__+cam.cy();
			}

			template<typename T,typename POINT,typename MATRIX> inline void getFullJacobian(const POINT &pIn,MATRIX &mOut) const	{
				CameraTempVariables<T> tmp;
				getTemporaryVariablesForTransform(pIn,tmp);
				getFullJacobianT(pIn,tmp,mOut);
			}

			template<typename T,typename POINT,typename MATRIX> void getFullJacobianT(const POINT &pIn,const CameraTempVariables<T> &tmp,MATRIX &mOut) const	{
				T x_=1/pIn[0];
				T x_2=square(x_);
				//First two jacobians...
			 mrpt::math::CMatrixFixedNumeric<T,3,3> J21;
				T tmpK=2*(cam.k1()+tmp.R*(2*cam.k2()+3*tmp.R*cam.k3()));
				T tmpKx=tmpK*tmp.x_;
				T tmpKy=tmpK*tmp.y_;
				T yx2=-pIn[1]*x_2;
				T zx2=-pIn[2]*x_2;
				J21.set_unsafe(0,0,yx2);
				J21.set_unsafe(0,1,x_);
				J21.set_unsafe(0,2,0);
				J21.set_unsafe(1,0,zx2);
				J21.set_unsafe(1,1,0);
				J21.set_unsafe(1,2,x_);
				J21.set_unsafe(2,0,tmpKx*yx2+tmpKy*zx2);
				J21.set_unsafe(2,1,tmpKx*x_);
				J21.set_unsafe(2,2,tmpKy*x_);
				//Last two jacobians...
				T pxpy=2*(cam.p1()*tmp.x_+cam.p2()*tmp.y_);
				T p1y=cam.p1()*tmp.y_;
				T p2x=cam.p2()*tmp.x_;
			 mrpt::math::CMatrixFixedNumeric<T,2,3> J43;
				T fx=cam.fx(),fy=cam.fy();
				J43.set_unsafe(0,0,fx*(tmp.K+2*p1y+6*p2x));
				J43.set_unsafe(0,1,fx*pxpy);
				J43.set_unsafe(0,2,fx*tmp.x_);
				J43.set_unsafe(1,0,fy*pxpy);
				J43.set_unsafe(1,1,fy*(tmp.K+6*p1y+2*p2x));
				J43.set_unsafe(1,2,fy*tmp.y_);
				mOut.multiply(J43,J21);
				//cout<<"J21:\n"<<J21<<"\nJ43:\n"<<J43<<"\nmOut:\n"<<mOut;
			}
		private:
			//These functions are little tricks to avoid multiple initialization.
			//They are intended to initialize the common parts of the jacobians just once,
			//and not in each iteration.
			//They are mostly useless outside the scope of this function.
		 mrpt::math::CMatrixFixedNumeric<double,2,2> firstInverseJacobian() const	{
			 mrpt::math::CMatrixFixedNumeric<double,2,2> res;
				res.set_unsafe(0,1,0);
				res.set_unsafe(1,0,0);
				return res;
			}
		 mrpt::math::CMatrixFixedNumeric<double,4,2> secondInverseJacobian() const	{
			 mrpt::math::CMatrixFixedNumeric<double,4,2> res;
				res.set_unsafe(0,0,1);
				res.set_unsafe(0,1,0);
				res.set_unsafe(1,0,0);
				res.set_unsafe(1,1,1);
				return res;
			}
		 mrpt::math::CMatrixFixedNumeric<double,3,4> thirdInverseJacobian() const	{
			 mrpt::math::CMatrixFixedNumeric<double,3,4> res;
				res.set_unsafe(0,1,0);
				res.set_unsafe(0,2,0);
				res.set_unsafe(1,0,0);
				res.set_unsafe(1,2,0);
				res.set_unsafe(2,0,0);
				res.set_unsafe(2,1,0);
				res.set_unsafe(2,2,1);
				res.set_unsafe(2,3,0);
				return res;
			}
		public:
			template<typename POINTIN,typename POINTOUT,typename MAT22> void getFullInverseModelWithJacobian(const POINTIN &pIn,POINTOUT &pOut,MAT22 &jOut) const	{
				//Temporary variables (well, there are some more, but these are the basics)
				//WARNING!: this shortcut to avoid repeated initialization makes the method somewhat
				//faster, but makes it incapable of being used in more than one thread
				//simultaneously!
				using mrpt::utils::square;
				static mrpt::math::CMatrixFixedNumeric<double,2,2> J1(firstInverseJacobian());
				static mrpt::math::CMatrixFixedNumeric<double,4,2> J2(secondInverseJacobian());
				static mrpt::math::CMatrixFixedNumeric<double,3,4> J3(thirdInverseJacobian());
				static mrpt::math::CMatrixFixedNumeric<double,2,3> J4;	//This is not initialized in a special way, although declaring it
				mrpt::math::CArrayNumeric<double,4> tmp1;
				mrpt::math::CArrayNumeric<double,2> tmp2;	//This would be a CArray<double,3>, but to avoid copying, we let "R2" lie in tmp1.
				//Camera Parameters
				double cx=cam.cx(),cy=cam.cy(),ifx=1/cam.fx(),ify=1/cam.fy();
				double K1=cam.k1(),K2=cam.k2(),p1=cam.p1(),p2=cam.p2(),K3=cam.k3();
				//First step: intrinsic matrix.
				tmp1[0]=(pIn[0]-cx)*ifx;
				tmp1[1]=(pIn[1]-cy)*ify;
				J1.set_unsafe(0,0,ifx);
				J1.set_unsafe(1,1,ify);
				//Second step: adding temporary variables, related to the distortion.
				tmp1[2]=square(tmp1[0])+square(tmp1[1]);
				double sK1=square(K1);
				double K12=sK1-K2;
				double K123=-K1*sK1+2*K1*K2-K3;	//-K1^3+2K1K2-K3
				//tmp1[3]=1-K1*tmp1[2]+K12*square(tmp1[2]);
				tmp1[3]=1+tmp1[2]*(-K1+tmp1[2]*(K12+tmp1[2]*K123));
				J2.set_unsafe(2,0,2*tmp1[0]);
				J2.set_unsafe(2,1,2*tmp1[1]);
				double jTemp=-2*K1+4*tmp1[2]*K12+6*square(tmp1[2])*K123;
				J2.set_unsafe(3,0,tmp1[0]*jTemp);
				J2.set_unsafe(3,1,tmp1[1]*jTemp);
				//Third step: radial distortion. Really simple, since most work has been done in the previous step.
				tmp2[0]=tmp1[0]*tmp1[3];
				tmp2[1]=tmp1[1]*tmp1[3];
				J3.set_unsafe(0,0,tmp1[3]);
				J3.set_unsafe(0,3,tmp1[0]);
				J3.set_unsafe(1,1,tmp1[3]);
				J3.set_unsafe(1,3,tmp1[1]);
				//Fourth step: tangential distorion. A little more complicated, but not much more.
				double prod=tmp2[0]*tmp2[1];
				//References to tmp1[2] are not errors! That element is "R2".
				pOut[0]=tmp2[0]-p1*prod-p2*(tmp1[2]+2*square(tmp2[0]));
				pOut[1]=tmp2[1]-p1*(tmp1[2]+2*square(tmp2[1]))-p2*prod;
				J4.set_unsafe(0,0,1-p1*tmp2[1]-4*p2*tmp2[0]);
				J4.set_unsafe(0,1,-p1*tmp2[0]);
				J4.set_unsafe(0,2,-p2);
				J4.set_unsafe(1,0,-p2*tmp2[1]);
				J4.set_unsafe(1,1,1-4*p1*tmp2[1]-p2*tmp2[0]);
				J4.set_unsafe(1,2,-p1);
				//As fast as possible, and without more temporaries, let the jacobian be J4*J3*J2*J1;
				jOut.multiply_ABC(J4,J3,J2);	//Note that using the other order is not possible due to matrix sizes (jOut may, and most probably will, be fixed).
				jOut.multiply(jOut,J1);
			}

		}; // end class

	} // end namespace
} // end namespace
#endif //__CCamModel_H

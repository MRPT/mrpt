#ifndef CVD_INCLUDE_CAMERA_H
#define CVD_INCLUDE_CAMERA_H

#include <cmath>
#include <TooN/TooN.h>
#include <TooN/helpers.h>

namespace CVD
{
	template<class T>
		inline T SAT(T x){return (x<-1.0/3?-1e9:x);}

	/// Classes which represent camera calibrations.
	/// @ingroup gVision
	namespace Camera {

		/// A linear camera with zero skew
		/// @ingroup gVision
		class Linear {
			public:
				/// The number of parameters in the camera
				static const int num_parameters=4;

				///Load parameters from a stream 
				///@param is The stream to use
				void load(std::istream& is) {
					is >> my_camera_parameters;
				}
				/// Save parameters to a stream 
				///@param os The stream to use
				void save(std::ostream& os) const {
					os << my_camera_parameters;
				}

				/// Fast linear projection for working out what's there
				inline TooN::Vector<2> linearproject(const TooN::Vector<2>& camframe, TooN::DefaultPrecision scale) const {
					return TooN::Vector<2>(scale * diagmult(camframe, my_camera_parameters.slice<0,2>()) + my_camera_parameters.slice<2,2>());
				}
				/// Project from Euclidean camera frame to image plane
				inline TooN::Vector<2> project(const TooN::Vector<2>& camframe) const {
					return TooN::Vector<2>(diagmult(camframe, my_camera_parameters.slice<0,2>()) + my_camera_parameters.slice<2,2>());
				}
				/// Project from image plane to a Euclidean camera
				inline TooN::Vector<2> unproject(const TooN::Vector<2>& imframe) const {
					TooN::Vector<2> my_last_camframe;
					my_last_camframe[0] = (imframe[0]-my_camera_parameters[2])/my_camera_parameters[0];
					my_last_camframe[1] = (imframe[1]-my_camera_parameters[3])/my_camera_parameters[1];
					return my_last_camframe;
				}

				/// Get the derivative of image frame wrt camera frame
				/// in the form \f$ \begin{bmatrix} \frac{\partial \text{im1}}{\partial \text{cam1}} & \frac{\partial \text{im1}}{\partial \text{cam2}} \\ \frac{\partial \text{im2}}{\partial \text{cam1}} & \frac{\partial \text{im2}}{\partial \text{cam2}} \end{bmatrix} \f$
				TooN::Matrix<2,2> get_derivative_at(const TooN::Vector<2>&) const {
					TooN::Matrix<2,2> result;
					result(0,0) = my_camera_parameters[0];
					result(1,1) = my_camera_parameters[1];
					result(0,1) = result(1,0) = 0;
					return result;
				}


				/// Get the component of the motion of a point in the direction provided 
				///	with respect to each of the internal camera parameters
				/// @param direction The (x,y) direction to use
				TooN::Vector<num_parameters> get_parameter_derivs(const TooN::Vector<2>& pos, const TooN::Vector<2>& direction) const {
					TooN::Vector<num_parameters> result;
					result[0] = pos[0] * direction[0];
					result[1] = pos[1] * direction[1];
					result[2] = direction[0];
					result[3] = direction[1];

					return result;
				}

				TooN::Matrix<num_parameters,2> get_parameter_derivs(const TooN::Vector<2>& pos) const {
					TooN::Matrix<num_parameters,2> result;
					result(0,0) = pos[0];
					result(0,1) = 0;
					result(1,0) = 0;
					result(1,1) = pos[1];
					result(2,0) = 1;
					result(2,1) = 0;
					result(3,0) = 0;
					result(3,1) = 1;
					return result;
				}

				/// Update the internal camera parameters by adding the vector given
				/// @param updates Update vector in the format 
				/// \f$ \begin{pmatrix}\Delta f_u & \Delta f_v & \Delta u_0 & \Delta v_0 \end{pmatrix} \f$
				void update(const TooN::Vector<num_parameters>& updates){
					my_camera_parameters+=updates;
				}

				/// Returns the vector of camera parameters in the format
				/// \f$ \begin{pmatrix}f_u & f_v & u_0 & v_0 \end{pmatrix} \f$
				inline TooN::Vector<num_parameters>& get_parameters() {return my_camera_parameters;}
				inline const TooN::Vector<num_parameters>& get_parameters() const {return my_camera_parameters;}

			private:
				TooN::Vector<num_parameters> my_camera_parameters; 
		};


		/// A camera with zero skew and cubic distortion
		/// @ingroup gVision
		class Cubic {
			public:
				/// The number of parameters in the camera
				static const int num_parameters=5;

				///Load parameters from a stream 
				///@param is The stream to use
				inline void load(std::istream& is);
				/// Save parameters to a stream 
				///@param os The stream to use
				inline void save(std::ostream& os) const;

				/// Fast linear projection for working out what's there
				inline TooN::Vector<2> linearproject(const TooN::Vector<2>& camframe, TooN::DefaultPrecision scale=1) const; 
				/// Project from Euclidean camera frame to image plane
				inline TooN::Vector<2> project(const TooN::Vector<2>& camframe) const; 
				/// Project from image plane to a Euclidean camera
				inline TooN::Vector<2> unproject(const TooN::Vector<2>& imframe) const; 

				/// Get the derivative of image frame wrt camera frame at the last computed projection
				/// in the form \f$ \begin{bmatrix} \frac{\partial \text{im1}}{\partial \text{cam1}} & \frac{\partial \text{im1}}{\partial \text{cam2}} \\ \frac{\partial \text{im2}}{\partial \text{cam1}} & \frac{\partial \text{im2}}{\partial \text{cam2}} \end{bmatrix} \f$
				inline TooN::Matrix<2,2> get_derivative_at(const TooN::Vector<2>&) const;


				/// Get the component of the motion of a point in the direction provided 
				///	with respect to each of the internal camera parameters
				/// @param direction The (x,y) direction to use
				inline TooN::Vector<num_parameters> get_parameter_derivs(const TooN::Vector<2>& pos, const TooN::Vector<2>& direction) const ;

				inline TooN::Matrix<Camera::Cubic::num_parameters,2> get_parameter_derivs(const TooN::Vector<2>& pos) const;

				/// Update the internal camera parameters by adding the vector given
				/// @param updates Update vector in the format 
				/// \f$ \begin{pmatrix}\Delta f_u & \Delta f_v & \Delta u_0 & \Delta v_0 & \Delta c\end{pmatrix} \f$
				inline void update(const TooN::Vector<num_parameters>& updates);

				/// Returns the vector of camera parameters in the format
				/// \f$ \begin{pmatrix}f_u & f_v & u_0 & v_0 & c\end{pmatrix} \f$
				inline TooN::Vector<num_parameters>& get_parameters() {return my_camera_parameters;}
				inline const TooN::Vector<num_parameters>& get_parameters() const {return my_camera_parameters;}

			private:
				TooN::Vector<num_parameters> my_camera_parameters; // f_u, f_v, u_0, v_0
		};


		/// A camera with zero skew and quintic distortion
		/// @ingroup gVision
		class Quintic {
			public:
				/// The number of parameters in the camera
				static const int num_parameters=6;

				///Load parameters from a stream 
				///@param is The stream to use
				inline void load(std::istream& is);
				/// Save parameters to a stream 
				///@param os The stream to use
				inline void save(std::ostream& os) const;

				/// Fast linear projection for working out what's there
				inline TooN::Vector<2> linearproject(const TooN::Vector<2>& camframe, TooN::DefaultPrecision scale=1) const ;
				/// Project from Euclidean camera frame to image plane

				inline TooN::Vector<2> project_vector(const TooN::Vector<2>& x, const TooN::Vector<2>& d) const {
					const TooN::DefaultPrecision xsq = x*x;
					const TooN::DefaultPrecision& a = my_camera_parameters[4];
					const TooN::DefaultPrecision& b = my_camera_parameters[5];
					return (2 * (a + 2*b*xsq) * (x*d) * TooN::diagmult(my_camera_parameters.slice<0,2>(), x) +
							(1 + a*xsq + b*xsq*xsq)*TooN::diagmult(my_camera_parameters.slice<0,2>(), d));
				}

				inline TooN::Vector<2> project_vector(const TooN::Vector<2>& d) const {
					return diagmult(my_camera_parameters.slice<0,2>(), d);
				}
				inline TooN::Vector<2> unproject_vector(const TooN::Vector<2>& d) const {
					TooN::Vector<2> v;
					v[0] = d[0]/my_camera_parameters[0];
					v[1] = d[1]/my_camera_parameters[1];
					return v;
				}
				inline TooN::Vector<2> project(const TooN::Vector<2>& camframe) const; 
				inline std::pair<TooN::Vector<2>, TooN::Matrix<2> > project(const TooN::Vector<2>& camframe, const TooN::Matrix<2>& R) const;

				/// Project from image plane to a Euclidean camera
				inline TooN::Vector<2> unproject(const TooN::Vector<2>& imframe) const;

				inline std::pair<TooN::Vector<2>, TooN::Matrix<2> > unproject(const TooN::Vector<2>& imframe, const TooN::Matrix<2>& R) const;

				/// Get the derivative of image frame wrt camera frame at the last computed projection
				/// in the form \f$ \begin{bmatrix} \frac{\partial \text{im1}}{\partial \text{cam1}} & \frac{\partial \text{im1}}{\partial \text{cam2}} \\ \frac{\partial \text{im2}}{\partial \text{cam1}} & \frac{\partial \text{im2}}{\partial \text{cam2}} \end{bmatrix} \f$
				inline TooN::Matrix<2,2> get_derivative() const;
				inline TooN::Matrix<2,2> get_derivative(const TooN::Vector<2>& x) const;

				inline TooN::Matrix<2,2> get_inv_derivative() const;
				inline TooN::Matrix<2,2> get_inv_derivative(const TooN::Vector<2>& x) const;

				/// Get the motion of a point with respect to each of the internal camera parameters
				inline TooN::Matrix<num_parameters,2> get_parameter_derivs() const ;

				/// Get the component of the motion of a point in the direction provided 
				///	with respect to each of the internal camera parameters
				/// @param direction The (x,y) direction to use
				inline TooN::Vector<num_parameters> get_parameter_derivs(const TooN::Vector<2>& direction) const ;

				/// Update the internal camera parameters by adding the vector given
				/// @param updates Update vector in the format 
				/// \f$ \begin{pmatrix}\Delta f_u & \Delta f_v & \Delta u_0 & \Delta v_0 & \Delta c & \Delta q\end{pmatrix} \f$
				inline void update(const TooN::Vector<num_parameters>& updates);

				/// Returns the vector of camera parameters in the format
				/// \f$ \begin{pmatrix}f_u & f_v & u_0 & v_0 & c & q\end{pmatrix} \f$
				inline const TooN::Vector<num_parameters>& get_parameters() const {return my_camera_parameters;}
				/// Returns the vector of camera parameters in the format
				/// \f$ \begin{pmatrix}f_u & f_v & u_0 & v_0 & c & q\end{pmatrix} \f$
				inline TooN::Vector<num_parameters>& get_parameters() {return my_camera_parameters;}

			private:
				TooN::Vector<num_parameters> my_camera_parameters; // f_u, f_v, u_0, v_0
				mutable TooN::Vector<2> my_last_camframe;

				inline TooN::DefaultPrecision sat(TooN::DefaultPrecision x)
				{
					TooN::DefaultPrecision a;
					a = (-3*my_camera_parameters[4] - sqrt(9*my_camera_parameters[4]*my_camera_parameters[4] - 20 * my_camera_parameters[5]))/(10*my_camera_parameters[5]);

					if(x < a)
						return x;
					else
						return 1e9; //(inf)
				}
		};

		///A Camera with zero skew and Harris distortion. The camera distortion model is as follows:
		///
		///\f$
		/// \hat{\rho} = \frac{\rho}{\sqrt{1 + \alpha \rho^2}}
		///\f$
		///
		///This camera has the advantage that inversion of the radial distortion is trivial,
		///and unproject has a unique, closed-form solution. However, the square root makes
		///this camera slower than some of the other models for many use cases.
		///@ingroup gVision
		class Harris{

			private:
				TooN::Vector<2> radial_distort(const TooN::Vector<2>& camframe) const
				{
					TooN::DefaultPrecision r2 = camframe*camframe;
					return camframe / sqrt(1 + my_camera_parameters[4] * r2);
				}


			public:
				/// The number of parameters in the camera
				static const int num_parameters=5;

				///Load parameters from a stream 
				///@param is The stream to use
				inline void load(std::istream& is)
				{
					is >> my_camera_parameters;
				}

				/// Save parameters to a stream 
				///@param os The stream to use
				inline void save(std::ostream& os) const
				{
					os << my_camera_parameters;
				}


				/// Fast linear projection for working out what's there
				inline TooN::Vector<2> linearproject(const TooN::Vector<2>& camframe, TooN::DefaultPrecision scale=1) const
				{
					return TooN::Vector<2>(scale * diagmult(camframe, my_camera_parameters.slice<0,2>()) + my_camera_parameters.slice<2,2>());
				}

				/// Project from Euclidean camera frame to image plane
				inline TooN::Vector<2> project(const TooN::Vector<2>& camframe) const 
				{
					return linearproject(radial_distort(camframe));
				}

				/// Project from image plane to a Euclidean camera
				inline TooN::Vector<2> unproject(const TooN::Vector<2>& imframe) const
				{
					//Undo the focal length and optic axis.
					TooN::Vector<2> mod_camframe;
					mod_camframe[0] = (imframe[0]-my_camera_parameters[2])/my_camera_parameters[0];
					mod_camframe[1] = (imframe[1]-my_camera_parameters[3])/my_camera_parameters[1];
					TooN::DefaultPrecision rprime2 = mod_camframe*mod_camframe;
					return mod_camframe / sqrt(1 - my_camera_parameters[4] * rprime2);
				}

				/// Evaluate the derivative of image frame wrt camera frame at an arbitrary point @p pt.
				/// Get the derivative of image frame wrt camera frame at the last computed projection
				/// in the form \f$ \begin{bmatrix} \frac{\partial \text{im1}}{\partial \text{cam1}} & \frac{\partial \text{im1}}{\partial \text{cam2}} \\ \frac{\partial \text{im2}}{\partial \text{cam1}} & \frac{\partial \text{im2}}{\partial \text{cam2}} \end{bmatrix} \f$
				/// @sa get_derivative()
				inline TooN::Matrix<2,2> get_derivative_at(const TooN::Vector<2>& pt) const
				{
					TooN::Matrix<2,2> J;

					TooN::DefaultPrecision xc = pt[0];
					TooN::DefaultPrecision yc = pt[1];

					TooN::DefaultPrecision fu= my_camera_parameters[0];
					TooN::DefaultPrecision fv= my_camera_parameters[1];
					TooN::DefaultPrecision a = my_camera_parameters[4];

					TooN::DefaultPrecision g = 1/sqrt(1 + a * (xc*xc + yc*yc));
					TooN::DefaultPrecision g3= g*g*g;

					J[0][0] = fu * (g - a * xc*xc*g3);
					J[0][1] = - fu * a * xc * yc * g3;
					J[1][0] = - fv * a * xc * yc * g3;
					J[1][1] = fv * (g - a * yc*yc*g3);

					return J;
				}

				/// Evaluate the derivative of the image coordinates of a given point @p pt in camera
				/// coordinates with respect to each of the internal camera parameters
				inline TooN::Matrix<num_parameters,2> get_parameter_derivs_at(const TooN::Vector<2>& pt) const
				{
					TooN::Vector<2> mod_camframe = radial_distort(pt);

					TooN::Matrix<5, 2> result;

					TooN::DefaultPrecision xc = pt[0];
					TooN::DefaultPrecision yc = pt[1];
					TooN::DefaultPrecision r2 = xc*xc + yc*yc;

					TooN::DefaultPrecision fu= my_camera_parameters[0];
					TooN::DefaultPrecision fv= my_camera_parameters[1];
					TooN::DefaultPrecision a = my_camera_parameters[4];

					TooN::DefaultPrecision g = 1/sqrt(1 + a * r2);
					TooN::DefaultPrecision g3= g*g*g;

					//Derivatives of x_image:
					result[0][0] = mod_camframe[0];
					result[1][0] = 0;
					result[2][0] = 1;
					result[3][0] = 0;
					result[4][0] = - fu * xc * r2 / 2 * g3;

					//Derivatives of y_image:
					result[0][1] = 0;
					result[1][1] = mod_camframe[1];
					result[2][1] = 0;
					result[3][1] = 1;
					result[4][1] = - fv * yc * r2 / 2 * g3;

					return result;
				}

				/// Get the component of the motion of a point in the direction provided 
				///	with respect to each of the internal camera parameters
				/// @param direction The (x,y) direction to use
				inline TooN::Vector<num_parameters> get_parameter_derivs(const TooN::Vector<2>& position, const TooN::Vector<2>& direction) const
				{
					return get_parameter_derivs_at(position) * direction;
				}	

				/// Update the internal camera parameters by adding the vector given
				/// @param updates Update vector in the format 
				/// \f$ \begin{pmatrix}\Delta f_u & \Delta f_v & \Delta u_0 & \Delta v_0 & \Delta c\end{pmatrix} \f$
				//inline void update(const TooN::Vector<num_parameters>& updates);

				/// Returns the vector of camera parameters in the format
				/// \f$ \begin{pmatrix}f_u & f_v & u_0 & v_0 & c\end{pmatrix} \f$
				inline TooN::Vector<num_parameters>& get_parameters() 
				{
					return my_camera_parameters;
				}
				inline const TooN::Vector<num_parameters>& get_parameters() const
				{
					return my_camera_parameters;
				}

			private:
				TooN::Vector<num_parameters> my_camera_parameters; // f_u, f_v, u_0, v_0, alpha
		};

		///This camera class is a very similar to the Harris model.
		///However, it forces the pixel aspect ratio to be unity.
		class SquareHarris{

			private:
				TooN::Vector<2> radial_distort(const TooN::Vector<2>& camframe) const
				{
					TooN::DefaultPrecision r2 = camframe*camframe;
					return camframe / sqrt(1 + my_camera_parameters[4] * r2);
				}


			public:
				/// The number of parameters in the camera
				static const int num_parameters=5;

				///Load parameters from a stream 
				///@param is The stream to use
				inline void load(std::istream& is)
				{
					is >> my_camera_parameters;
				}

				/// Save parameters to a stream 
				///@param os The stream to use
				inline void save(std::ostream& os) const
				{
					os << my_camera_parameters;
				}


				/// Fast linear projection for working out what's there
				inline TooN::Vector<2> linearproject(const TooN::Vector<2>& camframe, TooN::DefaultPrecision scale=1) const
				{
					return TooN::Vector<2>(scale * (camframe* my_camera_parameters[0]) + my_camera_parameters.slice<2,2>());
				}

				/// Project from Euclidean camera frame to image plane
				inline TooN::Vector<2> project(const TooN::Vector<2>& camframe) const 
				{
					my_last_camframe = camframe;
					return linearproject(radial_distort(camframe));
				}

				/// Project from image plane to a Euclidean camera
				inline TooN::Vector<2> unproject(const TooN::Vector<2>& imframe) const
				{
					//Undo the focal length and optic axis.
					TooN::Vector<2> mod_camframe;
					mod_camframe[0] = (imframe[0]-my_camera_parameters[2])/my_camera_parameters[0];
					mod_camframe[1] = (imframe[1]-my_camera_parameters[3])/my_camera_parameters[0];
					TooN::DefaultPrecision rprime2 = mod_camframe*mod_camframe;
					my_last_camframe =  mod_camframe / sqrt(1 - my_camera_parameters[4] * rprime2);
					return my_last_camframe;
				}

				/// Get the derivative of image frame wrt camera frame at the last computed projection
				/// in the form \f$ \begin{bmatrix} \frac{\partial \text{im1}}{\partial \text{cam1}} & \frac{\partial \text{im1}}{\partial \text{cam2}} \\ \frac{\partial \text{im2}}{\partial \text{cam1}} & \frac{\partial \text{im2}}{\partial \text{cam2}} \end{bmatrix} \f$
				inline TooN::Matrix<2,2> get_derivative() const
				{
					TooN::Matrix<2,2> J;

					TooN::DefaultPrecision xc = my_last_camframe[0];
					TooN::DefaultPrecision yc = my_last_camframe[1];

					TooN::DefaultPrecision f= my_camera_parameters[0];
					TooN::DefaultPrecision a = my_camera_parameters[4];

					TooN::DefaultPrecision g = 1/sqrt(1 + a * (xc*xc + yc*yc));
					TooN::DefaultPrecision g3= g*g*g;

					J[0][0] = f * (g - 2 * a * xc*xc*g3);
					J[0][1] = -2 * f * a * xc * yc * g3;
					J[1][0] = -2 * f * a * xc * yc * g3;
					J[1][1] = f * (g - 2 * a * yc*yc*g3);

					return J;
				}

				/// Get the motion of a point with respect to each of the internal camera parameters
				inline TooN::Matrix<num_parameters,2> get_parameter_derivs() const 
				{
					TooN::Vector<2> mod_camframe = radial_distort(my_last_camframe);

					TooN::Matrix<5, 2> result;

					TooN::DefaultPrecision xc = my_last_camframe[0];
					TooN::DefaultPrecision yc = my_last_camframe[1];
					TooN::DefaultPrecision r2 = xc*xc + yc*yc;

					TooN::DefaultPrecision f= my_camera_parameters[0];
					TooN::DefaultPrecision a = my_camera_parameters[4];

					TooN::DefaultPrecision g = 1/sqrt(1 + a * r2);
					TooN::DefaultPrecision g3= g*g*g;

					//Derivatives of x_image:
					result[0][0] = mod_camframe[0];
					result[1][0] = 0;
					result[2][0] = 1;
					result[3][0] = 0;
					result[4][0] = - f * xc * r2 / 2 * g3;



					//Derivatives of y_image:
					result[0][1] = mod_camframe[1];
					result[1][1] = 0;
					result[2][1] = 0;
					result[3][1] = 1;
					result[4][1] = - f * yc * r2 / 2 * g3;

					return result;
				}

				/// Get the component of the motion of a point in the direction provided 
				///	with respect to each of the internal camera parameters
				/// @param direction The (x,y) direction to use
				inline TooN::Vector<num_parameters> get_parameter_derivs(const TooN::Vector<2>& direction) const
				{
					return get_parameter_derivs() * direction;
				}	

				/// Update the internal camera parameters by adding the vector given
				/// @param updates Update vector in the format 
				/// \f$ \begin{pmatrix}\Delta f_u & \Delta f_v & \Delta u_0 & \Delta v_0 & \Delta c\end{pmatrix} \f$
				//inline void update(const TooN::Vector<num_parameters>& updates);

				/// Returns the vector of camera parameters in the format
				/// \f$ \begin{pmatrix}f_u & f_v & u_0 & v_0 & c\end{pmatrix} \f$
				inline TooN::Vector<num_parameters>& get_parameters() 
				{
					my_camera_parameters[1] = 0;
					return my_camera_parameters;
				}

			private:
				TooN::Vector<num_parameters> my_camera_parameters; // f_u, f_v, u_0, v_0, alpha
				mutable TooN::Vector<2> my_last_camframe;
		};

		///A Camera for lenses which attempt to maintain a constant view angle per pixel. The camera distortion model
		///is as follows:
		///\f$
		/// \hat{\rho} = \frac{1}{\omega}\arctan{\omega \rho}
		///\f$
		///
		///This camera is derived from the FOV version described in Devernay and Faugeras, "Straight
		///lines have to be straight" for fish-eye lenses.
		///@ingroup gVision
		class ArcTan{

			private:
				TooN::Vector<2> radial_distort(const TooN::Vector<2>& camframe) const
				{
					const TooN::DefaultPrecision r2 = camframe*camframe;
					const TooN::DefaultPrecision w2 = my_camera_parameters[4] * my_camera_parameters[4];
					const TooN::DefaultPrecision factor = w2*r2;
					TooN::DefaultPrecision term = 1.0;
					TooN::DefaultPrecision scale = term;
					term *= factor;
					scale -= term/3.0;
					term *= factor;
					scale += term/5.0;
					term *= factor;
					scale -= term/7.0;
					return (scale * camframe);
				}


			public:
				/// The number of parameters in the camera
				static const int num_parameters=5;

				///Load parameters from a stream
				///@param is The stream to use
				inline void load(std::istream& is)
				{
					is >> my_camera_parameters;
				}

				/// Save parameters to a stream
				///@param os The stream to use
				inline void save(std::ostream& os) const
				{
					os << my_camera_parameters;
				}

				/// Fast linear projection for working out what's there
				inline TooN::Vector<2> linearproject(const TooN::Vector<2>& camframe, TooN::DefaultPrecision scale=1) const
				{
					return TooN::Vector<2>(scale * diagmult(camframe, my_camera_parameters.slice<0,2>()) + my_camera_parameters.slice<2,2>());
				}

				/// Project from Euclidean camera frame to image plane
				inline TooN::Vector<2> project(const TooN::Vector<2>& camframe) const
				{
					return linearproject(radial_distort(camframe));
				}

				/// Project from image plane to a Euclidean camera
				inline TooN::Vector<2> unproject(const TooN::Vector<2>& imframe) const
				{
					//Undo the focal length and optic axis.
					TooN::Vector<2> mod_camframe;
					mod_camframe[0] = (imframe[0]-my_camera_parameters[2])/my_camera_parameters[0];
					mod_camframe[1] = (imframe[1]-my_camera_parameters[3])/my_camera_parameters[1];
					const TooN::DefaultPrecision rprime2 = mod_camframe*mod_camframe;

					// first guess
					TooN::DefaultPrecision scale = mod_camframe*mod_camframe;

					const TooN::DefaultPrecision w2 = my_camera_parameters[4]* my_camera_parameters[4];
					const TooN::DefaultPrecision k3 = -w2/ 3.0;
					const TooN::DefaultPrecision k5 = w2*w2/ 5.0;
					const TooN::DefaultPrecision k7 = -w2*w2*w2/ 7.0;

					// 3 iterations of Newton-Raphson
					for(int i=0; i<3; i++){
						TooN::DefaultPrecision temp=1+scale*(k3 + scale*(k5 + scale*k7));
						TooN::DefaultPrecision error = rprime2 - scale*temp*temp;
						TooN::DefaultPrecision deriv = temp*(temp+2*scale*(k3 + 2*scale*(k5 + 1.5*scale*k7)));
						scale += error/deriv;
					}
					return mod_camframe/(1+scale*(k3+scale*(k5+scale*k7)));
				}

				/// Evaluate the derivative of image frame wrt camera frame at an arbitrary point @p pt.
				/// in the form \f$ \begin{bmatrix} \frac{\partial \text{im1}}{\partial \text{cam1}} & \frac{\partial \text{im1}}{\partial \text{cam2}} \\ \frac{\partial \text{im2}}{\partial \text{cam1}} & \frac{\partial \text{im2}}{\partial \text{cam2}} \end{bmatrix} \f$
				/// @sa get_derivative()
				inline TooN::Matrix<2,2> get_derivative(const TooN::Vector<2>& pt) const
				{
					TooN::Matrix<2,2> J = TooN::Identity;
					TooN::DefaultPrecision r2=pt*pt;
					const TooN::DefaultPrecision w2 = my_camera_parameters[4]* my_camera_parameters[4];
					const TooN::DefaultPrecision k3 = -w2/ 3.0;
					const TooN::DefaultPrecision k5 = w2*w2/ 5.0;
					const TooN::DefaultPrecision k7 = -w2*w2*w2/ 7.0;
					J *= (1 + k3*r2 + k5*r2*r2 + k7*r2*r2*r2);
					J += ((2*k3 + 4*k5*r2 + 6*k7*r2*r2) * pt.as_col()) * pt.as_row();
					J[0] *= my_camera_parameters[0];
					J[1] *= my_camera_parameters[1];
					return J;
				}


				/// Evaluate the derivative of the image coordinates of a given point @p pt in camera
				/// coordinates with respect to each of the internal camera parameters
				inline TooN::Matrix<num_parameters,2> get_parameter_derivs_at(const TooN::Vector<2>& pt) const
				{
					TooN::Vector<2> mod_camframe = radial_distort(pt);

					TooN::Matrix<5, 2> result;

					const TooN::DefaultPrecision xc = pt[0];
					const TooN::DefaultPrecision yc = pt[1];
					const TooN::DefaultPrecision r2 = xc*xc + yc*yc;
					const TooN::DefaultPrecision r4 = r2*r2;
					const TooN::DefaultPrecision r6 = r4*r2;

					const TooN::DefaultPrecision fu= my_camera_parameters[0];
					const TooN::DefaultPrecision fv= my_camera_parameters[1];

					const TooN::DefaultPrecision w = my_camera_parameters[4];
					const TooN::DefaultPrecision w3 = w*w*w;
					const TooN::DefaultPrecision w5 = w3*w*w;

					const TooN::DefaultPrecision k1 = -(2.0/3.0)*w*r2;
					const TooN::DefaultPrecision k2 = (4.0/5.0)*w3*r4;
					const TooN::DefaultPrecision k3 = -(6.0/7.0)*w5*r6;

					//Derivatives of x_image:
					result[0][0] = mod_camframe[0];
					result[1][0] = 0;
					result[2][0] = 1;
					result[3][0] = 0;
					result[4][0] = fu * (k1 + k2 + k3) * xc;

					//Derivatives of y_image:
					result[0][1] = 0;
					result[1][1] = mod_camframe[1];
					result[2][1] = 0;
					result[3][1] = 1;
					result[4][1] = fv * (k1 + k2 + k3) * yc;

					return result;
				}

				/// Get the component of the motion of a point in the direction provided
				/// with respect to each of the internal camera parameters
				/// @param direction The (x,y) direction to use
				inline TooN::Vector<num_parameters> get_parameter_derivs(const TooN::Vector<2>& pos, const TooN::Vector<2>& direction) const
				{
					return get_parameter_derivs_at(pos) * direction;
				}

				/// Update the internal camera parameters by adding the vector given
				/// @param updates Update vector in the format
				/// \f$ \begin{pmatrix}\Delta f_u & \Delta f_v & \Delta u_0 & \Delta v_0 & \Delta c\end{pmatrix} \f$
				//inline void update(const TooN::Vector<num_parameters>& updates);

				/// Returns the vector of camera parameters in the format
				/// \f$ \begin{pmatrix}f_u & f_v & u_0 & v_0 & c\end{pmatrix} \f$
				inline TooN::Vector<num_parameters>& get_parameters()
				{
					return my_camera_parameters;
				}
				inline const TooN::Vector<num_parameters>& get_parameters() const
				{
					return my_camera_parameters;
				}

			private:
				TooN::Vector<num_parameters> my_camera_parameters; // f_u, f_v, u_0, v_0, omega
		};


		///An adapter to make old-style cameras which remember the last projected point.
		///
		///
		///@ingroup gVision
		template<class C>
			class OldCameraAdapter: public C
		{
			public:
				using C::num_parameters;
				TooN::Vector<2> project(const TooN::Vector<2>& v) const
				{	
					my_last_camframe = v;
					return C::project(v);
				}

				TooN::Matrix<2> get_derivative() const
				{
					return C::get_derivative_at(my_last_camframe);

				}

				TooN::Matrix<num_parameters,2> get_parameter_derivs() const 
				{
					return C::get_parameter_derivs_at(my_last_camframe);
				}

				TooN::Vector<2> unproject(const TooN::Vector<2>& v) const
				{
					my_last_camframe = C::unproject(v);
					return my_last_camframe;
				}

			private:
				mutable TooN::Vector<2> my_last_camframe;
		};

	}






	/////////////////////////////////////
	// Camera::Cubic inline functions //
	/////////////////////////////////////

	void Camera::Cubic::load(std::istream& is) {
		is >> my_camera_parameters;
	}

	void Camera::Cubic::save(std::ostream& os) const {
		os << my_camera_parameters;
	}

	inline TooN::Vector<2> Camera::Cubic::linearproject(const TooN::Vector<2>& camframe, TooN::DefaultPrecision scale) const {
		return TooN::Vector<2>(scale * diagmult(camframe, my_camera_parameters.slice<0,2>()) + my_camera_parameters.slice<2,2>());
	}

	inline TooN::Vector<2> Camera::Cubic::project(const TooN::Vector<2>& camframe) const{
		TooN::Vector<2> mod_camframe = camframe * (1+SAT(my_camera_parameters[4]*(camframe*camframe)));
		return TooN::Vector<2>(diagmult(mod_camframe, my_camera_parameters.slice<0,2>()) + my_camera_parameters.slice<2,2>());
	}

	inline TooN::Vector<2> Camera::Cubic::unproject(const TooN::Vector<2>& imframe) const {
		TooN::Vector<2> mod_camframe;
		mod_camframe[0] = (imframe[0]-my_camera_parameters[2])/my_camera_parameters[0];
		mod_camframe[1] = (imframe[1]-my_camera_parameters[3])/my_camera_parameters[1];

		// first guess
		TooN::DefaultPrecision scale = 1+my_camera_parameters[4]*(mod_camframe*mod_camframe);

		// 3 iterations of Newton-Rapheson
		for(int i=0; i<3; i++){
			TooN::DefaultPrecision error = my_camera_parameters[4]*(mod_camframe*mod_camframe) - scale*scale*(scale-1);
			TooN::DefaultPrecision deriv = (3*scale -2)*scale;
			scale += error/deriv;
		}  
		return mod_camframe/scale;
	}

	TooN::Matrix<2,2> Camera::Cubic::get_derivative_at(const TooN::Vector<2>& pos) const {
		TooN::Matrix<2,2> result = TooN::Identity;
		result *= 1+my_camera_parameters[4]*(pos*pos);
		result += (2*my_camera_parameters[4]*pos.as_col()) * pos.as_row();
		result[0] *= my_camera_parameters[0];
		result[1] *= my_camera_parameters[1];
		return result;
	}

	TooN::Matrix<Camera::Cubic::num_parameters,2> Camera::Cubic::get_parameter_derivs(const TooN::Vector<2>& pos) const {
		TooN::Vector<2> mod_camframe = pos * (1+my_camera_parameters[4]*(pos*pos));
		TooN::Matrix<num_parameters,2> result;
		result(0,0) = mod_camframe[0]*my_camera_parameters[0];
		result(0,1) = 0;
		result(1,0) = 0;
		result(1,1) = mod_camframe[1]*my_camera_parameters[1];
		result(2,0) = 1*my_camera_parameters[0];
		result(2,1) = 0;
		result(3,0) = 0;
		result(3,1) = 1*my_camera_parameters[1];
		result[4] = diagmult(pos,my_camera_parameters.slice<0,2>())*(pos*pos);
		return result;
	}

	TooN::Vector<Camera::Cubic::num_parameters> Camera::Cubic::get_parameter_derivs(const TooN::Vector<2>& pos, const TooN::Vector<2>& direction) const {
		TooN::Vector<2> mod_camframe = pos * (1+my_camera_parameters[4]*(pos*pos));
		TooN::Vector<num_parameters> result;
		result[0] = mod_camframe[0] * direction[0] *my_camera_parameters[0];
		result[1] = mod_camframe[1] * direction[1] *my_camera_parameters[1];
		result[2] = direction[0] *my_camera_parameters[0];
		result[3] = direction[1] *my_camera_parameters[1];
		result[4] = (diagmult(pos,my_camera_parameters.slice<0,2>())*direction)*(pos*pos);
		return result;
	}

	void Camera::Cubic::update(const TooN::Vector<num_parameters>& updates){
		TooN::DefaultPrecision fu=my_camera_parameters[0];
		TooN::DefaultPrecision fv=my_camera_parameters[1];
		my_camera_parameters[0]+=fu*updates[0];
		my_camera_parameters[1]+=fv*updates[1];
		my_camera_parameters[2]+=fu*updates[2];
		my_camera_parameters[3]+=fv*updates[3];
		my_camera_parameters[4]+=updates[4];
		//my_camera_parameters+=updates;
	}

	/////////////////////////////////////
	// Camera::Quintic inline functions //
	/////////////////////////////////////

	void Camera::Quintic::load(std::istream& is) {
		is >> my_camera_parameters;
	}

	void Camera::Quintic::save(std::ostream& os) const {
		os << my_camera_parameters;
	}

	inline TooN::Vector<2> Camera::Quintic::linearproject(const TooN::Vector<2>& camframe, TooN::DefaultPrecision scale) const {
		return TooN::Vector<2>(scale * diagmult(camframe, my_camera_parameters.slice<0,2>()) + my_camera_parameters.slice<2,2>());
	}

	inline TooN::Vector<2> Camera::Quintic::project(const TooN::Vector<2>& camframe) const {
		my_last_camframe = camframe;
		TooN::DefaultPrecision sc = /*sat*/(camframe*camframe);
		TooN::Vector<2> mod_camframe = camframe * (1 + sc*(my_camera_parameters[4] + sc*my_camera_parameters[5]));
		return TooN::Vector<2>(diagmult(mod_camframe, my_camera_parameters.slice<0,2>()) + my_camera_parameters.slice<2,2>());
	}

	inline std::pair<TooN::Vector<2>, TooN::Matrix<2> > Camera::Quintic::project(const TooN::Vector<2>& camframe, const TooN::Matrix<2>& R) const
	{
		std::pair<TooN::Vector<2>, TooN::Matrix<2> > result;
		result.first = this->project(camframe);
		const TooN::Matrix<2> J = this->get_derivative();
		result.second = J * R * J.T();
		return result;
	}

	inline TooN::Vector<2> Camera::Quintic::unproject(const TooN::Vector<2>& imframe) const {
		TooN::Vector<2> mod_camframe;
		mod_camframe[0] = (imframe[0]-my_camera_parameters[2])/my_camera_parameters[0];
		mod_camframe[1] = (imframe[1]-my_camera_parameters[3])/my_camera_parameters[1];

		// first guess
		TooN::DefaultPrecision scale = mod_camframe*mod_camframe;

		// 3 iterations of Newton-Rapheson
		for(int i=0; i<3; i++){
			TooN::DefaultPrecision temp=1+scale*(my_camera_parameters[4]+my_camera_parameters[5]*scale);
			TooN::DefaultPrecision error = mod_camframe*mod_camframe - scale*temp*temp;
			TooN::DefaultPrecision deriv = temp*(temp+2*scale*(my_camera_parameters[4]+2*my_camera_parameters[5]*scale));
			scale += error/deriv;
		}  
		my_last_camframe = mod_camframe/(1+scale*(my_camera_parameters[4]+my_camera_parameters[5]*scale));

		//std::cout<<"Done inverse on "<<imframe<<" - when reprojected get "<<project(my_last_camframe)<<std::endl;

		return my_last_camframe;
	}

	inline std::pair<TooN::Vector<2>, TooN::Matrix<2> > Camera::Quintic::unproject(const TooN::Vector<2>& imframe, const TooN::Matrix<2>& R) const
	{
		std::pair<TooN::Vector<2>, TooN::Matrix<2> > result;
		result.first = this->unproject(imframe);
		TooN::Matrix<2> J = get_derivative();
		TooN::DefaultPrecision rdet = 1/ (J[0][0] * J[1][1] - J[0][1] * J[1][0]);
		TooN::Matrix<2> Jinv;
		Jinv[0][0] = rdet * J[1][1];
		Jinv[1][1] = rdet * J[0][0];
		Jinv[0][1] = -rdet * J[0][1];
		Jinv[1][0] = -rdet * J[1][0];    
		result.second = Jinv * R * Jinv.T();
		return result;
	}


	TooN::Matrix<2,2> Camera::Quintic::get_derivative() const {
		TooN::Matrix<2,2> result = TooN::Identity;
		TooN::DefaultPrecision temp1=my_last_camframe*my_last_camframe;
		TooN::DefaultPrecision temp2=my_camera_parameters[5]*temp1;
		result *= 1+temp1*(my_camera_parameters[4]+temp2);
		result += (2*(my_camera_parameters[4]+2*temp2)*my_last_camframe.as_col()) * my_last_camframe.as_row();
		result[0] *= my_camera_parameters[0];
		result[1] *= my_camera_parameters[1];
		return result;
	}



	TooN::Matrix<2,2> Camera::Quintic::get_inv_derivative() const {
		TooN::Matrix<2,2> result = TooN::Identity;
		TooN::DefaultPrecision temp1=my_last_camframe*my_last_camframe;
		TooN::DefaultPrecision temp2=my_camera_parameters[5]*temp1;
		TooN::DefaultPrecision temp3=2*(my_camera_parameters[4]+2*temp2);

		result *= 1+temp1*(my_camera_parameters[4]+temp2);

		result[0][0] +=  my_last_camframe[1]*my_last_camframe[1]*temp3;
		result[0][1]  =-(temp3*my_last_camframe[0]*my_last_camframe[1]);

		result[1][1] +=  my_last_camframe[0]*my_last_camframe[0]*temp3;
		result[1][0]  =-(temp3*my_last_camframe[0]*my_last_camframe[1]);

		(result.T())[0] *= my_camera_parameters[1];
		(result.T())[1] *= my_camera_parameters[0];

		result /= (result[0][0]*result[1][1] - result[1][0]*result[0][1]);

		return result;
	}

	TooN::Matrix<2,2> Camera::Quintic::get_inv_derivative(const TooN::Vector<2>& x) const
	{

		TooN::Matrix<2,2> result = TooN::Identity;
		TooN::DefaultPrecision temp1=x*x;
		TooN::DefaultPrecision temp2=my_camera_parameters[5]*temp1;
		TooN::DefaultPrecision temp3=2*(my_camera_parameters[4]+2*temp2);

		result *= 1+temp1*(my_camera_parameters[4]+temp2);
		//Identity(result,1+temp1*(my_camera_parameters[4]+temp2));

		result[0][0] +=  x[1]*x[1]*temp3;
		result[0][1]  =-(temp3*x[0]*x[1]);

		result[1][1] +=  x[0]*x[0]*temp3;
		result[1][0]  =-(temp3*x[0]*x[1]);

		(result.T())[0] *= my_camera_parameters[1];
		(result.T())[1] *= my_camera_parameters[0];

		result /= (result[0][0]*result[1][1] - result[1][0]*result[0][1]);

		return result;

	}

	TooN::Matrix<2,2> Camera::Quintic::get_derivative(const TooN::Vector<2>& x) const {
		TooN::Matrix<2,2> result = TooN::Identity;
		TooN::DefaultPrecision temp1=x*x;
		TooN::DefaultPrecision temp2=my_camera_parameters[5]*temp1;
		result *= 1+temp1*(my_camera_parameters[4]+temp2);
		//Identity(result,1+temp1*(my_camera_parameters[4]+temp2));
		result += (2*(my_camera_parameters[4]+2*temp2)*x.as_col()) * x.as_row();
		result[0] *= my_camera_parameters[0];
		result[1] *= my_camera_parameters[1];
		return result;
	}

	TooN::Matrix<Camera::Quintic::num_parameters,2> Camera::Quintic::get_parameter_derivs() const {
		TooN::Matrix<num_parameters,2> result;
		TooN::DefaultPrecision r2 = my_last_camframe * my_last_camframe;
		TooN::DefaultPrecision r4 = r2 * r2;
		TooN::Vector<2> mod_camframe = my_last_camframe * (1+ r2 * (my_camera_parameters[4] + r2 * my_camera_parameters[5]));

		result(0,0) = mod_camframe[0];
		result(1,0) = 0;
		result(2,0) = 1;
		result(3,0) = 0;
		result(4,0) = my_camera_parameters[0]*my_last_camframe[0]*r2;
		result(5,0) = my_camera_parameters[0]*my_last_camframe[0]*r4;

		result(0,1) = 0;
		result(1,1) = mod_camframe[1];
		result(2,1) = 0;
		result(3,1) = 1;
		result(4,1) = my_camera_parameters[1]*my_last_camframe[1]*r2;
		result(5,1) = my_camera_parameters[1]*my_last_camframe[1]*r4;

		//cout<<"Finish me!\n";					     
		return result;
	}

	TooN::Vector<Camera::Quintic::num_parameters> Camera::Quintic::get_parameter_derivs(const TooN::Vector<2>& direction) const {
		//TooN::Vector<num_parameters> result;
		//cout<<"Finish me!\n";					     
		//FIXME improve this somewhat
		return get_parameter_derivs() * direction;
	}

	void Camera::Quintic::update(const TooN::Vector<num_parameters>& updates){
		TooN::DefaultPrecision fu = my_camera_parameters[0];
		TooN::DefaultPrecision fv = my_camera_parameters[1];

		my_camera_parameters[0]+=updates[0]*fu;
		my_camera_parameters[1]+=updates[1]*fv;
		my_camera_parameters[2]+=updates[2]*fu;
		my_camera_parameters[3]+=updates[3]*fv;
		my_camera_parameters[4]+=updates[4];
		my_camera_parameters[5]+=updates[5];
	}
}
#endif

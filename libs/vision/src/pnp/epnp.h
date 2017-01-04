/* +---------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)               |
|                          http://www.mrpt.org/                             |
|                                                                           |
| Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
| See: http://www.mrpt.org/Authors - All rights reserved.                   |
| Released under BSD License. See details in http://www.mrpt.org/License    |
+---------------------------------------------------------------------------+ */

#ifndef _mrpt_epnp
#define _mrpt_epnp
#include <mrpt/otherlibs/do_opencv_includes.h>

#if MRPT_HAS_OPENCV

    namespace mrpt
    {
        namespace vision
        {
            namespace pnp
            {
                /** \addtogroup pnp Perspective-n-Point pose estimation
                 *  \ingroup mrpt_vision_grp
                 *  @{  
                 */
                 
                /**
                 * @class epnp
                 * @author Chandra Mangipudi
                 * @date 11/08/16
                 * @file epnp.h
                 * @brief Efficient PnP - Eigen Wrapper for OpenCV calib3d implementation
                 */
                class epnp 
                {
                    public:
                          //! Constructor for EPnP class
                          epnp(const cv::Mat& cameraMatrix, const cv::Mat& opoints, const cv::Mat& ipoints);
                          
                          //! Destructor for EPnP class
                          ~epnp();
                          
                          /**
                           * @brief Add a 2d/3d correspondence
                           * @param[in] X X coordinate in Camera coordinate system
                           * @param[in] Y Y coordinate in Camera coordinate system
                           * @param[in] Z Z coordinate in Camera coordinate system
                           * @param[in] u Image pixel coordinate u in x axis
                           * @param[in] v Image pixel coordinate v in y axis
                           */
                          void add_correspondence(const double X, const double Y, const double Z,
                                      const double u, const double v);

                          /**
                           * @brief OpenCV wrapper to compute pose
                           * @param[out] R Rotation Matrix
                           * @param[out] t Translation Vector
                           */
                          void compute_pose(cv::Mat& R, cv::Mat& t);
                    private:
                         /**
                          * @brief Initialize Camera Matrix
                          * @param[in] cameraMatrix Camera Intrinsic matrix as a OpenCV Matrix
                          */
                          template <typename T>
                          void init_camera_parameters(const cv::Mat& cameraMatrix)
                          {
                            uc = cameraMatrix.at<T> (0, 2);
                            vc = cameraMatrix.at<T> (1, 2);
                            fu = cameraMatrix.at<T> (0, 0);
                            fv = cameraMatrix.at<T> (1, 1);
                          }
                          
                          /**
                           * @brief Convert object points and image points from OpenCV format to STL matrices
                           * @param opoints Object points in Camera coordinate system
                           * @param ipoints Imate points in pixel coordinates
                           */
                          template <typename OpointType, typename IpointType>
                          void init_points(const cv::Mat& opoints, const cv::Mat& ipoints)
                          {
                              for(int i = 0; i < number_of_correspondences; i++)
                              {
                                  pws[3 * i    ] = opoints.at<OpointType>(0,i).x;
                                  pws[3 * i + 1] = opoints.at<OpointType>(0,i).y;
                                  pws[3 * i + 2] = opoints.at<OpointType>(0,i).z;

                                  us[2 * i    ] = ipoints.at<IpointType>(0,i).x*fu + uc;
                                  us[2 * i + 1] = ipoints.at<IpointType>(0,i).y*fv + vc;
                              }
                          }
                          
                          /**
                           * @brief Function to compute reprojection error
                           * @param R Rotation Matrix
                           * @param t Translation Vector
                           * @return 
                           */
                          double reprojection_error(const double R[3][3], const double t[3]);
                          
                          /**
                           * @brief Function to select 4 control points from n points
                           */
                          void choose_control_points(void);
                          
                          /**
                           * @brief Convert from object space to relative object space (Barycentric coordinates)
                           */
                          void compute_barycentric_coordinates(void);
                          
                          /**
                           * @brief Generate the Matrix M
                           * @param[out] M
                           * @param[in] row
                           * @param[in] alphas
                           * @param[in] u
                           * @param[in] v
                           */
                          void fill_M(CvMat * M, const int row, const double * alphas, const double u, const double v);
                          
                          /**
                           * @brief Internal function
                           * @param[in] betas
                           * @param[in] ut
                           */
                          void compute_ccs(const double * betas, const double * ut);
                          
                          /**
                           * @brief Internal function
                           */
                          void compute_pcs(void);
                          
                          
                          /**
                           * @brief Internal function
                           */
                          void solve_for_sign(void);

                          /**
                           * @brief Internal function
                           * @param[out] L_6x10
                           * @param[in] Rho
                           * @param[in] betas
                           */
                          void find_betas_approx_1(const CvMat * L_6x10, const CvMat * Rho, double * betas);
                          
                          /**
                           * @brief Internal function
                           * @param[out] L_6x10
                           * @param[in] Rho
                           * @param[in] betas
                           */
                          void find_betas_approx_2(const CvMat * L_6x10, const CvMat * Rho, double * betas);
                          
                          /**
                           * @brief Internal function
                           * @param[out] L_6x10
                           * @param[in] Rho
                           * @param[in] betas
                           */
                          void find_betas_approx_3(const CvMat * L_6x10, const CvMat * Rho, double * betas);
                          
                          /**
                           * @brief QR optimization algorithm
                           * @param[in] A
                           * @param[out] b
                           * @param[out] X
                           */
                          void qr_solve(CvMat * A, CvMat * b, CvMat * X);

                          /**
                           * @brief Dot product of two OpenCV vectors
                           * @param[in] v1
                           * @param[in] v2
                           * @return 
                           */
                          double dot(const double * v1, const double * v2);
                          
                          /**
                           * @brief Squared distance between two vectors
                           * @param[in] p1
                           * @param[in] p2
                           * @return 
                           */
                          double dist2(const double * p1, const double * p2);

                          /**
                           * @brief Get distances between all object points taken 2 at a time(nC2)
                           * @param rho
                           */
                          void compute_rho(double * rho);
                          
                          /**
                           * @brief Internal function
                           * @param[in] ut
                           * @param[out] l_6x10
                           */
                          void compute_L_6x10(const double * ut, double * l_6x10);

                          /**
                           * @brief Gauss Newton iterative algorithm
                           * @param[in] L_6x10
                           * @param[in] Rho
                           * @param[in,out] current_betas
                           */
                          void gauss_newton(const CvMat * L_6x10, const CvMat * Rho, double current_betas[4]);
                          
                          /**
                           * @brief Internal function
                           * @param[in] l_6x10
                           * @param[in] rho
                           * @param[in] cb
                           * @param[out] A
                           * @param[out] b
                           */
                          void compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,
                                            const double cb[4], CvMat * A, CvMat * b);

                          /**
                           * @brief Function to compute pose
                           * @param[in] ut
                           * @param[in] betas
                           * @param[out] R
                           * @param[out] t
                           * @return 
                           */
                          double compute_R_and_t(const double * ut, const double * betas,
                                     double R[3][3], double t[3]);

                          /**
                           * @brief Helper function to @func compute_R_and_t()
                           * @param R
                           * @param t
                           */
                          void estimate_R_and_t(double R[3][3], double t[3]);

                          /**
                           * @brief Copy function of output result
                           * @param[out] R_dst
                           * @param[out] t_dst
                           * @param[in] R_src
                           * @param[in] t_src
                           */
                          void copy_R_and_t(const double R_dst[3][3], const double t_dst[3],
                                    double R_src[3][3], double t_src[3]);


                          double uc; //! Image center in x-direction
                          double vc; //! Image center in y-direction
                          double fu; //! Focal length in x-direction
                          double fv; //! Focal length in y-direction

                          std::vector<double> pws, us, alphas, pcs; //! Internal member variables
                          int number_of_correspondences; //! Number of 2d/3d correspondences

                          double cws[4][3], ccs[4][3]; //! Internal member variables
                          double cws_determinant; //! Internal member variable
                          int max_nr; //! Internal member variable
                          double * A1, * A2; //! Internal member variables
                };
            
                /** @}  */ // end of grouping
                
            }
        }
    }
#endif
#endif

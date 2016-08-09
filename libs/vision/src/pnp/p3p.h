#ifndef P3P_H
#define P3P_H


#include <Eigen/Dense>
#include <Eigen/SVD>

#if MRPT_HAS_OPENCV
    #include <opencv2/opencv.hpp>
#endif

namespace mrpt
{
    namespace vision
    {
        class p3p
        {
         public:
          p3p(double fx, double fy, double cx, double cy);
          p3p(Eigen::MatrixXd cam_intrinsic);

          #if MRPT_HAS_OPENCV
            p3p(cv::Mat cameraMatrix);
            bool solve(cv::Mat& R, cv::Mat& tvec, const cv::Mat& opoints, const cv::Mat& ipoints);
          #endif
          bool solve(Eigen::Ref<Eigen::Matrix3d> R, Eigen::Ref<Eigen::Vector3d> t, Eigen::MatrixXd obj_pts, Eigen::MatrixXd img_pts);
          
          int solve(double R[4][3][3], double t[4][3],
                    double mu0, double mv0,   double X0, double Y0, double Z0,
                    double mu1, double mv1,   double X1, double Y1, double Z1,
                    double mu2, double mv2,   double X2, double Y2, double Z2);
          bool solve(double R[3][3], double t[3],
                     double mu0, double mv0,   double X0, double Y0, double Z0,
                     double mu1, double mv1,   double X1, double Y1, double Z1,
                     double mu2, double mv2,   double X2, double Y2, double Z2,
                     double mu3, double mv3,   double X3, double Y3, double Z3);

         private:
          #if MRPT_HAS_OPENCV
              template <typename T>
              void init_camera_parameters(const cv::Mat& cameraMatrix)
              {
                cx = cameraMatrix.at<T> (0, 2);
                cy = cameraMatrix.at<T> (1, 2);
                fx = cameraMatrix.at<T> (0, 0);
                fy = cameraMatrix.at<T> (1, 1);
              }
              template <typename OpointType, typename IpointType>
              void extract_points(const cv::Mat& opoints, const cv::Mat& ipoints, std::vector<double>& points)
              {
                  points.clear();
                  points.resize(20);
                  for(int i = 0; i < 4; i++)
                  {
                      points[i*5] = ipoints.at<IpointType>(i).x*fx + cx;
                      points[i*5+1] = ipoints.at<IpointType>(i).y*fy + cy;
                      points[i*5+2] = opoints.at<OpointType>(i).x;
                      points[i*5+3] = opoints.at<OpointType>(i).y;
                      points[i*5+4] = opoints.at<OpointType>(i).z;
                  }
              }
          #endif
          void extract_points(Eigen::MatrixXd obj_pts, Eigen::MatrixXd img_pts, std::vector<double>& points)
          {
              points.clear();
              points.resize(20);
              for(int i=0; i<4; i++)
              {
                  points[i*5] = img_pts(i,0)*fx + cx;
                  points[i*5+1]=img_pts(i,1)*fy + cy;
                  points[i*5+2]=obj_pts(i,0);
                  points[i*5+3]=obj_pts(i,1);
                  points[i*5+4]=obj_pts(i,2);
              }
          }
          void init_inverse_parameters();
          int solve_for_lengths(double lengths[4][3], double distances[3], double cosines[3]);
          bool align(double M_start[3][3],
                     double X0, double Y0, double Z0,
                     double X1, double Y1, double Z1,
                     double X2, double Y2, double Z2,
                     double R[3][3], double T[3]);

          bool jacobi_4x4(double * A, double * D, double * U);

          double fx, fy, cx, cy;
          double inv_fx, inv_fy, cx_fx, cy_fy;
        };
    }
}

#endif // P3P_H

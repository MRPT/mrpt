#ifndef CVD_ESM_H_
#define CVD_ESM_H_

#include <TooN/wls.h>
#include <TooN/so3.h>

#include <cvd/image.h>
#include <cvd/image_io.h>
#include <cvd/vector_image_ref.h>
#include <cvd/vision.h>

#include <vector>
#include <sstream>
#include <iomanip>

namespace CVD {

static int DEBUG_ESM = 0;

/// @defgroup gEsm Efficient Second Order Minimization (ESM)
/// This module implements the ESM template tracking algorithm for homography-based
/// image transformations as described by Benhimane & Mails, 
/// "Real-time image-based tracking of planes using efficient second-order minimization", 2004.
///
/// Within this module the geometric transformation of the template and the radiometric (appearance) 
/// transformation are separated into two different concepts. Concrete type instances are used to 
/// parameterize the generic @ref ESMEstimator class. The following are the two important concepts.
/// 
/// The transform object @ref T has to implement the following concept:
/// @code
/// class Transform {
///     static const int dimensions = X;
///     Matrix<3> get_matrix(); 
///     Vector<dimensions> get_jacobian( const Vector<2> & point, const Vector<2> & gradient );
///     void update( const Vector<dimensions> & delta );
/// };
/// @endcode
/// The implementations are @ref Homography, @ref HomographyPrefix and @ref CameraRotation.
/// 
/// A second object deals with the appearance model used and has to implement 
/// the following concept:
/// @code
/// class Appearance {
///     static const int dimensions = X;
///     template <typename PIXEL> double difference( const PIXEL & warped, const PIXEL & templatePixel );
///     template <typename PIXEL> tuple<double, Vector<dimensions> > difference_jacobian( const PIXEL & warped, const PIXEL & templatePixel );
///     Vector<2> image_jacobian( const Vector<2> & gradWarped, const Vector<2> & gradTemplate );
///     void update( const Vector<dimensions> & delta );
/// };
/// @endcode
/// The implementations are @ref StaticAppearance, @ref OffsetAppearance and @ref BlurAppearance.
///
/// @ingroup gVision

/// Result class storing some information about the optimization
/// @ingroup gEsm
struct ESMResult {
    double error;   ///< final total squared error 
    double delta;   ///< norm of the last update
    int pixels;     ///< common pixels in last iteration
    int iterations; ///< number of iterations performed
    
    /// returns RMSE error computed from @ref error and @pixels
    double RMSE() const { return std::sqrt(error/std::max(1,pixels)); }
};

inline std::ostream & operator<<(std::ostream & out, const ESMResult & r ){
    out << r.error << " " << r.pixels << " " << r.RMSE() << " " << r.delta << " " << r.iterations;
    return out;
}

namespace Internal { // forward declaration of some internal functions
    // intersects the line AB with the rectangle rect, returns the interval for the parameter t
    // for X = A + (B-A)*t
    // A, B in homogeneous coordinates, yielding the correct perspective interpolation for t
    inline TooN::Vector<2> getBounds(const TooN::Vector<2> & rect, const TooN::Vector<3> & A, const TooN::Vector<3> & B);
    inline std::vector<TooN::Vector<2,int> > getBounds( const TooN::Vector<2> & rect, const ImageRef & in, const TooN::Matrix<3> & H );
    inline std::vector<TooN::Vector<2,int> > erode( const std::vector<TooN::Vector<2,int> > & in );

    // H takes pixels in out to pixels in in as a 2D homography (3x3 matrix)
    template <typename T> inline std::vector<TooN::Vector<2,int> > transform_perspective( BasicImage<T> & out, const BasicImage<T> & in, const TooN::Matrix<3> & H);
    // H takes pixels in out to pixels in in as a 2D affine transformation (3x3 matrix)
    template <typename T> inline std::vector<TooN::Vector<2,int> > transform_affine( BasicImage<T> & out, const BasicImage<T> & in, const TooN::Matrix<3> & H);
    // H takes pixels in out to pixels in in as a 2D translation only (stored in the 3x3 matrix)
    // This is extra optimized to use the constant mixing factors in the bi-linear interpolation
    template <typename T> inline std::vector<TooN::Vector<2,int> > transform_translation( BasicImage<T> & out, const BasicImage<T> & in, const TooN::Matrix<3> & H);
    // automatically selects the right transform implementation based on the properties of H
    template <typename T> inline std::vector<TooN::Vector<2,int> > transform( BasicImage<T> & out, const BasicImage<T> & in, const TooN::Matrix<3> & H);
    
    // calculates the gradient of a one component image within the given bounds only. The 
    // function uses central differences, but does not divide by 2!
    template<typename GRADIENT, typename IMAGE> inline Image<GRADIENT> gradient( const BasicImage<IMAGE> & img, const std::vector<TooN::Vector<2,int> > & bounds );
    // calculates the gradient of a one component image directly. The 
    // function uses central differences, but does not divide by 2!
    template<typename GRADIENT, typename IMAGE> inline Image<GRADIENT> gradient( const BasicImage<IMAGE> & img );
    
    /// a full ESM optimization function. It takes a template image and its gradient, a target image and a 
    /// general transform object, plus some parameters and iterates until convergence.
    /// @param T the geometric transformation, used as output as well
    /// @param A the appearance transformation, used as output as well
    /// @param templateImage the reference template image
    /// @param templateGradient and its gradient of the same size
    /// @param target the target image, can be any size
    /// @param max_iterations maximal number of iterations to perform
    /// @param min_delta minimal change vector, if the vector is smaller than this, assume convergence and stop
    /// @result struct carrying the convergence data of the optimization
    ///
    /// @ingroup gEsm
    template <typename TRANSFORM, typename APPEARANCE, typename IMAGE, typename GRADIENT>
    inline ESMResult esm_opt( TRANSFORM & T, APPEARANCE & A, const BasicImage<IMAGE> & templateImage, const BasicImage<GRADIENT> & templateGradient, const BasicImage<IMAGE> & target, const int max_iterations = 40, const double min_delta = 1e-8, const double max_RMSE = 1.0 );                
} // namespace Internal

/// a generic implementation for 2D homography-based transformations parameterized
/// by a 3x3 matrix with determinant == 1. The class implements the interface described by @ref ESMTransform.
/// The template parameter defines the number of parameters used for the 2D transformation 
/// and should lie within 1 to 8. The following lists the individual meaning for all 8 parameters:
/// @li 0 x translation
/// @li 1 y translation
/// @li 2 in-plane rotation
/// @li 3 uniform scale
/// @li 4 non-uniform scale
/// @li 5 symmetric component (the last for a full affine model)
/// @li 6 and 7 missing perspective parameters
/// @ingroup gEsm
template<int PARAMS>
class Homography {
public:
    static const int dimensions = PARAMS;
    
    Homography() : H(TooN::Identity) {}
    
    template <int R, int C, typename P, typename B>
    Homography( const TooN::Matrix<R, C, P, B> & h) : H(h) {}
    
    const TooN::Matrix<3> & get_matrix() const { return H; }
    TooN::Matrix<3> & get_matrix() { return H; }
    const TooN::Vector<PARAMS> & get_jacobian( const TooN::Vector<2> & p, const TooN::Vector<2> & grad ) const {
        switch(PARAMS){
        case 8:
            // 0 0 p[1]
            J[7] = -p[1]*(grad*p); // TooN::makeVector(-p[1]*p[0], -p[1]*p[1]);
        case 7:
            // 0 0 p[0]
            J[6] = -p[0]*(grad*p); // TooN::makeVector(-p[0]*p[0], -p[0]*p[1]);
        case 6:
            // p[1] p[0] 0 
            J[5] = grad[0]*p[1] + grad[1]*p[0];// TooN::makeVector(p[1], p[0]);
        case 5:
            // p[0] -p[1] 0 
            J[4] = grad[0]*p[0] - grad[1]*p[1]; // TooN::makeVector(p[0], -p[1]);
        case 4:
            // p[0] p[1] -2 
            J[3] = 3 * (grad * p); //TooN::makeVector(3*p[0], 3*p[1]);
        case 3:
            // -p[1] p[0] 0 
            J[2] = - grad[0]*p[1] + grad[1]*p[0] ; // TooN::makeVector(-p[1], p[0]);
        case 2:
            J[1] = grad[1]; // TooN::makeVector(0, 1);
        case 1:
            J[0] = grad[0]; // TooN::makeVector(1, 0);
            break;
        default: 
            assert(false);
        }
        return J;
    }
    
    void update( const TooN::Vector<PARAMS> & v ){
        TooN::Matrix<3> G = TooN::Zeros;
        switch(PARAMS){
        case 8:
            G(2,1) = v[7];
        case 7:
            G(2,0) = v[6];
        case 6:
            G(0,1) = v[5];
            G(1,0) = v[5];
        case 5:
            G(0,0) = v[4];
            G(1,1) = -v[4];
        case 4:
            G(0,0) += v[3];
            G(1,1) += v[3];
            G(2,2) += -2*v[3];
        case 3:
            G(0,1) -= v[2];
            G(1,0) += v[2];
        case 2:
            G(1,2) = v[1];
        case 1:
            G(0,2) = v[0];
            break;
        default: assert(false);
        }
        H = H * TooN::exp(-G);
    }

protected:
    TooN::Matrix<3> H;
    mutable TooN::Vector<PARAMS> J;
};

/// This class provides a generic 2D homography transformation, but also applies
/// a fixed transformation on the input pixel locations. This allows to change
/// the reference point for the optimized mapping, for example to estimate it 
/// around the center of the template image. The class implements the interface described by @ref ESMTransform.
/// The parameterization is the same as for @ref Homography.
/// @ingroup gEsm
template<int PARAMS>
class HomographyPrefix : public Homography<PARAMS> {
public:
    static const int dimensions = PARAMS;
    
    HomographyPrefix() : Pre(TooN::Identity) {}

    template <int R, int C, typename P, typename B>
    HomographyPrefix( const TooN::Matrix<R, C, P, B> & p ) {
        set_prefix(p);
    }
    
    template <int R, int C, typename P, typename B, int R2, int C2, typename P2, typename B2>
    HomographyPrefix( const TooN::Matrix<R, C, P, B> & h, const TooN::Matrix<R2, C2, P2, B2> & p) {
        set_prefix(p);
        H = h * H;
    }
    
    const TooN::Matrix<3> get_matrix() const { return H * Pre; }
    
    template <int R, int C, typename P, typename B>
    void set_prefix( const TooN::Matrix<R, C, P, B> & p) {
        Pre = p;
        const TooN::Matrix<3> id = TooN::Identity;
        H = TooN::gaussian_elimination(Pre, id);
        PreGrad = H.T().template slice<0,0,2,2>();  // this is an approximation, if Pre is a projective warp !
    }

    const TooN::Vector<PARAMS> & get_jacobian( const TooN::Vector<2> & x, const TooN::Vector<2> & grad ) const {
        if( PARAMS > 2){
            const TooN::Vector<2> p = TooN::project(Pre * TooN::unproject(x));
            return Homography<PARAMS>::get_jacobian(p, PreGrad * grad);
        }
        return Homography<PARAMS>::get_jacobian(x, grad);
    }

protected:
    using Homography<PARAMS>::H;
    TooN::Matrix<3> Pre;
    TooN::Matrix<2> PreGrad;
};

/// a special implementation for 2D homography-based transformations described
/// as a camera rotating around its centre. The class implements the interface described by @ref esm.
/// It requires a linear camera model besides the rotation to be estimated.
/// @ingroup gEsm
class CameraRotation {
public:
    static const int dimensions = 3;

    CameraRotation() {}
    
    template <typename P, typename B>
    CameraRotation( const TooN::Vector<4, P, B> & camera_params, const TooN::SO3<> & init = TooN::SO3<>() ) 
    : R(init) 
    {
        set_camera(camera_params);
    }
    
    template <typename P, typename B>
    void set_camera( const TooN::Vector<4, P, B> & camera_params ){
        k = camera_params;
        K = TooN::Identity;
        K(0,0) = camera_params[0];
        K(1,1) = camera_params[1];
        K(0,2) = camera_params[2];
        K(1,2) = camera_params[3];
        
        kinv = TooN::makeVector( 1/camera_params[0], 1/camera_params[1],  -camera_params[2]/camera_params[0], -camera_params[3]/camera_params[1]);
        Kinv = TooN::Identity;
        Kinv(0,0) = kinv[0];
        Kinv(1,1) = kinv[1];
        Kinv(0,2) = kinv[2];
        Kinv(1,2) = kinv[3];
    }
    
    const TooN::Matrix<3> get_matrix() const { return K*R*Kinv; }
    
    const TooN::Vector<dimensions> & get_jacobian( const TooN::Vector<2> & p, const TooN::Vector<2> & grad ) const {
        // this implements grad * proj * K * G_i * Kinv * p;
        const TooN::Vector<3> pW = TooN::makeVector(kinv[0] * p[0] + kinv[2], kinv[1] * p[1] + kinv[3], 1);
        const TooN::Vector<3> J_pK = TooN::makeVector(grad[0] * k[0], grad[1] * k[1], grad * (k.slice<2,2>() - p));
        J[0] = J_pK * TooN::SO3<>::generator_field(0, pW);
        J[1] = J_pK * TooN::SO3<>::generator_field(1, pW);
        J[2] = J_pK * TooN::SO3<>::generator_field(2, pW);
        return J;
    }
    
    void update( const TooN::Vector<dimensions> & v ){
        R = R * TooN::SO3<>::exp(-v);
    }

    const TooN::SO3<> & get_rotation() const { return R; }
    TooN::SO3<> & get_rotation() { return R; }

protected:
    TooN::Matrix<3> K, Kinv; ///< linear camera matrix and its inverse
    TooN::Vector<4> k, kinv; ///< same parameters in vector form for convenience
    TooN::SO3<> R;           ///< the transformation, a rotation of the camera around its centre
    mutable TooN::Vector<dimensions> J; ///< internal temporary to avoid multiple re-initialisations
};

/// Basic appearance model implementing no change in the image.
/// @ingroup gEsm
class StaticAppearance {
public:
    static const int dimensions = 0;
    template <typename PIXEL> double difference( const PIXEL & warped, const PIXEL & templatePixel ) const { 
        return warped - templatePixel;
    }
    template <typename PIXEL> std::pair<double, TooN::Vector<dimensions> > difference_jacobian( const PIXEL & warped, const PIXEL & templatePixel ) const {
        return std::make_pair( difference(warped, templatePixel), TooN::Vector<dimensions>());
    }
    TooN::Vector<2> image_jacobian( const TooN::Vector<2> & gradWarped, const TooN::Vector<2> & gradTemplate ) const { 
        return (gradWarped + gradTemplate) * 0.25;
    }
    void update( const TooN::Vector<dimensions> & d ) {}
};

/// Simple appearance model that assumes a constant offset in the intensities of the image vs. the template.
/// The parameter @ref offset is estimated during the ESM optimization.
/// @ingroup gEsm
class OffsetAppearance {
public:
    static const int dimensions = 1;
    template <typename PIXEL> double difference( const PIXEL & warped, const PIXEL & templatePixel ) const { 
        return warped - templatePixel - offset; 
    }
    template <typename PIXEL> std::pair<double, TooN::Vector<dimensions> > difference_jacobian( const PIXEL & warped, const PIXEL & templatePixel ) const {
        return std::make_pair( difference(warped, templatePixel), TooN::makeVector(1));
    }
    TooN::Vector<2> image_jacobian( const TooN::Vector<2> & gradWarped, const TooN::Vector<2> & gradTemplate ) const {
        return (gradWarped + gradTemplate) * 0.25; 
    }
    void update( const TooN::Vector<dimensions> & d ) { 
        offset += d[0]; 
    }
    
    OffsetAppearance() : offset(0) {}
    double offset;
};

/// Blur appearance model that assumes that the input image was subject to blur. 
/// There is no parameter to estimate, but the image jacobians look different.
/// @todo reference paper from Lepetit here
/// @ingroup gEsm
class BlurAppearance {
public:
    static const int dimensions = 0;
    template <typename PIXEL> double difference( const PIXEL & warped, const PIXEL & templatePixel ) const { 
        return warped - templatePixel; 
    }
    template <typename PIXEL> std::pair<double, TooN::Vector<dimensions> > difference_jacobian( const PIXEL & warped, const PIXEL & templatePixel ) const {
        return std::make_pair( difference(warped, templatePixel), TooN::Vector<dimensions>());
    }
    TooN::Vector<2> image_jacobian( const TooN::Vector<2> & gradWarped, const TooN::Vector<2> & gradTemplate ) const { 
        return (gradWarped + 0.5*gradTemplate) * 0.25; 
    }
    void update( const TooN::Vector<dimensions> & d ) { 
    }
};

#if 0

class LinearAppearance {
    static const int dimensions = 2;

    template <typename PIXEL> double difference( const PIXEL & templatePixel, const PIXEL & warped ) const { return warped - templatePixel*scale - offset; }
    Vector<2> image_jacobian( const Vector<2> & gradTemplate, const Vector<2> & gradImage ) const { return gradTemplate + gradImage; }
    Vector<dimensions> get_jacobian( const Vector<2> & point ) { return TooN::makeVector(1.0); }
    void update( const Vector<dimensions> & d ) { offset += d[0]; }
    
    double offset;
    double scale;
};
#endif

template <typename P>
inline TooN::Matrix<3,3,P> scaleHomography( const TooN::Matrix<3,3,P> & H, const P & f ){
    const TooN::Vector<3,P> s = TooN::makeVector<P>(1/f, 1/f, 1);
    const TooN::Vector<3,P> is = TooN::makeVector<P>(f, f, 1);
    return is.as_diagonal() * H * s.as_diagonal();
}

#if 0
    
/// Helper function to compute the 2D homography between to images.
/// @ingroup gEsm
template<int PARAMS, typename APPEARANCE, typename IMAGE>
inline TooN::Matrix<3> inter_frame_homography( const BasicImage<IMAGE> & from, const BasicImage<IMAGE> & to, const TooN::Matrix<3> & init = TooN::Identity, ESMResult * const res = NULL){    
    TooN::Matrix<3> prefix = TooN::Identity;
#if 1
    prefix(0,2) = -from.size().x/2;
    prefix(1,2) = -from.size().y/2;
#else
    prefix(0,0) = 2.0/from.size().x;
    prefix(1,1) = 2.0/from.size().y;
    prefix(0,2) = -1;
    prefix(1,2) = -1;
    cout << prefix << endl;
#endif
    
    HomographyPrefix<PARAMS> H(init, prefix);
    APPEARANCE appearance;
    ESMTransform<HomographyPrefix<PARAMS>, APPEARANCE> T(H, appearance);
        
    ESMResult r = esm_opt(T, from, to, 40, 1e-2);
    
    if(res != NULL)
        *res = r;
    return T.transform.get_matrix();
}

#endif

/// The main class for the ESM module. This class stores the template image, the transformations and other information required to run the optimization. It is 
/// parameterized with two types that describe which geometric and radiometric transformations to use.
/// @ingroup gEsm
template <typename TRANSFORM, typename APPEARANCE, typename IMAGE = CVD::byte, typename GRADIENT = TooN::Vector<2, typename Pixel::traits<IMAGE>::wider_type> >
class ESMEstimator {
public:
    TRANSFORM transform;
    APPEARANCE appearance;
        
    Image<IMAGE> templ;
    Image<GRADIENT> templGradient;
    
    int max_iterations;
    double min_delta;
    double max_RMSE;
    
    ESMResult result;
    
    ESMEstimator() : max_iterations(40), min_delta(1e-5), max_RMSE(1e-2) {}
    ESMEstimator( const Image<IMAGE> & t, const Image<GRADIENT> & g ) : templ(t), templGradient(g), max_iterations(40), min_delta(1e-5), max_RMSE(1e-2) {}
    ESMEstimator( const Image<IMAGE> & t) : templ(t), max_iterations(40), min_delta(1e-5), max_RMSE(1e-2) {
        templGradient = Internal::gradient<IMAGE, GRADIENT>(templ);
    }
    ESMEstimator( const BasicImage<IMAGE> & t) : max_iterations(40), min_delta(1e-5), max_RMSE(1e-2) {
        templ.copy_from(t);
        templGradient = Internal::gradient<IMAGE, GRADIENT>(templ);
    }
    
    void set_image( const Image<IMAGE> & t, const Image<GRADIENT> & g ){
        assert(t.size() == g.size());
        templ = t;
        templGradient = g;
    }

    void set_image( const Image<IMAGE> & t ){
        templ = t;
        templGradient = Internal::gradient<GRADIENT>(templ);
    }

    void set_image( const BasicImage<IMAGE> & t ){
        Image<IMAGE> temp;
        temp.copy_from(t);
        set_image(temp);
    }
    
    void reset() { 
        transform = TRANSFORM();
        appearance = APPEARANCE();
    }

    const ESMResult & optimize( const BasicImage<IMAGE> & to ){
        return result = Internal::esm_opt( transform, appearance, templ, templGradient, to, max_iterations, min_delta, max_RMSE);
    }

    const ESMResult & optimize( const BasicImage<IMAGE> & from, const BasicImage<IMAGE> & to ){
        Image<GRADIENT> fromGradient = Internal::gradient<GRADIENT>(from);
        return optimize( from, fromGradient, to );
    }

    const ESMResult & optimize( const BasicImage<IMAGE> & from, const BasicImage<GRADIENT> & fromGradient, const BasicImage<IMAGE> & to ){
        return result = Internal::esm_opt( transform, appearance, from, fromGradient, to, max_iterations, min_delta, max_RMSE);
    }

    const ESMResult & get_result() const { return result; }
};

/// a specialization of @ref ESMEstimator for pure camera rotation only.
/// @ingroup gEsm
template <typename APPEARANCE, typename IMAGE = CVD::byte, typename GRADIENT = TooN::Vector<2, typename Pixel::traits<IMAGE>::wider_type> >
class RotationEstimator : public ESMEstimator<CameraRotation, APPEARANCE, IMAGE, GRADIENT> {
public:
    using ESMEstimator<CameraRotation, APPEARANCE, IMAGE, GRADIENT>::transform;
    using ESMEstimator<CameraRotation, APPEARANCE, IMAGE, GRADIENT>::appearance;

    RotationEstimator( const TooN::Vector<4> & cam_params, const TooN::SO3<> & init = TooN::SO3<>() ) {
        transform.set_camera( cam_params );
        transform.get_rotation() = init;
    }
    
    void reset() { 
        transform.get_rotation() = TooN::SO3<>(); 
        appearance = APPEARANCE();
    }

};

    namespace Internal {
        
        // intersects the line AB with the rectangle rect, returns the interval for the parameter t
        // for X = A + (B-A)*t
        // A, B in homogeneous coordinates, yielding the correct perspective interpolation for t
        inline TooN::Vector<2> getBounds(const TooN::Vector<2> & rect, const TooN::Vector<3> & A, const TooN::Vector<3> & B){
            const TooN::Vector<3> dir = A - B;
            TooN::Vector<2> interval = TooN::makeVector(0,1);
            
            TooN::Matrix<4,3> lines;
            lines[0] = TooN::makeVector(1,0,0);
            lines[1] = TooN::makeVector(-1,0,rect[0]);
            lines[2] = TooN::makeVector(0,1,0);
            lines[3] = TooN::makeVector(0,-1,rect[1]);
            
            const TooN::Vector<4> hitA = lines * A;
            const TooN::Vector<4> hitB = lines * B;
            
            for(int i = 0; i < 4; ++i){
                if(hitA[i] <=0 && hitB[i] <= 0)
                    return TooN::makeVector(0,0);
                if(hitA[i] * hitB[i] < 0){
                    const double t = hitA[i] / (lines[i] * dir);
                    if(hitA[i] < 0){
                        interval[0] = std::max(t, interval[0]);
                    } else {
                        interval[1] = std::min(t, interval[1]);
                    }
                }
            }
            
            if(interval[0] > interval[1])
                return TooN::makeVector(0,0);
            return interval;
        }
        
        inline std::vector<TooN::Vector<2,int> > getBounds( const TooN::Vector<2> & rect, const ImageRef & in, const TooN::Matrix<3> & H ){
            std::vector<TooN::Vector<2,int> > bounds(in.y);
            for(int y = 0; y < in.y; ++y){
                // map the current scan line
                const TooN::Vector<3> A = H * TooN::makeVector(0,y,1);
                const TooN::Vector<3> B = H * TooN::makeVector(in.x-1, y, 1);
                // determine valid sample interval
                const TooN::Vector<2> b = getBounds(rect, A, B) * (in.x-1);
                bounds[y] = TooN::makeVector<int>(ceil(b[0]), floor(b[1])+1);
            }
            return bounds;
        }
        
        inline std::vector<TooN::Vector<2,int> > erode( const std::vector<TooN::Vector<2,int> > & in ){
            using std::min; using std::max;
            std::vector<TooN::Vector<2,int> > out(in.size());
            out.front() = out.back() = TooN::makeVector(0, 0);
            for(unsigned i = 1; i < in.size()-1; ++i){
                out[i][0] = max(in[i-1][0], max(in[i][0] + 1, in[i+1][0]));  // the right most of all three 
                out[i][1] = min(in[i-1][1], min(in[i][1] - 1, in[i+1][1]));  // the left most of all three
                if(out[i][0] > out[i][1])                               // if its empty, then mark as (0,0)
                    out[i][0] = out[i][1] = 0;
            }
            return out;
        }
        
        // H takes pixels in out to pixels in in as a 2D homography (3x3 matrix)
        template <typename T>
        inline std::vector<TooN::Vector<2,int> > transform_perspective( BasicImage<T> & out, const BasicImage<T> & in, const TooN::Matrix<3> & H){
            const ImageRef & size = out.size();
            const TooN::Vector<2> insize = vec(in.size() - ImageRef(1,1));
            const TooN::Vector<3> across = H.T()[0];
            const TooN::Vector<3> down = H.T()[1];
            TooN::Vector<3> base = H.T()[2];
            
            const std::vector<TooN::Vector<2, int> > bounds = getBounds(insize, size, H);
            
            for(int y = 0; y < size.y; ++y){
                TooN::Vector<3> X = base + bounds[y][0] * across;
                T * data = out[y] + bounds[y][0];
                for(int x = bounds[y][0]; x < bounds[y][1]; ++x, X += across, ++data){
                    const TooN::DefaultPrecision inv = 1/X[2];
                    sample(in, X[0] * inv, X[1] * inv, *data );
                }
                base += down;                                                           // next line
            }
            
            return bounds;
        }
        
        // H takes pixels in out to pixels in in as a 2D affine transformation (3x3 matrix)
        template <typename T>
        inline std::vector<TooN::Vector<2,int> > transform_affine( BasicImage<T> & out, const BasicImage<T> & in, const TooN::Matrix<3> & H){
            const ImageRef & size = out.size();
            const TooN::Vector<2> insize = vec(in.size() - ImageRef(1,1));
            const TooN::Vector<2> across = H.T()[0].slice<0,2>();
            const TooN::Vector<2> down = H.T()[1].slice<0,2>();
            TooN::Vector<2> base = H.T()[2].slice<0,2>();
            
            const std::vector<TooN::Vector<2, int> > bounds = getBounds(insize, size, H);

            for(int y = 0; y < size.y; ++y){
                TooN::Vector<2> X = base + bounds[y][0] * across;
                T * data = out[y] + bounds[y][0];
                for(int x = bounds[y][0]; x < bounds[y][1]; ++x, X += across, ++data){
                    sample(in, X[0], X[1], *data );
                }
                base += down;                                                           // next line
            }

            return bounds;
        }
        
        // H takes pixels in out to pixels in in as a 2D translation only (stored in the 3x3 matrix)
        // This is extra optimized to use the constant mixing factors in the bi-linear interpolation
        template <typename T>
        inline std::vector<TooN::Vector<2,int> > transform_translation( BasicImage<T> & out, const BasicImage<T> & in, const TooN::Matrix<3> & H){
            const ImageRef & size = out.size();
            const TooN::Vector<2> insize = vec(in.size() - ImageRef(1,1));
            const TooN::Vector<2> across = H.T()[0].slice<0,2>();
            const TooN::Vector<2> down = H.T()[1].slice<0,2>();
            TooN::Vector<2> base = H.T()[2].slice<0,2>();
            
            std::vector<TooN::Vector<2, int> > bounds = getBounds(insize, size, H);
            
            const double fx = H(0,2) - floor(H(0,2));
            const double fy = H(1,2) - floor(H(1,2));
            const double ul = fx * fy;
            const double ur = (1-fx) * fy;
            const double ll = fx * (1-fy);
            const double lr = (1-fx)*(1-fy);
            
            for(int y = 0; y < size.y; ++y){
                const TooN::Vector<2> & interval = bounds[y];                           // determine valid sample interval
                int x = interval[0];
                T * data = out[y] + x;                                                  
                TooN::Vector<2> X = base + x * across;                                  // then sample
                for(; x < interval[1]; ++x, ++data, X += across){
                    sample(in, X[0], X[1], *data );
                }
                base += down;                                                           // next line
            }
            
            return bounds;
        }
        
        template <typename T>
        inline std::vector<TooN::Vector<2,int> > transform( BasicImage<T> & out, const BasicImage<T> & in, const TooN::Matrix<3> & H){
            const double perspective = H(2,0)*H(2,0) + H(2,1)*H(2,1);
            if(perspective < 1e-10)
                return transform_affine(out, in, H);
            return transform_perspective(out, in, H);
        }
        
        template<typename GRADIENT, typename IMAGE>
        inline Image<GRADIENT> gradient( const BasicImage<IMAGE> & img, const std::vector<TooN::Vector<2,int> > & bounds ){
            assert(bounds.size() == unsigned(img.size().y));
            Image<GRADIENT> grad(img.size());
            for(int y = 1; y < img.size().y-1; ++y){
                for(int x = bounds[y][0]; x < bounds[y][1]; ++x){
                    grad[y][x][0] = img[y][x+1] - img[y][x-1];
                    grad[y][x][1] = img[y+1][x] - img[y-1][x];
                }
            }
            return grad;
        }
        
        template<typename GRADIENT, typename IMAGE>
        inline Image<GRADIENT> gradient( const BasicImage<IMAGE> & img ){
            Image<GRADIENT> grad(img.size());
            for(int y = 1; y < img.size().y-1; ++y){
                for(int x = 1; x < img.size().x-1; ++x){
                    grad[y][x][0] = img[y][x+1] - img[y][x-1];
                    grad[y][x][1] = img[y+1][x] - img[y-1][x];
                }
            }
            return grad;
        }
        
        template <typename TRANSFORM, typename APPEARANCE, typename IMAGE, typename GRADIENT>
        inline ESMResult esm_opt( TRANSFORM & T, APPEARANCE & A, const BasicImage<IMAGE> & templateImage, const BasicImage<GRADIENT> & templateGradient, const BasicImage<IMAGE> & target, const int max_iterations, const double min_delta, const double max_RMSE ){
            assert(templateImage.size() == templateGradient.size());
            
            const int dimensions = TRANSFORM::dimensions+APPEARANCE::dimensions;
            
            Image<IMAGE> warped(templateImage.size());
            
            TooN::WLS<dimensions> wls;
        
            ESMResult result = {1e100, 1e100, 0, 0};
        
            // first warp here to be able to compare error before and after
            std::vector<TooN::Vector<2,int> > mask = Internal::transform(warped, target, T.get_matrix());
            mask = Internal::erode(mask);
        
            do {
                // get the gradient for the warped image
                Image<GRADIENT> grad_warped = Internal::gradient<GRADIENT>(warped, mask);
        
                // create the least squares system
                wls.clear();
                wls.add_prior(1e-10);
                TooN::Vector<dimensions> J;
                for(unsigned y = 0; y < mask.size(); ++y){
                    for(int x = mask[y][0]; x < mask[y][1]; ++x){
                        std::pair<double, TooN::Vector<APPEARANCE::dimensions> > da = A.difference_jacobian( warped[y][x], templateImage[y][x]);
                        J.template slice<0,TRANSFORM::dimensions>() = T.get_jacobian(TooN::makeVector(x,y), A.image_jacobian(grad_warped[y][x], templateGradient[y][x]));
                        J.template slice<TRANSFORM::dimensions, APPEARANCE::dimensions>() = da.second;
                        wls.add_mJ(da.first, J);
                    }
                }
        
                // solve and update
                wls.compute();
                T.update(wls.get_mu().template slice<0,TRANSFORM::dimensions>());
                A.update(wls.get_mu().template slice<TRANSFORM::dimensions, APPEARANCE::dimensions>());
                ++result.iterations;
        
                // compute new warp to calculate new error
                mask = Internal::transform(warped, target, T.get_matrix());
                mask = Internal::erode(mask);
        
                // compute error after update
                result.error = 0;
                // and number of pixels for RMS computation
                result.pixels = 0;
                for(unsigned y = 0; y < mask.size(); ++y){
                    result.pixels += mask[y][1] - mask[y][0];
                    for(int x = mask[y][0]; x < mask[y][1]; ++x){
                        const double d = A.difference(warped[y][x], templateImage[y][x]);
                        result.error += d*d;
                    }
                }
        
                // store results
                result.delta = norm(wls.get_mu());

#if 0                
                if(DEBUG_ESM){
                    std::ostringstream sout;
                    sout << "debug_" << std::setw(4) << std::setfill('0') << DEBUG_ESM << "_" << std::setw(4) << std::setfill('0') << result.iterations << ".jpg";
                    Image<byte> warped(templateImage.size());
                    warped.fill(0);
                    Internal::transform(warped, target, T.get_matrix());
                    img_save(warped, sout.str());
                    std::cout << wls.get_mu() << "\t" << T << std::endl;
                    std::cout << wls.get_C_inv() << std::endl;
                    std::cout << wls.get_vector() << std::endl;
                    std::cout << result << std::endl;
                }
#endif

            } while(result.iterations < max_iterations && result.delta > min_delta && result.RMSE() > max_RMSE);
        
            return result;
        }
                
    } // namespace Internal

template <int PARAMS>
inline std::ostream & operator<<(std::ostream & out, const Homography<PARAMS> & t ){
    out << t.get_matrix()[0] << " " << t.get_matrix()[1] << " " << t.get_matrix()[2];
    return out;
}

template <int PARAMS>
inline std::ostream & operator<<(std::ostream & out, const HomographyPrefix<PARAMS> & t ){
    out << t.get_matrix()[0] << " " << t.get_matrix()[1] << " " << t.get_matrix()[2];
    return out;
}

inline std::ostream & operator<<(std::ostream & out, const StaticAppearance & t ){
    return out;
}

inline std::ostream & operator<<(std::ostream & out, const OffsetAppearance & t ){
    out << t.offset;
    return out;
}

inline std::ostream & operator<<(std::ostream & out, const BlurAppearance & t ){
    return out;
}

} // namespace CVD

#endif // CVD_ESM_H

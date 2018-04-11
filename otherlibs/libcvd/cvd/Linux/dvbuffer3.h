// -*- c++ -*-

#ifndef __DVBUFFER_3_H
#define __DVBUFFER_3_H
#include <cvd/videobuffer.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <inttypes.h>
#include <cvd/colourspaces.h>

namespace CVD
{
  namespace Exceptions
  {
    namespace DVBuffer3
    {
      /// Class for all DVBuffer3 exceptions
      /// @ingroup gException
      struct All : public CVD::Exceptions::VideoBuffer::All 
      {
	All(std::string sWhat)
	  {
	    what = "DVBuffer3: " + sWhat;
	  }
      };
    }
  }

  /// Internal DVBuffer3 helpers
  namespace DV3
  {
    /// This enumerates the list of controllable features
    /// (This isn't the full set of DC1394 ones, just a few.)
    enum DV3Feature { BRIGHTNESS,  EXPOSURE,  SHARPNESS,
		      WHITE_BALANCE,  HUE,  SATURATION,
		      GAMMA,  SHUTTER,  GAIN,  IRIS,
		      FOCUS, ZOOM,   PAN,  TILT, FRAME_RATE};
    
    /// This enumerates all the colourspace types supported by DC1394
    /// N.b. only a small fraction of these map 1:1 to libCVD types
    enum DV3ColourSpace { MONO8,  MONO16,  MONO16S,
			  RGB8,	  RGB16,   RGB16S,
			  YUV411, YUV422,  YUV444,
			  RAW8,	  RAW16};
    
    /// This enumerates the different colour filter tile patterns for Bayer
    /// images. This can be queried from the RawDVBuffer3 for a Bayer mode.
    /// This is only supported for DC1394 v2
    enum DV3ColourFilter { UNDEFINED = -1, RGGB = 0, GBRG = 1, GRBG = 2, BGGR = 3 };
    
#ifndef DOXYGEN_IGNORE_INTERNAL
    // Translation helper classes to go from CVD-types to the above
    template<class C>  
      struct CSConvert
      {	static const DV3ColourSpace space = C::Error__type_not_valid_for_camera; };
    template<> struct CSConvert<byte>
      {	static const DV3ColourSpace space = MONO8;};
    template<> struct CSConvert<short unsigned int>
      {	static const DV3ColourSpace space = MONO16;};
    template<> struct CSConvert<yuv411>
      {	static const DV3ColourSpace space = YUV411;};
    template<> struct CSConvert<yuv422>
      {	static const DV3ColourSpace space = YUV422;};
    template<> struct CSConvert<Rgb<byte> > 
      {	static const DV3ColourSpace space = RGB8;};
    template<> struct CSConvert<bayer_bggr>
      { static const DV3ColourSpace space = RAW8; }; 
    template<> struct CSConvert<bayer_gbrg>
      { static const DV3ColourSpace space = RAW8; }; 
    template<> struct CSConvert<bayer_grbg>
      { static const DV3ColourSpace space = RAW8; }; 
    template<> struct CSConvert<bayer_rggb>
      { static const DV3ColourSpace space = RAW8; }; 
    template<> struct CSConvert<bayer_bggr16be>
      { static const DV3ColourSpace space = RAW16; }; 
    template<> struct CSConvert<bayer_gbrg16be>
      { static const DV3ColourSpace space = RAW16; }; 
    template<> struct CSConvert<bayer_grbg16be>
      { static const DV3ColourSpace space = RAW16; }; 
    template<> struct CSConvert<bayer_rggb16be>
      { static const DV3ColourSpace space = RAW16; }; 

    template<class C> struct CSFilter { static const DV3ColourFilter filter = UNDEFINED; };
    template<> struct CSFilter<bayer_bggr> { static const DV3ColourFilter filter = BGGR; };
    template<> struct CSFilter<bayer_gbrg> { static const DV3ColourFilter filter = GBRG; };
    template<> struct CSFilter<bayer_grbg> { static const DV3ColourFilter filter = GRBG; };
    template<> struct CSFilter<bayer_rggb> { static const DV3ColourFilter filter = RGGB; };
    template<> struct CSFilter<bayer_bggr16be> { static const DV3ColourFilter filter = BGGR; };
    template<> struct CSFilter<bayer_gbrg16be> { static const DV3ColourFilter filter = GBRG; };
    template<> struct CSFilter<bayer_grbg16be> { static const DV3ColourFilter filter = GRBG; };
    template<> struct CSFilter<bayer_rggb16be> { static const DV3ColourFilter filter = RGGB; };

    struct LibDCParams;
#endif
    
    /// Non-templated libDC1394 interface. This is used by DVBuffer3. If you want 
    /// typed video frames, you should use DVBuffer 3 instead..
    /// The implementation of this class depends on which version of libDC1394 is 
    /// installed on the system. Format 7 support is only present for libDC1394 V2.
    class RawDVBuffer3: public virtual RawVideoBuffer
    {
    public:
      /// Mode-selecting constructor for all standard modes & Format 7. First it tries
      /// to find a standard mode, then it looks at the Format 7 modes. If an offset
      /// is given, standard modes are skipped (they don't allow offsets).
      /// @param colourspace Enumerated colourspace requested
      /// @param nCamNumber Which camera on the bus to use
      /// @param irSize Requested video size; if left at (-1,-1) use biggest available
      /// @param fFrameRate Requested frame-rate; if negative, use fastest available
      /// @param irOffset offset of video frame in CCD; if left at (-1,-1) use default modes or center window
      RawDVBuffer3(DV3ColourSpace colourspace,
		   int nCamNumber=0, 
		   uint64_t cam_guid=-1,
		   int cam_unit=-1,
		   bool verbose=0,
		   bool bus_reset=0,
		   ImageRef irSize = ImageRef(-1,-1),
		   float fFrameRate=-1.0, 
		   ImageRef irOffset = ImageRef(-1,-1),
		   int format7_mode=-1);
      
      ~RawDVBuffer3();
      static void stopAllTransmissions(void);
      inline ImageRef size() {return mirSize;}
      inline ImageRef offset() {return mirOffset;}
      inline double frame_rate() {return mdFramerate;}
      inline DV3ColourFilter colour_filter() { return mColourfilter; }
      
      VideoFrame<byte>* get_frame();
      void put_frame(VideoFrame<byte>* f);
      bool frame_pending();

      void set_feature_value(DV3Feature nFeature, unsigned int nValue);
      unsigned int get_feature_value(DV3Feature nFeature);
      std::pair<unsigned int, unsigned int> get_feature_min_max(DV3Feature nFeature);
      void auto_on_off(DV3Feature nFeature, bool bValue);
      void power_on_off(DV3Feature nFeature, bool bValue);
      
    private:
	  
      ImageRef mirSize;
      ImageRef mirOffset;
      double mdFramerate;
      DV3ColourFilter mColourfilter;
      /// This encapsulates the actual libDC1394 variables
      LibDCParams *mpLDCP;
    };
    
  }

  /// A video buffer from a Firewire (IEEE 1394) camera. 
  /// This can use both v1.x and v2.0 series of libdc1394. For v2 it
  /// selects the video mode as described for RawDVBuffer3 above. For
  /// v1 this just wraps DVBuffer2.
  template <class pixel_T> 
    class DVBuffer3 : public VideoBuffer<pixel_T>, public DV3::RawDVBuffer3
    {
    public:
    DVBuffer3(unsigned int nCamNumber=0, 
	      ImageRef irSize = ImageRef(-1,-1), 
	      float fFPS = -1.0, 
	      ImageRef irOffset = ImageRef(-1,-1),
		  bool verbose=0,
		  bool bus_reset=0,
		  int format7_mode=-1)
      : VideoBuffer<pixel_T>(VideoBufferType::Live),
	    RawDVBuffer3(DV3::CSConvert<pixel_T>::space, nCamNumber, 0, -1, verbose, bus_reset, irSize, fFPS, irOffset, format7_mode)
	{
		if(DV3::CSFilter<pixel_T>::filter != DV3::UNDEFINED && colour_filter() != DV3::CSFilter<pixel_T>::filter )
			throw(Exceptions::DVBuffer3::All("wrong colour filter expected"));
	}
      
      virtual ~DVBuffer3()   {}
      double frame_rate()    {return RawDVBuffer3::frame_rate();  }
      ImageRef size()        {return RawDVBuffer3::size(); }
      virtual VideoFrame<pixel_T>* get_frame()
      {
	return reinterpret_cast<VideoFrame<pixel_T>*>(RawDVBuffer3::get_frame());
      }
      virtual void put_frame(VideoFrame<pixel_T>* f)
      {
	RawDVBuffer3::put_frame(reinterpret_cast<VideoFrame<byte>*>(f));
      }
      virtual bool frame_pending() {return RawDVBuffer3::frame_pending();}
      virtual void seek_to(double){}
    };
  
}

#endif

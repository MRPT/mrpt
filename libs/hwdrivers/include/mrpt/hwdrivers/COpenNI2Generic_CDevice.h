/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

/** @file Include this file only from your user code if you have OPENNI2 */

namespace mrpt {
  namespace hwdrivers {

class COpenNI2Generic::CDevice{
    public:
        typedef stlplus::smart_ptr<CDevice> Ptr;
        enum{
			COLOR_STREAM, DEPTH_STREAM, IR_STREAM,
			STREAM_TYPE_SIZE  // this last value is to know the number of possible channels, leave it always at the end!
        };
#if MRPT_HAS_OPENNI2
    private:
        class CStream{
            public:
                typedef stlplus::smart_ptr<CStream> Ptr;
            private:

                std::ostream&       m_log;
                openni::Device&     m_device;
                std::string         m_strName;
                openni::SensorType  m_type;
                openni::VideoStream m_stream;
                openni::PixelFormat m_format;
                bool                m_verbose;
            public:
                CStream(openni::Device& device, openni::SensorType type, openni::PixelFormat format, std::ostream& log, bool verbose);
                virtual ~CStream();
                const std::string& getName()const{ return m_strName; }
                bool               isValid()const;
                bool               isMirrorSupported()const;
                bool               setMirror(bool flag);
                void               setCloseRange(int& value);
                virtual bool       open(int w, int h, int fps);
                virtual bool       start();
                virtual void       destroy();
                virtual bool       getFrame(openni::VideoFrameRef& frame, uint64_t &timestamp, bool &there_is_obs, bool &hardware_error);

                int                getFrameWidth() const{ return m_stream.getVideoMode().getResolutionX(); }
                int                getFrameHeight()const{ return m_stream.getVideoMode().getResolutionY(); }
                double             getHFov()const{ return m_stream.getHorizontalFieldOfView(); }
                double             getFx()  const{ return getFrameWidth()  / (2.0 * tan(getHFov() / 2.0)); }
                double             getVFov()const{ return m_stream.getVerticalFieldOfView(); }
                double             getFy()  const{ return getFrameHeight() / (2.0 * tan(getVFov() / 2.0)); }
                double             getCx()  const{ return (getFrameWidth()  - 1) * 0.5; }
                double             getCy()  const{ return (getFrameHeight() - 1) * 0.5; }

				void disableAutoExposure() {m_stream.getCameraSettings()->setAutoExposureEnabled(false); }
				void enableAutoExposure() {m_stream.getCameraSettings()->setAutoExposureEnabled(true); }

                void getCameraParam(mrpt::utils::TCamera & param)const{
                    param.ncols = getFrameWidth();
                    param.nrows = getFrameHeight();
                    param.fx(getFx());
                    param.fy(getFy());
                    param.cx(getCx());
                    param.cy(getCy());
                }

                static  Ptr        create(openni::Device& device, openni::SensorType type, openni::PixelFormat format, std::ostream& log, bool verbose);
        };
        openni::DeviceInfo  m_info;
        openni::Device      m_device;
        CStream::Ptr        m_streams[STREAM_TYPE_SIZE];
        bool                m_mirror;
        std::stringstream   m_log;
		bool                m_verbose;

        bool synchMirrorMode();
        bool startStreams();

        inline void resize(mrpt::utils::CImage& rgb  , int w, int h){ rgb.resize(w, h, CH_RGB, true); }
        inline void resize(mrpt::math::CMatrix& depth, int w, int h){ depth.resize(h, w); }
        inline void resize(mrpt::obs::CObservation3DRangeScan& obs, int w, int h){
            resize(obs.intensityImage, w, h);
            obs.rangeImage_setSize(h, w);
        }

        inline void setPixel(const openni::RGB888Pixel& src, mrpt::utils::CImage& rgb  , int x, int y){ rgb.setPixel(x, y, (src.r << 16) + (src.g << 8) + src.b); }
        inline void setPixel(const openni::DepthPixel& src , mrpt::math::CMatrix& depth, int x, int y){
            static const double rate = 1.0 / 1000;
            depth(y, x) = src * rate;
        }

        template <class NI_PIXEL, class MRPT_DATA>
        void copyRow(const char* src, MRPT_DATA& rgb, int w, const int y){
            const NI_PIXEL* s = (const NI_PIXEL*)src;
            for (int xc = 0; xc < w; ++xc, ++s){
                int x = xc;
                if(isMirrorMode()){
                    x = w - xc - 1;
                }
                setPixel(*s, rgb, x, y);
            }
        }

        template <class NI_PIXEL, class MRPT_DATA>
        void copyFrame(openni::VideoFrameRef& frame, MRPT_DATA& dst){
            const char*  data    = (const char*)frame.getData();
            const int    stride  = frame.getStrideInBytes();
            const int    width   = frame.getWidth();
            const int    height  = frame.getHeight();
            resize(dst, width, height);
            for (int y = 0; y < height; ++y, data+=stride){
                copyRow<NI_PIXEL, MRPT_DATA>(data, dst, width, y);
            }
        }

    public:
		CDevice(const openni::DeviceInfo& info, openni::PixelFormat rgb, openni::PixelFormat depth, bool m_verbose);
        virtual ~CDevice();

        const openni::DeviceInfo& getInfo()const{ return m_info; }
        std::string getLog()const{ return m_log.str(); }

        void clearLog(){ m_log.str(""); m_log.clear(); }
        bool isMirrorMode()const{ return m_mirror; }
        void setMirrorMode(bool mode){ m_mirror = mode; }
		bool hasColor()const{ if (!m_streams[COLOR_STREAM]) return false; else return m_streams[COLOR_STREAM]->isValid(); }
		bool hasDepth()const{ if (!m_streams[DEPTH_STREAM]) return false; else return m_streams[DEPTH_STREAM]->isValid(); }

        bool isOpen()const;
        void close();
        bool open(int w, int h, int fps);

        bool getNextFrameRGB(mrpt::utils::CImage &img, uint64_t &timestamp, bool &there_is_obs, bool &hardware_error);
        bool getNextFrameD  (mrpt::math::CMatrix &img, uint64_t &timestamp, bool &there_is_obs, bool &hardware_error);
        bool getNextFrameRGBD(mrpt::obs::CObservation3DRangeScan &obs, bool &there_is_obs, bool &hardware_error);

        bool getCameraParam(int streamType, mrpt::utils::TCamera& param)const{
            if(streamType < 0 || streamType >= STREAM_TYPE_SIZE){
                return false;
            }
			if(!m_streams[streamType] || m_streams[streamType]->isValid() == false){ return false; }
            m_streams[streamType]->getCameraParam(param);
            return true;
        }

        bool getSerialNumber(unsigned int& sn);

		static Ptr create(const openni::DeviceInfo& info, openni::PixelFormat rgb, openni::PixelFormat depth, bool verbose);

		openni::Device & getDevicePtr(){ return m_device; }

    private:
        bool getSerialNumber(std::string& sn);

#endif // MRPT_HAS_OPENNI2
};

}
}

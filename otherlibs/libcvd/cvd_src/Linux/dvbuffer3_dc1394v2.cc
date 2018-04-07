// -*- c++ -*-

#include <cvd/Linux/dvbuffer3.h>
#include <cvd/byte.h>
#include <cvd/timer.h>

#include <dc1394/dc1394.h>
#ifndef __APPLE__
#include <libraw1394/raw1394.h>
#endif
#include <vector>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

using namespace CVD;
using namespace std;
using namespace DV3;
using CVD::Exceptions::DVBuffer3::All;

namespace CVD
{

  namespace DV3
  {
      
    class VPrint_
    {
	public:
	    VPrint_(bool _p)
	    :p(_p)
	    {}

	    template<class C> VPrint_& operator<<(const C& c)
	    {
		if(p)
		    cerr << c;
		
		return *this;
	    }

	    bool p;
    };

    class VPrint: public VPrint_
    {
	public:
	VPrint(bool _b)
	:VPrint_(_b){}

	template<class C> VPrint_& operator<<(const C& c)
	{
	    if(p)
		cerr << "RawDVBuffer3: " << c;
	    
	    return *this;
	}

    };

    string coding(dc1394color_coding_t ii)
    {
    	int i = (int)ii;
    	static vector<string> c;

	if(c.empty())
	{
	    c.push_back("MONO8");
	    c.push_back("YUV411");
	    c.push_back("YUV422");
	    c.push_back("YUV444");
	    c.push_back("RGB8");
	    c.push_back("MONO16");
	    c.push_back("RGB16");
	    c.push_back("MONO16S");
	    c.push_back("RGB16S");
	    c.push_back("RAW8");
	    c.push_back("RAW16");
         }

	 i-=352;
	 if(i < 0 || i >= (int)c.size())
	     return "error";
	 else
	     return c[i];
    }


    string filter(dc1394color_filter_t f)
    {
	switch(f)
	{
	    case DC1394_COLOR_FILTER_RGGB:
	        return "RGGB";
	    case DC1394_COLOR_FILTER_BGGR:
	        return "BGGR";
	    case DC1394_COLOR_FILTER_GRBG:
	        return "GRBG";
	    case DC1394_COLOR_FILTER_GBRG:
	        return "GBRG";
	}

	return "error";
    }

    static DV3ColourFilter DV3_from_DC_ColourFilter(dc1394color_filter_t f, uint32_t /*vendor*/, uint32_t /*model*/, uint64_t guid)
    {
      // some cameras report incorrect bayer patterns
      if (guid==0x814436200006075ULL) { return GBRG; }
      return static_cast<DV3ColourFilter>(f - DC1394_COLOR_FILTER_MIN);
    }

    struct LibDCParams
    {
      dc1394_t *pDC1394;
      dc1394camera_t *pCamera;
      LibDCParams()   {  pDC1394 = NULL; pCamera = NULL;  }
    };
    
    struct DV3Frame : public VideoFrame<byte>
    {
      DV3Frame(dc1394video_frame_t *pDC1394Frame)
	: VideoFrame<byte>(const_cast<const cvd_timer & >(timer).conv_ntime(pDC1394Frame->timestamp / 1000000.0),
			   pDC1394Frame->image,
			   ImageRef(pDC1394Frame->size[0], pDC1394Frame->size[1]))
      {	mpDC1394Frame = pDC1394Frame; }
    protected:
      dc1394video_frame_t *mpDC1394Frame;
      friend class RawDVBuffer3;
    };
    
    static dc1394color_coding_t DC_from_DV3_ColourSpace(DV3ColourSpace s, uint32_t /*vendor*/, uint32_t /*model*/, uint64_t guid)
    {
      // some cameras report their raw bayer mode as being mono and do not
      // have a mono mode at all...
      if (guid==0x814436200006075ULL) {
        //vendor==0x81443 model==0x0 ?
        switch(s)
	  {
	  case MONO8:  return DC1394_COLOR_CODING_RAW8; 
	  case MONO16: return DC1394_COLOR_CODING_RAW16; 
	  case RAW8:   return DC1394_COLOR_CODING_MONO8; 
	  case RAW16:  return DC1394_COLOR_CODING_MONO16; 
	  default: break;
	  }
      }
      switch(s)
	{
	case MONO8:  return DC1394_COLOR_CODING_MONO8; 
	case MONO16: return DC1394_COLOR_CODING_MONO16; 
	case MONO16S:return DC1394_COLOR_CODING_MONO16S; 
	case RGB8:   return DC1394_COLOR_CODING_RGB8; 
	case RGB16:  return DC1394_COLOR_CODING_RGB16; 
	case RGB16S: return DC1394_COLOR_CODING_RGB16S; 
	case YUV411: return DC1394_COLOR_CODING_YUV411; 
	case YUV422: return DC1394_COLOR_CODING_YUV422; 
	case YUV444: return DC1394_COLOR_CODING_YUV444; 
	case RAW8:   return DC1394_COLOR_CODING_RAW8; 
	case RAW16:  return DC1394_COLOR_CODING_RAW16; 
	}
      throw(All("DC_from_DV3: Invalid colourspace"));
    }

    static dc1394feature_t DC_from_DV3_Feature(DV3Feature f)
    {
      switch(f)
	{
	case BRIGHTNESS:    return DC1394_FEATURE_BRIGHTNESS;
	case EXPOSURE  :    return DC1394_FEATURE_EXPOSURE;
	case SHARPNESS:     return DC1394_FEATURE_SHARPNESS;
	case WHITE_BALANCE: return DC1394_FEATURE_WHITE_BALANCE;
	case HUE:           return DC1394_FEATURE_HUE;
	case SATURATION:    return DC1394_FEATURE_SATURATION;
	case GAMMA:         return DC1394_FEATURE_GAMMA;
	case SHUTTER:       return DC1394_FEATURE_SHUTTER;
	case GAIN:          return DC1394_FEATURE_GAIN;
	case IRIS:          return DC1394_FEATURE_IRIS;
	case FOCUS:         return DC1394_FEATURE_FOCUS;
	case ZOOM:          return DC1394_FEATURE_ZOOM;
	case PAN:           return DC1394_FEATURE_PAN;
	case TILT:          return DC1394_FEATURE_TILT;
	case FRAME_RATE:    return DC1394_FEATURE_FRAME_RATE;
	}
      throw(All("DC_from_DV3: Invalid feature"));
    }
    
    RawDVBuffer3::RawDVBuffer3(DV3ColourSpace colourspace,
			       int nCamNumber,
			       uint64_t cam_guid,
			       int cam_unit,
			       bool verbose,
			       bool bus_reset,
			       ImageRef irSize,
			       float fFrameRate, 
			       ImageRef irOffset,
				   int format7_mode)
    {
      VPrint log(verbose);

      mpLDCP = new LibDCParams;
  
      // Create a libDC1394 context.
      mpLDCP->pDC1394 = dc1394_new();
  
      // A variable to record error values..
      dc1394error_t error;

      // Enumerate the cameras connected to the system.
      dc1394camera_list_t *pCameraList = NULL;
  
      error = dc1394_camera_enumerate(mpLDCP->pDC1394, &pCameraList);
      if(error) throw(All("Camera enumerate"));

      log << "Requesting: \n";
      log << "    colourspace:   " <<  colourspace  << "\n";
      log << "    camera number: " <<  nCamNumber  << "\n";
      log << "    camera guid:   " <<  cam_guid  << "\n";
      log << "    camera unit:   " <<  cam_unit  << "\n";
      log << "    size:          " <<  irSize  << "\n";
      log << "    framerate:     " <<  fFrameRate  << "\n";
      log << "    offset:        " <<  irOffset  << "\n";
      log << "    format7 mode:  " <<  format7_mode  << "\n";
  
      if(pCameraList->num == 0)
	{
	  dc1394_camera_free_list(pCameraList);
	  throw(All("No cameras found."));
	}
      else
        {
	    log << "List of cameras:\n";
	    for(unsigned int i=0; i < pCameraList->num; i++)
	    {
	      log << "    Camera: " << i << ": unit=" << pCameraList->ids[i].unit << " guid=" << hex << pCameraList->ids[i].guid << dec << "\n";

	      if(nCamNumber == -1 && cam_guid == pCameraList->ids[i].guid)
	      {
	      	if(cam_unit == -1 || cam_unit == pCameraList->ids[i].unit)
		  nCamNumber = i;
	      }
	    }
	}
 	 
      if(nCamNumber + 1 > (int)pCameraList->num)
	{
	  dc1394_camera_free_list(pCameraList);
	  throw(All("Selected camera out of range"));
	}
  
      // Allocate a camera struct...
      mpLDCP->pCamera = dc1394_camera_new(mpLDCP->pDC1394, 
					  pCameraList->ids[nCamNumber].guid);
      dc1394_camera_free_list(pCameraList);
  
      if(!mpLDCP->pCamera) throw(All("Failed on dc1394_camera_new"));

      if (bus_reset) {
	dc1394switch_t is_iso_on;
	if (dc1394_video_get_transmission(mpLDCP->pCamera, &is_iso_on)!=DC1394_SUCCESS) is_iso_on = DC1394_OFF;
	if (is_iso_on==DC1394_ON) {
	  dc1394_video_set_transmission(mpLDCP->pCamera, DC1394_OFF);
	}
      }

      log << "Selected camera: " << hex << mpLDCP->pCamera->vendor_id << ":" << mpLDCP->pCamera->model_id << "(guid: " << mpLDCP->pCamera->guid << dec << ")\n";
 
      // What mode to use?
      dc1394color_coding_t nTargetColourCoding = DC_from_DV3_ColourSpace(colourspace, mpLDCP->pCamera->vendor_id, mpLDCP->pCamera->model_id, mpLDCP->pCamera->guid);

      dc1394_camera_reset(mpLDCP->pCamera);

      log << "Target colour coding: " << nTargetColourCoding << " (" << coding(nTargetColourCoding) << ")\n";
      bool foundAStandardMode = false;
	dc1394video_mode_t nMode;
	mColourfilter = UNDEFINED;
	if(irOffset.x == -1 && format7_mode == -1){
		try {
			// First, get a list of the modes which the camera supports.
			dc1394video_modes_t modes;
			error = dc1394_video_get_supported_modes(mpLDCP->pCamera, &modes);
			if(error) throw(All("Could not get modelist"));
	
			// Second: Build up a list of the modes which are the right colour-space
			std::vector<dc1394video_mode_t> vModes;
			for(unsigned int i = 0; i < modes.num; i++)
			{
			  dc1394video_mode_t nMode = modes.modes[i];
			  // ignore format 7 for now
			  if(nMode >= DC1394_VIDEO_MODE_FORMAT7_0) continue;
			  dc1394color_coding_t nColourCoding;
			  error=dc1394_get_color_coding_from_video_mode(mpLDCP->pCamera,nMode,&nColourCoding);
			  if(error) throw(All("Error in get_color_coding_from_video_mode"));
			  if(nColourCoding == nTargetColourCoding)
				vModes.push_back(nMode);
			}
			if(vModes.size() == 0) throw(-1);
			
			// Third: Select mode according to size
			bool bModeFound = false;
			dc1394video_mode_t nMode;
			if(irSize.x != -1)	   // Has the user specified a target size? Choose mode according to that..
				for(size_t i = 0; i<vModes.size(); i++){
					uint32_t x,y;
					dc1394_get_image_size_from_video_mode(mpLDCP->pCamera, vModes[i], &x, &y);
					if(x == (uint32_t) irSize.x && y == (uint32_t) irSize.y) {
						bModeFound = true;
						nMode = vModes[i];
						break;
					}
				}
			else  // If the user didn't specify a target size, choose the one with the 
			{ // highest resolution.
				sort(vModes.begin(), vModes.end());
				bModeFound = true;
				nMode = vModes.back();
			}
			if(!bModeFound) throw(-1);
			
			// Store the size of the selected mode..
			uint32_t x,y;
			dc1394_get_image_size_from_video_mode(mpLDCP->pCamera, nMode, &x, &y);
			mirSize.x = x;
			mirSize.y = y;
			mirOffset = irOffset;

			// Got mode, now decide on frame-rate. Similar thing: first get list, then choose from list.
			dc1394framerates_t framerates;
			dc1394framerate_t nChosenFramerate = DC1394_FRAMERATE_MIN;
			mdFramerate = -1.0;
			error = dc1394_video_get_supported_framerates(mpLDCP->pCamera,nMode,&framerates);
			if(error) throw(All("Could not query supported framerates"));
			
			if(fFrameRate > 0) // User wants a specific frame-rate?
			{
				for(unsigned int i=0; i<framerates.num; i++){
					float f_rate;
					dc1394_framerate_as_float(framerates.framerates[i],&f_rate); 
					if(f_rate == fFrameRate){
						nChosenFramerate = framerates.framerates[i];
						mdFramerate = f_rate;
						break;
					}
				}
			} else { // Just pick the highest frame-rate the camera can do.
				for(unsigned int i=0; i<framerates.num; i++){
					float f_rate;
					dc1394_framerate_as_float(framerates.framerates[i],&f_rate); 
					if(f_rate > mdFramerate) {
						nChosenFramerate = framerates.framerates[i];
						mdFramerate = f_rate;
					}
				}
			}
			if(mdFramerate == -1.0) throw(-1);
		
			// Selected mode and frame-rate; Now tell the camera to use these.
			// At the moment, hard-code the channel to speed 400. This is maybe something to fix in future?
			error = dc1394_video_set_iso_speed(mpLDCP->pCamera, DC1394_ISO_SPEED_400);
			if(error) throw(All("Could not set ISO speed."));
			
			error = dc1394_video_set_iso_channel(mpLDCP->pCamera, nCamNumber);
			if(error) throw(All("Could not set ISO channel.")); 
		
			error = dc1394_video_set_mode(mpLDCP->pCamera, nMode);
			if(error) throw(All("Could not set video mode"));
			  
			error = dc1394_video_set_framerate(mpLDCP->pCamera, nChosenFramerate);
			if(error) throw(All("Could not set frame-rate"));		 
			// no need to check Format 7 modes
			foundAStandardMode = true;
		}
		catch(int e){
			//cout << "DCBuffer is checking format 7 modes as well" << endl;
			foundAStandardMode = false;
		}
	}
	if(!foundAStandardMode){
	        log << "Failed to find a standard mode. Using FORMAT_7\n";

		dc1394format7modeset_t modeset;
		error = dc1394_format7_get_modeset(mpLDCP->pCamera, &modeset);
		if(error) throw(All("Could not get Format 7 modes."));
		int index = 0;
		if(irOffset.x == -1)
			mirOffset = ImageRef(0,0);
		else
			mirOffset = irOffset;
		for(; index < DC1394_VIDEO_MODE_FORMAT7_NUM ; ++index){
			const dc1394format7mode_t & mode = modeset.mode[index];

			log << "    FORMAT_7 mode index " << index << "\n";
			log << "        present:       " << mode.present << "\n";
			if(!mode.present) continue;
			log << "        size:          " << mode.size_x << "x" << mode.size_y << "\n";
			log << "        max size:      " << mode.max_size_x << "x" << mode.max_size_y << "\n";
			log << "        position?:     " << mode.pos_x << "x" << mode.pos_y << "\n";
			log << "        unit size:     " << mode.unit_size_x << "x" << mode.unit_size_y << "\n";
			log << "        unit pos:      " << mode.unit_pos_x << "x" << mode.unit_pos_y << "\n";
			log << "        color codings: " << mode.color_codings.num << "\n";

			for(unsigned int i=0; i <  mode.color_codings.num && i < DC1394_COLOR_CODING_NUM; i++)
			    log << "            color: " << mode.color_codings.codings[i] << "(" << coding(mode.color_codings.codings[i]) << ")\n";
			log << "        color: " << mode.color_coding <<  "(" << coding(mode.color_coding) << ")\n";

			log << "        pixnum:        " << mode.pixnum << "\n";
			log << "        packet size:   " << mode.packet_size << "\n";
			log << "        unit pkt size: " << mode.unit_packet_size << "\n";
			log << "        max pkt size:  " << mode.max_packet_size << "\n";
			log << "        total bytes:   " << mode.total_bytes << "\n";
			log << "        color filter:  " << mode.color_filter << "(" << filter(mode.color_filter) << ")\n";

			if(format7_mode != -1 && format7_mode != index)
				continue;

			// does the mode exist ?
			// does it support the colour format we need ?
			unsigned int i;
			for(i = 0; i < mode.color_codings.num && i < DC1394_COLOR_CODING_NUM ; ++i)
			{
				if(mode.color_codings.codings[i] == nTargetColourCoding)
					break;
			}

			if(i == mode.color_codings.num) 
			{
			    log << "      No matching mode\n";
			    continue;
			}
			if(irSize.x != -1){
				// can it support the size ?
				if((irSize.x + mirOffset.x) > (int)mode.max_size_x || (irSize.y + mirOffset.y) > (int)mode.max_size_y || irSize.x % mode.unit_size_x != 0 || irSize.y % mode.unit_size_y != 0)
				{
					log << "      Cannot support size/offset combination\n";
					continue;
				   
				}
			} else {
				irSize.x = mode.max_size_x;
				irSize.y = mode.max_size_y;
			}
			// found one
			break;
		}
		if(index == DC1394_VIDEO_MODE_FORMAT7_NUM) throw(All("Could not find any usable format!"));
		const dc1394format7mode_t & mode = modeset.mode[index];
		nMode = static_cast<dc1394video_mode_t>( DC1394_VIDEO_MODE_FORMAT7_0 + index);
	
		// At the moment, hard-code the channel to speed 400. This is maybe something to fix in future?
		error = dc1394_video_set_iso_speed(mpLDCP->pCamera, DC1394_ISO_SPEED_400);
		if(error) throw(All("Could not set ISO speed."));
	
		error = dc1394_video_set_iso_channel(mpLDCP->pCamera, nCamNumber);
		if(error) throw(All("Could not set ISO channel."));
	
		error = dc1394_video_set_mode(mpLDCP->pCamera, nMode);
		if(error) throw(All("Could not set video mode."));
	
		// frame rate calculations
		int num_packets = (int)(8000.0/fFrameRate + 0.5);
		log << "Number of packets: " << num_packets << "\n";
		int packet_size = (irSize.x * irSize.y * 8 + num_packets * 8 - 1 ) / (num_packets * 8);
		mdFramerate = fFrameRate;
		// offset calculations
		if(irOffset.x == -1){
			mirOffset.x = (mode.max_size_x - irSize.x) / 2;
			mirOffset.y = (mode.max_size_y - irSize.y) / 2;
		} else {
			mirOffset = irOffset;
		}
		mirSize = irSize;
		log << "Requesting:\n";
		log << "    packet size:    " << packet_size << "\n";
		log << "    left:           " << mirOffset.x << "\n";
		log << "    right:          " << mirOffset.y << "\n";
		log << "    width:          " << mirSize.x << "\n";
		log << "    height:         " << mirSize.y << "\n";
		error = dc1394_format7_set_roi( mpLDCP->pCamera, nMode,
								nTargetColourCoding,
								packet_size, 
								mirOffset.x, // left
								mirOffset.y, // top
								mirSize.x, // width
								mirSize.y);	 // height
		
		uint32_t pkt_size, offx, offy, posx, posy;
		error = dc1394_format7_get_roi( mpLDCP->pCamera, nMode,
								&nTargetColourCoding,
								&pkt_size, 
								&offx, // left
								&offy, // top
								&posx, // width
								&posy);	 // height
		log << "Got:\n";
		log << "    packet size:    " << pkt_size << "\n";
		log << "    left:           " << offx << "\n";
		log << "    right:          " << offy << "\n";
		log << "    width:          " << posx << "\n";
		log << "    height:         " << posy << "\n";
		log << error << "\n";
		dc1394color_filter_t filterType;
		error = dc1394_format7_get_color_filter(mpLDCP->pCamera, nMode, &filterType);
		mColourfilter = DV3_from_DC_ColourFilter(filterType, mpLDCP->pCamera->vendor_id, mpLDCP->pCamera->model_id, mpLDCP->pCamera->guid);
	}

	// Hack Alert: If someone requested raw bayer output but we have not
	// yet determined the bayer filter type, then try harder to find it.
	// This happens, if a default mode announced as MONO8 is actually a
	// raw bayer mode instead, which is a common quirk for many cameras.
	// We will try to query the format 7 bayer patterns, which will
	// probably be the same as for all other modes...
	if ((mColourfilter == UNDEFINED) && ((colourspace == RAW8)||(colourspace == RAW16))) {
		dc1394format7modeset_t modeset;
		error = dc1394_format7_get_modeset(mpLDCP->pCamera, &modeset);
		if(error) throw(All("Could not get Format 7 modes."));

		for(int index = 0; index < DC1394_VIDEO_MODE_FORMAT7_NUM ; ++index){
			dc1394color_filter_t filterType;
			dc1394video_mode_t nMode2 = static_cast<dc1394video_mode_t>( DC1394_VIDEO_MODE_FORMAT7_0 + index);
			error = dc1394_format7_get_color_filter(mpLDCP->pCamera, nMode2, &filterType);
			if (error) continue;
			mColourfilter = DV3_from_DC_ColourFilter(filterType, mpLDCP->pCamera->vendor_id, mpLDCP->pCamera->model_id, mpLDCP->pCamera->guid);

			if (mColourfilter != UNDEFINED) break;
		}
	}

	// Hack Alert: The code below sets the iso channel without this
	// having been properly allocated!	Likewise we never allocate
	// bandwidth. Both of these could be allocated if the following
	// two lines were erased, and the `0' parameter to
	// dc1394_capture_setup were changed to
	// DC1394_CAPTURE_FLAGS_DEFAULT; but this causes problems when
	// the program crashes, as the resources are not deallocated
	// properly.
	// This hack emulates what dvbuffer.cc does using libdc1394v1.
	
	// Hack to disable resource allocation -- see above
	// error = dc1394_capture_setup(mpLDCP->pCamera, 4, DC1394_CAPTURE_FLAGS_DEFAULT);
	error = dc1394_capture_setup(mpLDCP->pCamera, 4, 0);
	if(error) throw(All("Could not setup capture."));
	
	error = dc1394_video_set_transmission(mpLDCP->pCamera, DC1394_ON);					
	if(error) throw(All("Could not start ISO transmission."));			
	}

    RawDVBuffer3::~RawDVBuffer3()
    {
      if(mpLDCP->pCamera)
	{
	  dc1394_video_set_transmission(mpLDCP->pCamera, DC1394_OFF);
	  dc1394_capture_stop(mpLDCP->pCamera);
	  dc1394_camera_set_power(mpLDCP->pCamera, DC1394_OFF);
	  dc1394_camera_free(mpLDCP->pCamera);
	}
    
      if(mpLDCP->pDC1394)
	dc1394_free(mpLDCP->pDC1394);
  
      delete mpLDCP;
    }

#ifndef __APPLE__
    void RawDVBuffer3::stopAllTransmissions(void)
    {
      raw1394handle_t rawhandle = raw1394_new_handle();
      int numPorts = raw1394_get_port_info(rawhandle, NULL,0);
      for (int i=0; i<numPorts; i++) {
	raw1394handle_t port = raw1394_new_handle_on_port(i);
	if (port==NULL) continue;
	raw1394_iso_stop(port);
	raw1394_iso_shutdown(port);
	raw1394_reset_bus(port);
	raw1394_destroy_handle(port);
      }
      raw1394_destroy_handle(rawhandle);
    }
#endif //__APPLE__

    bool RawDVBuffer3::frame_pending()
    {
      return true;
    }

    VideoFrame<byte>* RawDVBuffer3::get_frame()
    {
      dc1394error_t error;
      dc1394video_frame_t *pDC1394Frame;
    
      error = dc1394_capture_dequeue(mpLDCP->pCamera,
				     DC1394_CAPTURE_POLICY_WAIT, 
				     &pDC1394Frame);
    
      if(error) throw(All("Failed on deque"));
      
      DV3::DV3Frame *pDV3Frame = new DV3::DV3Frame(pDC1394Frame);
      return pDV3Frame;
    
    }

    void RawDVBuffer3::put_frame(VideoFrame<byte> *pVidFrame)
    {
      dc1394error_t error;
      DV3::DV3Frame *pDV3Frame = dynamic_cast<DV3::DV3Frame*>(pVidFrame);
      if(!pVidFrame) throw(All("put_frame got passed an alien frame"));

      error = dc1394_capture_enqueue(mpLDCP->pCamera, pDV3Frame->mpDC1394Frame);

      if(error != DC1394_SUCCESS)
      {
        //FIXME we really need to clean up exceptions here!
        throw(All("failed to reenqueue frame"));
      }

      delete pDV3Frame;
    }

    unsigned int  RawDVBuffer3::get_feature_value(DV3Feature nFeature)
    {
      if(!mpLDCP || !mpLDCP->pCamera)
	return 0;
      dc1394error_t error;
      unsigned int nValue = 0;
      error = dc1394_feature_get_value(mpLDCP->pCamera, DC_from_DV3_Feature(nFeature), &nValue);
      if(error)
	return 0;
      else
	return nValue;
    }

    void RawDVBuffer3::set_feature_value(DV3Feature nFeature, unsigned int nValue)
    {
      if(!mpLDCP || !mpLDCP->pCamera)
	return;
      dc1394_feature_set_value(mpLDCP->pCamera, DC_from_DV3_Feature(nFeature), nValue);
    }
    
    std::pair<unsigned int,unsigned int> RawDVBuffer3::get_feature_min_max(DV3Feature nFeature)
    {
      std::pair<unsigned int, unsigned int> res;
      res.first = res.second = 0;
      if(!mpLDCP || !mpLDCP->pCamera)
	return res;
      dc1394error_t error;
      unsigned int nMin;
      unsigned int nMax;
      error = dc1394_feature_get_boundaries(mpLDCP->pCamera, 
					    DC_from_DV3_Feature(nFeature), 
					    &nMin,
					    &nMax);
      if(error)
	return res;
      res.first = nMin;
      res.second = nMax;
      return res;
    }
    
    void RawDVBuffer3::auto_on_off(DV3Feature nFeature, bool bValue)
    {
      if(!mpLDCP || !mpLDCP->pCamera)
	return;
      dc1394_feature_set_mode(mpLDCP->pCamera, DC_from_DV3_Feature(nFeature), 
			      bValue? DC1394_FEATURE_MODE_AUTO : DC1394_FEATURE_MODE_MANUAL);
    }
    
    void RawDVBuffer3::power_on_off(DV3Feature nFeature, bool bValue)
    {
      if(!mpLDCP || !mpLDCP->pCamera)
        return;
      dc1394_feature_set_power(mpLDCP->pCamera, DC_from_DV3_Feature(nFeature), 
                                  bValue? DC1394_ON : DC1394_OFF);
    }
  }

}

/*
 *  Plane-based Map
 *  Construction of plane-based maps and localization in it from RGBD Images.
 *  Copyright (c) 2012, Eduardo Fernández-Moral eduardofernandez@uma.es
 *
 *  http://code.google.com/p/PbMap******************************************************************************* /
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __PBMAP_SERIALIZER__
#define __PBMAP_SERIALIZER__


#include <fstream>
//#include <cvd/thread.h>

//#include <Utils.h>
#include <tinyxml.h>
#include "PbMap.h"
#include "Plane.h"


//#define PBMAP_XML_ID "PbMap"
#define PBMAP_XML_ID "PTAM_Xtion"
#define PBMAP_VERSION "1.1"

/**
 * This class is used for serializing and deserializing maps into a xml format
 */
class PbMapSerializer //: public CVD::Thread
{
  public:

    enum MapStatus { MAP_OK, MAP_FAILED2, MAP_EXISTS };

    PbMapSerializer(PbMap &mPbm);
    ~PbMapSerializer();

    bool Init( std::string sCommand, std::string sParams );

    MapStatus SavePlanes( std::string sFileName );
    MapStatus LoadPbMap( std::string sFileName );

    virtual void run();

  private:

    std::string msDirName;                                  // The directory to save the map(s) to
    PbMap &mPbm;

    bool mbOK;                                              // Init() has been run.

    bool _SaveAMapPlane( Plane &mp, TiXmlElement * mapPlaneNode );
    bool _SaveMapPlanes( TiXmlElement * rootNode );

//    void _CreateSaveLUTs();
//    int _LookupKeyFrame( KeyFrame * k );
//    int _LookupMapPoint( MapPoint * m );
//
//    bool _SaveGlobalPosition(TiXmlElement *submapNode);
//    bool _SaveNeighbors(TiXmlElement *submapNode);
//    bool _SaveSSO(TiXmlElement *submapNode);
//    bool _SaveNeigSSO(TiXmlElement *submapNode);

//    //loading
//    void Reset();
//    MapStatus _LoadMap( std::string sDirName );
//
//    bool _LoadAKeyFrame( TiXmlHandle &hKF, const std::string & sPath, bool bQueueFrame = false );
//    bool _LoadKeyFrames( TiXmlHandle &hRoot, const std::string & sPath );
//
//    bool _LoadAMapPoint( TiXmlHandle &hMP, bool bQueuePoint = false );
//    bool _LoadMapPoints( TiXmlHandle &hRoot );

    bool _LoadAMapPlane( TiXmlHandle &hMP );
    bool _LoadMapPlanes( TiXmlHandle &hRoot );

//    bool _LoadGlobalPosition(TiXmlHandle &hMP);
//    bool _LoadNeighbors(TiXmlHandle &hMP);
//    bool _LoadSSO(TiXmlHandle &hMP);
//    bool _LoadNeigSSO(TiXmlHandle &hMP);
//
//    KeyFrame * _LookupKeyFrame( int uid );
//    MapPoint * _LookupMapPoint( int uid );
//    bool _CrossReferencing(TiXmlHandle &hRoot);
//
////    bool _LoadSSO();
////    bool _LoadSSOshare();
//
//    //other
//    bool _LockMap();
//    void _UnlockMap();
//    void _UnRegisterWithMap();
//    void _RegisterWithMap( Map * map );
//    void _CleanUp();
////    Map * _FindTheMap( std::string sParam );

//    std::map< const MapPoint *, int > mmMapPointSaveLUT;    // lookup table for assigning uid to mappoints
//    std::map< const KeyFrame *, int > mmKeyFrameSaveLUT;    // lookup table for assigning uid to keyframes
//    std::map< int, MapPoint * >       mmMapPointLoadLUT;    // lookup table for assigning uid to mappoints
//    std::map< int, KeyFrame * >       mmKeyFrameLoadLUT;    // lookup table for assigning uid to keyframes
//    //look up table for cross referencing keyframes with map points. need this as load keyframes then map points. then cross ref.
//    std::map< KeyFrame*, std::vector< std::pair< int, Measurement > > > mmMeasCrossRef;
//    std::vector< std::pair< int, int > > mvFailureQueueUIDs;
//
    std::string msCommand;                                  // The command passed to Init()
    std::string msParams;                                   // The params passed to Init()
//    Map * mpInitMap;                                        // The map passed to Init(). The current map.

//    void ResetAll();
//    MapMaker *mMapMaker;
//    Tracker *mTracker;
//    LoopClosure *mLoopClosure;

  /*!PbMapSerializer's stop controller*/
  bool	m_pbMapSerialiser_runing;

  /*!PbMapSerializer's stop var*/
  bool	m_pbMapSerialiser_finished;
};


#endif


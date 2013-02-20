/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

/*  Plane-based Map (PbMap) library
 *  Construction of plane-based maps and localization in it from RGBD Images.
 *  Writen by Eduardo Fernandez-Moral. See docs for <a href="group__mrpt__pbmap__grp.html" >mrpt-pbmap</a>
 */
#include <mrpt/pbmap.h> // precomp. hdr

#include <mrpt/pbmap/PbMapSerializer.h>
#include <mrpt/pbmap/Miscellaneous.h>
#include <gvars3/GStringUtil.h>

#include <iostream>
#include <iomanip>
//#include <cvd/vision.h>
//#include <cvd/image_io.h>

#include <sys/stat.h>
#include <sys/types.h>

#include <dirent.h> // To read files from disk

#ifdef WIN32
#include "direct.h"
#endif


using namespace std;
using namespace GVars3;

/**
 * Constructor
 */
PbMapSerializer::PbMapSerializer(PbMap &mPbm) :
//    mPbm.vPlanes(planes),
//    mPbm.FloorPlane(floorPlaneID),
    m_pbMapSerialiser_runing(false),
    m_pbMapSerialiser_finished(false),
    mPbm(mPbm),
    mbOK(false)
{
}

/**
 * Initialize the map serializer with the command and parameters required.
 * The current map is also passed in case this needs to be known.
 * @param sCommand The serialization command
 * @param sParams The parameters, such as map number and/or path
 */
bool PbMapSerializer::Init( std::string sCommand, std::string sParams )//, Map &currentMap )
{
//  if( isRunning() )
  if( m_pbMapSerialiser_runing )
  {
    cout << "Serialization is currently running. Please try again in a moment." << endl;
    return false;
  }
//  cout << "Serializing...\n"; // QUITAR
//  mbOK = false;

  msCommand = sCommand;
  msParams = sParams;

  mbOK = true;

  return true;
}


/**
 * Load a map plane
 * @param hMP XML node
 * @return success
 */
bool PbMapSerializer::_LoadAMapPlane( TiXmlHandle &hMP )
{
//cout << "_LoadAMapPlane()\n";

  int uid = -1;
  hMP.ToElement()->QueryIntAttribute("id", &uid);
  if( uid == -1 ) {
    cerr << " keyframe has no ID. aborting" << endl;
    return false;
  }

  //////////////////// fill in map plane

  Plane *mp = new Plane();

  mp->id = static_cast<unsigned>(uid);

  string sTmp = hMP.ToElement()->Attribute( "normal" );
  sscanf( sTmp.c_str(), "%lf %lf %lf", &mp->v3normal[0], &mp->v3normal[1], &mp->v3normal[2]);

  sTmp = hMP.ToElement()->Attribute( "center" );
  sscanf( sTmp.c_str(), "%lf %lf %lf", &mp->v3center[0], &mp->v3center[1], &mp->v3center[2]);

  sTmp = hMP.ToElement()->Attribute( "RGB" );
  sscanf( sTmp.c_str(), "%f %f %f", &mp->v3colorNrgb[0], &mp->v3colorNrgb[1], &mp->v3colorNrgb[2]);

//  sTmp = hMP.ToElement()->Attribute( "ColorDev" );
//  sscanf( sTmp.c_str(), "%f %f %f", &mp->v3colorNrgbDev[0], &mp->v3colorNrgbDev[1], &mp->v3colorNrgbDev[2]);

  hMP.ToElement()->QueryIntAttribute( "numObservations", &uid );
  mp->numObservations = static_cast<unsigned>(uid);

  hMP.ToElement()->QueryDoubleAttribute( "areaNumVoxels", &mp->areaVoxels );

  hMP.ToElement()->QueryDoubleAttribute( "areaHull", &mp->areaHull );

  hMP.ToElement()->QueryDoubleAttribute( "ratioXY", &mp->elongation );

  sTmp = hMP.ToElement()->Attribute( "PpalComp" );
  sscanf( sTmp.c_str(), "%lf %lf %lf", &mp->v3PpalDir[0], &mp->v3PpalDir[1], &mp->v3PpalDir[2]);

  hMP.ToElement()->QueryIntAttribute( "fullArea", &uid );
  mp->bFullExtent = (uid == 1) ? true : false;

  hMP.ToElement()->QueryIntAttribute( "structure", &uid );
  mp->bFromStructure = (uid == 1) ? true : false;

  hMP.ToElement()->QueryValueAttribute( "label", &mp->label );

  if( mp->label.substr(0,5) == "Floor")
    mPbm.FloorPlane = mp->id;

  //load neighbor index
  {
    int nSize = -1;
    TiXmlElement* pElem = hMP.FirstChild("Neighbors").ToElement();

    pElem->QueryIntAttribute("size", &nSize);
    if( nSize == -1 ) {
      cerr << " submap has not a valid number of neighbors. aborting" << endl;
      return false;
    }

    const char *str = pElem->GetText();
    TiXmlElement* pElem2 = hMP.FirstChild("CommonObservations").ToElement();
    const char *str2 = pElem2->GetText();
    string sTmp2;
    if(str != NULL )  {
      sTmp = str;
      sTmp2 = str2;
    }
    else  {
      sTmp = "";
      sTmp2 = "";
    }

    std::vector<string> vsUids, vsUids2;
    vsUids = ChopAndUnquoteString(sTmp);
    vsUids2 = ChopAndUnquoteString(sTmp2);

    assert( vsUids.size() == nSize && vsUids2.size() == nSize );

    for(size_t i = 0; i < vsUids.size(); i++)
    {
      int *pN = ParseAndAllocate<int>(vsUids[i]);
      int *pN2 = ParseAndAllocate<int>(vsUids2[i]);
      if( pN )
      {
        mp->neighborPlanes.insert( pair<unsigned,unsigned>(*pN, *pN2) );
        delete pN;
      }
    }
  }

  // Load the convex hull vertex
  {
    int nSize = -1;
    TiXmlHandle hConvexHull = hMP.FirstChild("ConvexHull").ToElement();
    hConvexHull.ToElement()->QueryIntAttribute("size", &nSize);

    if( nSize == -1 ) {
      cerr << " convex hull has not a valid number of vertex. aborting" << endl;
      return false;
    }

    mp->polygonContourPtr->resize(nSize);
    unsigned counter = 0;
    for(TiXmlElement* pNode = hConvexHull.FirstChild().Element();
        pNode != NULL;
        pNode = pNode->NextSiblingElement() )
    {
      pcl::PointXYZRGBA point;
      sTmp = pNode->Attribute("pos");
      sscanf( sTmp.c_str(), "%f %f %f", &point.x, &point.y, &point.z );
      mp->polygonContourPtr->points[counter] = point;
      counter++;
    }
  }


  mPbm.vPlanes.push_back( *mp );

  return true;
}


/**
 * Load a list of planes recursivly
 * @param hRoot XML node
 * @return success
 */
bool PbMapSerializer::_LoadMapPlanes( TiXmlHandle &hRoot )
{
//cout << "PbMapSerializer::_LoadMapPlanes()\n";

  int nSize = -1;
  TiXmlHandle pNode = hRoot.FirstChild( "MapPlanes" );
  pNode.ToElement()->QueryIntAttribute("size", &nSize);

//cout << nSize << " planes to load\n";

  for(TiXmlElement* pElem = pNode.FirstChild().Element();
      pElem != NULL;
      pElem = pElem->NextSiblingElement() )
  {
    TiXmlHandle hmp( pElem );
    if( !_LoadAMapPlane( hmp ) ) {
      cerr << "Failed to Load plane " << pElem->Attribute("id") << ". Abort." << endl;
      return false;
    }
  }

  if( (int)mPbm.vPlanes.size() != nSize ) {
    cerr << "Loaded the wrong number of mapplanes. " << mPbm.vPlanes.size()
        << " instead of " << nSize << ". Aborting" << endl;
    return false;
  }

  return true;
}


/**
 * Load a map specified in the sFileName.
 * @param sFileName the directory containing the map
 * @return success
 */
PbMapSerializer::MapStatus PbMapSerializer::LoadPbMap( std::string sFileName )
{
//  cout << "PbMapSerializer::LoadPbMap from " << sFileName << endl;

  mPbm.vPlanes.clear();

  MapStatus ms = MAP_OK;

//  if( pMap == NULL ) {
//    cerr << "LoadMap: got a NULL map pointer. abort" << endl;
//    return MAP_FAILED2;
//  }

//  _RegisterWithMap( pMap );

  TiXmlDocument mXMLDoc;                                 //XML file

  //load the XML file
  if( !mXMLDoc.LoadFile( sFileName ) )  {
    cerr << "Failed to load " << sFileName << ". Aborting." << endl;
    return MAP_FAILED2;
  }

  TiXmlHandle hDoc(&mXMLDoc);
  TiXmlElement* pElem;
  TiXmlHandle hRoot(0);

  pElem = hDoc.FirstChildElement().Element();
  // should always have a valid root but handle gracefully if it does not
  if (!pElem)
  {
    cerr << "No root handle in XML file " << sFileName << endl;
    return MAP_FAILED2;
  }

  string sID( PBMAP_XML_ID );
  string sVersion( PBMAP_VERSION );
  string sFileVersion = pElem->Attribute("version");

  if( ( sID.compare( pElem->Value() ) != 0 ) &&
        ( sVersion.compare( sFileVersion ) != 1.1 ) )
  {
    cerr << "Invalid XML file. Need a version " << sVersion << " " << sID
         << " XML file. Not a version " << sFileVersion << " " << pElem->Value() << " file." << endl;
    return MAP_FAILED2;
  }

  // save this for later
  hRoot = TiXmlHandle(pElem);

//cout << "Submap Locked by " << this << "\n";
//for( map< void *, bool >::iterator i = mpMap->mapLockManager.mLockRecords.begin(); i != mpMap->mapLockManager.mLockRecords.end(); i++ )
//  cout << "Thread " << i->first << " state " << i->second << endl;

//  ////////////  Get map lock  ////////////
//  if( !_LockMap() ) {
//    cerr << "Failed to get map lock" << endl;
//    return MAP_FAILED2;
//  }

  bool bOK = false;

  // load map points
  bOK = _LoadMapPlanes( hRoot );
  if( !bOK ) {
    cout << "LoadPbMap failed\n";
//    mpMap->Reset();
//    _UnlockMap();
    return MAP_FAILED2;
  }

  ////////////  relase map lock  ////////////
//  _UnlockMap();

  return MAP_OK;
}


/**
 * save a map plane
 * @param mp the map plane to save
 * @param mapPlanesNode  XML node
 * @return success
 */
bool PbMapSerializer::_SaveAMapPlane( Plane &mp, TiXmlElement * mapPlanesNode )
{
  TiXmlElement* mpe = new TiXmlElement("MapPlane");
  mapPlanesNode->LinkEndChild( mpe );

  mpe->SetAttribute( "id", mp.id );

  mpe->SetAttribute( "numObservations", mp.numObservations );

  mpe->SetDoubleAttribute( "areaNumVoxels", mp.areaVoxels);

  mpe->SetDoubleAttribute( "areaHull", mp.areaHull );

  mpe->SetAttribute( "fullArea", mp.bFullExtent ? 1 : 0 );

  mpe->SetDoubleAttribute( "ratioXY", mp.elongation );

  mpe->SetAttribute( "structure", mp.bFromStructure ? 1 : 0 );

  if( mp.label == "" )
  {
    char label[32];
    sprintf(label, "P%u", mp.id);
    mp.label = label;
  }
  mpe->SetAttribute( "label", mp.label );

  string s;
  ostringstream os;

  os << mp.v3normal;
  s = os.str();
  PruneWhiteSpace( s );
  mpe->SetAttribute("normal", s );

  os.str("");
  os << mp.v3center;
  s = os.str();
  PruneWhiteSpace( s );
  mpe->SetAttribute("center", s );

  os.str("");
  os << mp.v3PpalDir;
  s = os.str();
  PruneWhiteSpace( s );
  mpe->SetAttribute("PpalComp", s );

  os.str("");
//  os << mp.v3colorC1C2C3;
  os << mp.v3colorNrgb;
  s = os.str();
  PruneWhiteSpace( s );
  mpe->SetAttribute("RGB", s );

//  os.str("");
//  os << mp.v3colorNrgbDev;
//  s = os.str();
//  PruneWhiteSpace( s );
//  mpe->SetAttribute("ColorDev", s );

  //save neighbors
  {
    TiXmlElement * pElem = new TiXmlElement( "Neighbors" );
    mpe->LinkEndChild( pElem );
    pElem->SetAttribute("size", mp.neighborPlanes.size() );
    os.str("");
    for(std::map<unsigned,unsigned>::iterator it=mp.neighborPlanes.begin(); it != mp.neighborPlanes.end(); it++)
      os << it->first << " ";
    pElem->LinkEndChild( new TiXmlText(os.str() ) );

    // Save edge weights (number of common observations)
    pElem = new TiXmlElement( "CommonObservations" );
    mpe->LinkEndChild( pElem );
//    pElem->SetAttribute("size", mp.neighborPlanes.size() );
    os.str("");
    for(std::map<unsigned,unsigned>::iterator it=mp.neighborPlanes.begin(); it != mp.neighborPlanes.end(); it++)
      os << it->second << " ";
    pElem->LinkEndChild( new TiXmlText(os.str() ) );
  }

  // Save points of convex hull
  TiXmlElement * pCHull = new TiXmlElement( "ConvexHull" );
  mpe->LinkEndChild( pCHull );
  pCHull->SetAttribute("size", mp.polygonContourPtr->size() );
  for(unsigned i=0; i < mp.polygonContourPtr->size(); i++)
  {
    TiXmlElement * pCHullVertex = new TiXmlElement( "Vertex" );
    pCHull->LinkEndChild( pCHullVertex );
    os.str("");
    os << mp.polygonContourPtr->points[i].x << " " << mp.polygonContourPtr->points[i].y << " " << mp.polygonContourPtr->points[i].z;
    s = os.str();
    PruneWhiteSpace( s );
    pCHullVertex->SetAttribute("pos", s );
  }

  return true;
}

/**
 * save all the mappoints in a map.
 * @param rootNode The root node to write to in the XML file
 * @return success.
 */
bool PbMapSerializer::_SaveMapPlanes( TiXmlElement * rootNode )
{
  vector<Plane>::iterator mp;
  vector<int> uids;
  int m = -1;

  TiXmlElement * mapPlanesNode = new TiXmlElement( "MapPlanes" );
  rootNode->LinkEndChild( mapPlanesNode );
  mapPlanesNode->SetAttribute( "size", mPbm.vPlanes.size() );

  //all points are saved, bad or not. don't save trash though
  for( mp = mPbm.vPlanes.begin(); mp != mPbm.vPlanes.end(); mp++ )
  {
    if( !_SaveAMapPlane( *mp, mapPlanesNode) )
    {
      cerr << " could not save map plane " << endl;
      return false;
    }
  }

  return true;
}


/** Save the map out.
 * if no sFileName is provided then the map number will be used.
 * otherwise map will be saved to path/to/sFileName
 *
 * @param sFileName the filename ("mapname" or "path/to/mapname" or "")
 * @return status
 */
PbMapSerializer::MapStatus PbMapSerializer::SavePlanes( string sFileName )
{
  cout << "PbMapSerializer::SavePbMap...\n"; // QUITAR
  MapStatus ms = MAP_OK;

  if( mPbm.vPlanes.empty() ) {
    cerr << "SaveMap: got no planes -> abort" << endl;
    return MAP_FAILED2;
  }

  //recusively save the map planes

  ////////////  Get PbMapMaker lock  ////////////
//  if( !_LockMap() ) {
//    cerr << "Failed to get PbMapMaker lock" << endl;
//    return MAP_FAILED2;
//  }

  TiXmlDocument xmlDoc;     //XML file

  TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "", "" );
  xmlDoc.LinkEndChild( decl );

  TiXmlElement * rootNode = new TiXmlElement(PBMAP_XML_ID);
  xmlDoc.LinkEndChild( rootNode );
  rootNode->SetAttribute("version", PBMAP_VERSION);

  bool bOK = false;

  // save submap info

  ////////////  save the map planes  ////////////
  bOK = _SaveMapPlanes( rootNode );   // recursively save each map plane
  if( !bOK )  {
//    _UnlockMap();
    return MAP_FAILED2;
  }
//cout << "PbMapSerializer::_SaveMapPlanes\n"; // QUITAR

  ////////////  relase map lock  ////////////
//  _UnlockMap();

  xmlDoc.SaveFile(sFileName);
cout << "xmlDoc.SaveFile\n"; // QUITAR
  return MAP_OK;
}


/** Called when the thread is run.
 * It calls the appropiate function in the PbMapSerializer based on the input options
 */
void PbMapSerializer::run()
{
// { mrpt::synch::CCriticalSectionLocker csl(&CS_LC);
// { mrpt::synch::CCriticalSectionLocker csl(&CS_RM_MS);
  m_pbMapSerialiser_runing = true;

  if( !mbOK ) {
    cerr << " Call Init() before running the thread!" << endl;
    return;
  }

//  Map * pMap = _ParseCommandAndParameters();

// eTODO Implement function to load msDirName

//  //if a map was specified then it is either load or save a map
//  if( pMap ) {

    if( msCommand == "SavePbMap" )  {
      SavePlanes( msParams );
    }
    else if( msCommand == "LoadPbMap" )  {
      LoadPbMap( msParams );
    }

  mbOK = false;

  m_pbMapSerialiser_runing = false;
}

/**
 * Destructor
 */
PbMapSerializer::~PbMapSerializer()
{
}

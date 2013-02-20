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

#ifndef __PBMAPLOCALISER_H
#define __PBMAPLOCALISER_H

#include <mrpt/config.h>
#include <mrpt/utils/utils_defs.h>

#include <mrpt/system/threads.h>

#include "PbMap.h"
#include "SubgraphMatcher.h"
#include "ConsistencyTest.h"

namespace mrpt {
namespace pbmap {

  /*! This class is used to explore the PbMap (or other previously acquired PbMaps)
   *  to find places observed previously (it has uses in e.g. place recognition or relocalization).
   *  PbMapLocaliser run its own thread, which is created at initialization.
   * \ingroup mrpt_pbmap_grp
   */
  class PbMapLocaliser
  {
   public:

  /*!Constructor.*/
    PbMapLocaliser(PbMap &mPbM);
    ~PbMapLocaliser();

  /*!Vector to store the name of previous PbMaps (previous places).*/
    std::vector<std::string> previousPbMapNames;

  /*!Vector of vectors containing previous PbMaps.*/
    std::vector<PbMap> previousPbMaps;

  ///*!Vector to store the index of the floor plane for the previous PbMaps (-1 indicates that the floor was not detected).*/  // Mover a la clase PbMapLocaliser  y montarlo en un define
  //  std::vector<int> vFloors;

  /*!Number of planes of our search space.*/
    size_t totalPrevPlanes;

    /*!observedPlanes is a list containing the current observed planes.*/
    std::vector<unsigned> vQueueObservedPlanes;

    /*!List of places where the system has been localised, keeping also the associated point cloud corresponding to the PbMap.*/  // Mover a la clase PbMapLocaliser
    std::map<std::string, pcl::PointXYZ> foundPlaces;

    /*!Point cloud of recognized place.*/
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr alignedModelPtr;

    map<unsigned, vector< pair<double,int> > > evalColor;

   private:

    /*!The current PbMap.*/
    PbMap &mPbMap;

    /*!The matching object.*/
    SubgraphMatcher matcher;

    /*!Load previous PbMaps to search for previous places.*/
    void LoadPreviousPbMaps(std::string fileMaps);

    /*!List of places that have been matched, together with their plane correspondences.*/  // Cambiar nombre
    std::map<std::string, std::pair<int,double> > planeRecognitionLUT;

    /*!Search the subgraph defined by a plane (neighborhood of 1-connected planes) in the rest of the PbMap or PbMaps aquired till the moment.*/  // Cambiar nombre
    bool searchPlaneContext(Plane &searchPlane);

    /*!Search the 2nd order neighbors.*/
    void compareSubgraphNeighbors(SubgraphMatcher &matcher);

    double getAreaMatch();

    /*!Best previous PbMap correspondence.*/
    unsigned bestMap;

    /*!Best correspondence between pair of planes.*/
    map<unsigned, unsigned> bestMatch;

    /*!Score of the matched places.*/  // Cambiar nombre o Quitar!
    double score;

   protected:

  //  /*!PbMapLocaliser command callback"*/
  //  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);

    /*!This executes the PbMapLocaliser's thread*/
    void run();

    /*!PbMapLocaliser's thread handle*/
    mrpt::system::TThreadHandle pbMapLocaliser_hd;

    /*!PbMapLocaliser's exit thread*/
    bool stop_pbMapLocaliser();

    /*!PbMapLocaliser's stop controller*/
    bool	m_pbMapLocaliser_must_stop;

    /*!PbMapLocaliser's stop var*/
    bool	m_pbMapLocaliser_finished;
  };

} } // End of namespaces

#endif

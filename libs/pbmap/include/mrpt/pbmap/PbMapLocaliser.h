/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*  Plane-based Map (PbMap) library
 *  Construction of plane-based maps and localization in it from RGBD Images.
 *  Writen by Eduardo Fernandez-Moral. See docs for <a href="group__mrpt__pbmap__grp.html" >mrpt-pbmap</a>
 */

#ifndef __PBMAPLOCALISER_H
#define __PBMAPLOCALISER_H

#include <mrpt/config.h>

#if MRPT_HAS_PCL

#include <mrpt/utils/utils_defs.h>

#include <mrpt/system/threads.h>

#include <mrpt/pbmap/PbMap.h>
#include <mrpt/pbmap/SubgraphMatcher.h>
#include <mrpt/pbmap/ConsistencyTest.h>
#include <mrpt/pbmap/link_pragmas.h>

namespace mrpt {
namespace pbmap {

  /*! This class is used to explore the PbMap (or other previously acquired PbMaps)
   *  to find places observed previously (it has uses in e.g. place recognition or relocalization).
   *  PbMapLocaliser run its own thread, which is created at initialization.
   *
   * \ingroup mrpt_pbmap_grp
   */
  class PBMAP_IMPEXP PbMapLocaliser
  {
   public:

  /*!Constructor.*/
    PbMapLocaliser(PbMap &mPbM, const std::string &config_file);
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

    std::map<unsigned, std::vector< std::pair<double,int> > > evalColor;

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
    std::map<unsigned, unsigned> bestMatch;

    /*!Score of the matched places.*/  // Cambiar nombre o Quitar!
    double score;

   protected:

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

#endif

/* bindings */
#include "bindings.h"
#include "maps_bindings.h"
#include "system_bindings.h"

/* MRPT */
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/system/datetime.h>
#include <mrpt/maps/CMultiMetricMapPDF.h>
#include <mrpt/opengl/CSetOfObjects.h>

/* std */
#include <stdint.h>

/* namespaces */
using namespace boost::python;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::utils;


// CMetricMap
void CMetricMapWrap::internal_clear()
{
    this->get_override("internal_clear")();
}

bool CMetricMapWrap::isEmpty()
{
    return this->get_override("isEmpty")();
}

double CMetricMapWrap::computeObservationLikelihood(const CObservation *obs, const CPose3D &takenFrom)
{
    return this->get_override("computeObservationLikelihood")(obs, takenFrom);
}

void CMetricMapWrap::saveMetricMapRepresentationToFile(const std::string &filNamePrefix)
{
    this->get_override("saveMetricMapRepresentationToFile")(filNamePrefix);
}

void CMetricMapWrap::getAs3DObject(mrpt::opengl::CSetOfObjectsPtr &outObj)
{
    this->get_override("getAs3DObject")(outObj);
}
// end of CMetricMap

// COccupancyGridMap2D
COccupancyGridMap2D *COccupancyGridMap2D_copy(COccupancyGridMap2D &self)
{
  return (COccupancyGridMap2D *)self.clone();
}

bool COccupancyGridMap2D_insertObservation(COccupancyGridMap2D &self, CObservation &obs, CPose3D &pose)
{
    return self.insertObservation(&obs, &pose);
}

object COccupancyGridMap2D_to_ROS_OccupancyGrid_msg1(COccupancyGridMap2D &self, str frame_id)
{
    // import msg
    dict locals;
    exec("from nav_msgs.msg import OccupancyGrid\n"
         "occupancy_grid_msg = OccupancyGrid()\n",
         object(), locals);
    object occupancy_grid_msg = locals["occupancy_grid_msg"];
    // set info
    int32_t width = self.getSizeX();
    int32_t height = self.getSizeY();
    occupancy_grid_msg.attr("header").attr("frame_id") = frame_id;
    occupancy_grid_msg.attr("header").attr("stamp") = TTimeStamp_to_ROS_Time(long_(mrpt::system::now()));
    occupancy_grid_msg.attr("info").attr("width") = width;
    occupancy_grid_msg.attr("info").attr("height") = height;
    occupancy_grid_msg.attr("info").attr("resolution") = self.getResolution();
    occupancy_grid_msg.attr("info").attr("origin").attr("position").attr("x") = self.getXMin();
    occupancy_grid_msg.attr("info").attr("origin").attr("position").attr("y") = self.getYMin();
    // set data
    boost::python::list data;
    for (int32_t y = 0; y < height; ++y) {
        for (int32_t x = 0; x < width; ++x) {
            float occupancy = self.getCell(x,y);
            if (occupancy < 0.45) {
                data.append(100);
            } else if (occupancy > 0.55){
                data.append(0);
            } else {
                data.append(-1);
            }
        }
    }
    occupancy_grid_msg.attr("data") = data;
    return occupancy_grid_msg;
}

#ifdef ROS_EXTENSIONS
object COccupancyGridMap2D_to_ROS_OccupancyGrid_msg2(COccupancyGridMap2D &self)
{
    return COccupancyGridMap2D_to_ROS_OccupancyGrid_msg1(self, str("map"));
}

void COccupancyGridMap2D_from_ROS_OccupancyGrid_msg(COccupancyGridMap2D &self, object occupancy_grid_msg)
{
    // set info
    float x_min = extract<float>(occupancy_grid_msg.attr("info").attr("origin").attr("position").attr("x"));
    float y_min = extract<float>(occupancy_grid_msg.attr("info").attr("origin").attr("position").attr("y"));
    float resolution = extract<float>(occupancy_grid_msg.attr("info").attr("resolution"));
    int32_t width = extract<int32_t>(occupancy_grid_msg.attr("info").attr("width"));
    int32_t height = extract<int32_t>(occupancy_grid_msg.attr("info").attr("height"));
    float x_max = x_min + width * resolution;
    float y_max = y_min + height * resolution;
    self.setSize(x_min, x_max, y_min, y_max, resolution);
    // set data
    int32_t idx = 0;
    boost::python::list data = extract<boost::python::list>(occupancy_grid_msg.attr("data"));
    for (int32_t y = 0; y < height; ++y) {
        for (int32_t x = 0; x < width; ++x) {
            int32_t occupancy = extract<int32_t>(data[idx]);
            if (occupancy >= 0) {
                self.setCell(x, y, (100 - occupancy) / 100.0);
            } else {
                self.setCell(x, y, 0.5);
            }
            idx++;
        }
    }
}
#endif

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(COccupancyGridMap2D_fill_overloads, fill, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(COccupancyGridMap2D_setSize_overloads, setSize, 5, 6)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(COccupancyGridMap2D_resizeGrid_overloads, resizeGrid, 4, 6)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(COccupancyGridMap2D_loadFromBitmapFile_overloads, loadFromBitmapFile, 2, 4)
// end of COccupancyGridMap2D


// CPointsMap
struct CPointsMapWrap : CPointsMap, wrapper<CPointsMap>
{
    CObject *duplicate() const
    {
        return this->get_override("duplicate")();
    }

    void PLY_import_set_vertex_count(size_t count)
    {
        this->get_override("PLY_import_set_vertex_count")(count);
    }

    void reserve(size_t newLength)
    {
        this->get_override("reserve")(newLength);
    }

    void resize(size_t newLength)
    {
        this->get_override("resize")(newLength);
    }

    void setSize(size_t newLength)
    {
        this->get_override("setSize")(newLength);
    }

    void setPointFast(size_t index, float x, float y, float z)
    {
        this->get_override("setPointFast")(index, x, y, z);
    }

    void insertPointFast(float x, float y, float z)
    {
        this->get_override("insertPointFast")(x, y, z);
    }

    void copyFrom(const CPointsMap &obj)
    {
        this->get_override("copyFrom")(obj);
    }

    void getPointAllFieldsFast(const size_t index, std::vector<float> &point_data)
    {
        this->get_override("getPointAllFieldsFast")(index, point_data);
    }

    void setPointAllFieldsFast(const size_t index, const std::vector<float> &point_data)
    {
        this->get_override("setPointAllFieldsFast")(index, point_data);
    }
};
// end of CPointsMap

// CSimplePointsMap
void CSimplePointsMap_loadFromRangeScan1(CSimplePointsMap &self, const CObservation2DRangeScan &rangeScan)
{
    self.loadFromRangeScan(rangeScan);
}

void CSimplePointsMap_loadFromRangeScan2(CSimplePointsMap &self, const CObservation2DRangeScan &rangeScan, const CPose3D &robotPose)
{
    self.loadFromRangeScan(rangeScan, &robotPose);
}

boost::python::tuple CSimplePointsMap_getPointAllFieldsFast(CSimplePointsMap &self, uint32_t index)
{
    boost::python::list ret_val;
    std::vector<float> point_xyz;
    self.getPointAllFieldsFast(index, point_xyz);
    ret_val.append(point_xyz[0]);
    ret_val.append(point_xyz[1]);
    ret_val.append(point_xyz[2]);
    return boost::python::tuple(ret_val);
}

boost::python::tuple CSimplePointsMap_getPointFast(CSimplePointsMap &self, uint32_t index)
{
    boost::python::list ret_val;
    float x, y, z;
    self.getPointFast(index, x, y, z);
    ret_val.append(x);
    ret_val.append(y);
    ret_val.append(z);
    return boost::python::tuple(ret_val);
}

uint32_t CSimplePointsMap_getSize(CSimplePointsMap &self)
{
    return self.getPointsBufferRef_x().size();
}
// end of CSimplePointsMap

// CSimpleMap
void CSimpleMap_insert(CSimpleMap &self, CPose3DPDF &in_posePDF, CSensoryFrame &in_SF)
{
    // create smart pointers
    CPose3DPDFPtr in_posePDFPtr = (CPose3DPDFPtr) in_posePDF.duplicateGetSmartPtr();
    CSensoryFramePtr in_SFPtr = (CSensoryFramePtr) in_SF.duplicateGetSmartPtr();
    // insert smart pointers
    self.insert(in_posePDFPtr, in_SFPtr);
}
// end of CSimpleMap

// CMultiMetricMapPDF
CPose2D CMultiMetricMapPDF_getLastPose(CMultiMetricMapPDF& self, size_t i)
{
    mrpt::math::TPose3D last_pose = *(self.getLastPose(i));
    return CPose2D(last_pose);
}

list CMultiMetricMapPDF_getPath(CMultiMetricMapPDF& self, size_t i)
{
    std::deque<mrpt::math::TPose3D> path;
    list ret_val;
    self.getPath(i, path);
    for (size_t i = 0; i < path.size(); ++i) ret_val.append(path[i]);
    return ret_val;
}

CPose3DPDFParticles CMultiMetricMapPDF_getEstimatedPosePDFAtTime(CMultiMetricMapPDF& self, size_t timeStep)
{
    CPose3DPDFParticles out_estimation;
    self.getEstimatedPosePDFAtTime(timeStep, out_estimation);
    return out_estimation;
}
// end of CMultiMetricMapPDF

// FIXME
#ifdef MRPT_BELOW_1_3
// TMetricMapInitializer
struct TMetricMapInitializerWrap : TMetricMapInitializer, boost::python::wrapper<TMetricMapInitializer>
{
    void loadFromConfigFile_map_specific(const mrpt::utils::CConfigFileBase& source, const std::string& sectionNamePrefix)
    {
        this->get_override("loadFromConfigFile_map_specific")(source, sectionNamePrefix);
    }

    void dumpToTextStream_map_specific(mrpt::utils::CStream &out) const
    {
        this->get_override("dumpToTextStream_map_specific")(out);
    }
};
// end of TMetricMapInitializer
#endif

// CMultiMetricMap
mrpt::opengl::CSetOfObjectsPtr CMultiMetricMap_getAs3DObject(CMultiMetricMap &self)
{
    mrpt::opengl::CSetOfObjectsPtr outObj = mrpt::opengl::CSetOfObjects::Create();
    self.getAs3DObject(outObj);
    return outObj;
}
// end of CMultiMetricMap

void export_maps()
{
    // map namespace to be submodule of package
    MAKE_SUBMODULE(maps)

    // CMetricMap
    {
        scope s = class_<CMetricMapWrap, boost::noncopyable>("CMetricMap", no_init)
        ;
    }

    // COccupancyGridMap2D
    {
        scope s = class_<COccupancyGridMap2D>("COccupancyGridMap2D", init<optional<float, float, float, float, float> >())
            .def("fill", &COccupancyGridMap2D::fill, COccupancyGridMap2D_fill_overloads()) //, "Fills all the cells with a default value.")
            .def("setSize", &COccupancyGridMap2D::setSize, COccupancyGridMap2D_setSize_overloads()) //, "Change the size of gridmap, erasing all its previous contents.")
            .def("resizeGrid", &COccupancyGridMap2D::resizeGrid, COccupancyGridMap2D_resizeGrid_overloads()) //, "Change the size of gridmap, maintaining previous contents.")
            .def("insertObservation", &COccupancyGridMap2D_insertObservation, "Insert the observation information into this map.\n:param: obs The Observation\n:param: robotPose The 3D pose of the robot mobile base in the map reference system.")
            .def("saveAsBitmapFile", &COccupancyGridMap2D::saveAsBitmapFile, "Saves the gridmap as a graphical file (BMP,PNG,...).")
            .def("loadFromBitmapFile", &COccupancyGridMap2D::loadFromBitmapFile, COccupancyGridMap2D_loadFromBitmapFile_overloads()) //, "Load the gridmap from a image in a file (the format can be any supported by CImage::loadFromFile).")
            .def("getSizeX", &COccupancyGridMap2D::getSizeX, "Returns the horizontal size of grid map in cells count.")
            .def("getSizeY", &COccupancyGridMap2D::getSizeY, "Returns the vertical size of grid map in cells count.")
            .def("getXMin", &COccupancyGridMap2D::getXMin, "Returns the \"x\" coordinate of left side of grid map.")
            .def("getXMax", &COccupancyGridMap2D::getXMax, "Returns the \"x\" coordinate of right side of grid map.")
            .def("getYMin", &COccupancyGridMap2D::getYMin, "Returns the \"y\" coordinate of top side of grid map.")
            .def("getYMax", &COccupancyGridMap2D::getYMax, "Returns the \"y\" coordinate of bottom side of grid map.")
            .def("getResolution", &COccupancyGridMap2D::getResolution, "Returns the resolution of the grid map.")
            .def("setCell", &COccupancyGridMap2D::setCell, "Change the contents [0,1] of a cell, given its index.")
            .def("getCell", &COccupancyGridMap2D::getCell, "Read the real valued [0,1] contents of a cell, given its index.")
            .def("laserScanSimulator", &COccupancyGridMap2D::laserScanSimulator, "Simulates a laser range scan into the current grid map.")
            .def("loadFromSimpleMap", &COccupancyGridMap2D::loadFromSimpleMap, "Load the map contents from a CSimpleMap object, erasing all previous content of the map.")
            .def_readwrite("insertionOptions", &COccupancyGridMap2D::insertionOptions)
            .def_readwrite("likelihoodOptions", &COccupancyGridMap2D::likelihoodOptions)
            .def("copy", &COccupancyGridMap2D_copy, return_value_policy<manage_new_object>())
#ifdef ROS_EXTENSIONS
            .def("to_ROS_OccupancyGrid_msg", &COccupancyGridMap2D_to_ROS_OccupancyGrid_msg1, "Convert to ROS OccupancyGrid Message")
            .def("to_ROS_OccupancyGrid_msg", &COccupancyGridMap2D_to_ROS_OccupancyGrid_msg2, "Convert to ROS OccupancyGrid Message")
            .def("from_ROS_OccupancyGrid_msg", &COccupancyGridMap2D_from_ROS_OccupancyGrid_msg, "Convert from ROS OccupancyGrid Message")
#endif
        ;

        // TInsertionOptions
        class_<COccupancyGridMap2D::TInsertionOptions, bases<CLoadableOptions> >("TInsertionOptions", init<>())
            .def_readwrite("mapAltitude", &COccupancyGridMap2D::TInsertionOptions::mapAltitude)
            .def_readwrite("useMapAltitude", &COccupancyGridMap2D::TInsertionOptions::useMapAltitude)
            .def_readwrite("maxDistanceInsertion", &COccupancyGridMap2D::TInsertionOptions::maxDistanceInsertion)
            .def_readwrite("maxOccupancyUpdateCertainty", &COccupancyGridMap2D::TInsertionOptions::maxOccupancyUpdateCertainty)
            .def_readwrite("considerInvalidRangesAsFreeSpace", &COccupancyGridMap2D::TInsertionOptions::considerInvalidRangesAsFreeSpace)
            .def_readwrite("decimation", &COccupancyGridMap2D::TInsertionOptions::decimation)
            .def_readwrite("horizontalTolerance", &COccupancyGridMap2D::TInsertionOptions::horizontalTolerance)
            .def_readwrite("CFD_features_gaussian_size", &COccupancyGridMap2D::TInsertionOptions::CFD_features_gaussian_size)
            .def_readwrite("CFD_features_median_size", &COccupancyGridMap2D::TInsertionOptions::CFD_features_median_size)
            .def_readwrite("wideningBeamsWithDistance", &COccupancyGridMap2D::TInsertionOptions::wideningBeamsWithDistance)
        ;

        // TLikelihoodOptions
        class_<COccupancyGridMap2D::TLikelihoodOptions, bases<CLoadableOptions> >("TLikelihoodOptions", init<>())
            .def_readwrite("likelihoodMethod", &COccupancyGridMap2D::TLikelihoodOptions::likelihoodMethod)
            .def_readwrite("LF_stdHit", &COccupancyGridMap2D::TLikelihoodOptions::LF_stdHit)
            .def_readwrite("LF_zHit", &COccupancyGridMap2D::TLikelihoodOptions::LF_zHit)
            .def_readwrite("LF_zRandom", &COccupancyGridMap2D::TLikelihoodOptions::LF_zRandom)
            .def_readwrite("LF_maxRange", &COccupancyGridMap2D::TLikelihoodOptions::LF_maxRange)
            .def_readwrite("LF_decimation", &COccupancyGridMap2D::TLikelihoodOptions::LF_decimation)
            .def_readwrite("LF_maxCorrsDistance", &COccupancyGridMap2D::TLikelihoodOptions::LF_maxCorrsDistance)
            .def_readwrite("LF_alternateAverageMethod", &COccupancyGridMap2D::TLikelihoodOptions::LF_alternateAverageMethod)
            .def_readwrite("MI_exponent", &COccupancyGridMap2D::TLikelihoodOptions::MI_exponent)
            .def_readwrite("MI_skip_rays", &COccupancyGridMap2D::TLikelihoodOptions::MI_skip_rays)
            .def_readwrite("MI_ratio_max_distance", &COccupancyGridMap2D::TLikelihoodOptions::MI_ratio_max_distance)
            .def_readwrite("rayTracing_useDistanceFilter", &COccupancyGridMap2D::TLikelihoodOptions::rayTracing_useDistanceFilter)
            .def_readwrite("rayTracing_decimation", &COccupancyGridMap2D::TLikelihoodOptions::rayTracing_decimation)
            .def_readwrite("rayTracing_stdHit", &COccupancyGridMap2D::TLikelihoodOptions::rayTracing_stdHit)
            .def_readwrite("consensus_takeEachRange", &COccupancyGridMap2D::TLikelihoodOptions::consensus_takeEachRange)
            .def_readwrite("consensus_pow", &COccupancyGridMap2D::TLikelihoodOptions::consensus_pow)
            .def_readwrite("OWA_weights", &COccupancyGridMap2D::TLikelihoodOptions::OWA_weights)
            .def_readwrite("enableLikelihoodCache", &COccupancyGridMap2D::TLikelihoodOptions::enableLikelihoodCache)
        ;
    }

    // CPointsMap
    {
        scope s = class_<CPointsMapWrap, boost::noncopyable>("CPointsMap", no_init)
            .def("reserve", &CPointsMapWrap::reserve, "Reserves memory for a given number of points: the size of the map does not change, it only reserves the memory.")
            .def("resize", &CPointsMapWrap::resize, "Resizes all point buffers so they can hold the given number of points: newly created points are set to default values, and old contents are not changed.")
            .def("setSize", &CPointsMapWrap::setSize, "Resizes all point buffers so they can hold the given number of points, *erasing* all previous contents and leaving all points to default values.")
            .def("setPointFast", &CPointsMapWrap::setPointFast, "Changes the coordinates of the given point (0-based index), *without* checking for out-of-bounds and *without* calling mark_as_modified().")
            .def("insertPointFast", &CPointsMapWrap::insertPointFast, "The virtual method for insertPoint() *without* calling mark_as_modified().")
            .def("copyFrom", &CPointsMapWrap::copyFrom, "Virtual assignment operator, copies as much common data (XYZ, color,...) as possible from the source map into this one.")
            .def("getPointAllFieldsFast", &CPointsMapWrap::getPointAllFieldsFast, "Get all the data fields for one point as a vector: depending on the implementation class this can be [X Y Z] or [X Y Z R G B], etc...")
            .def("setPointAllFieldsFast", &CPointsMapWrap::setPointAllFieldsFast, "Set all the data fields for one point as a vector: depending on the implementation class this can be [X Y Z] or [X Y Z R G B], etc...")
        ;
    }

    // CSimplePointsMap
    {
        scope s = class_<CSimplePointsMap, bases<CPointsMap> >("CSimplePointsMap", init<>())
            .def("loadFromRangeScan", &CSimplePointsMap_loadFromRangeScan1, "Transform the range scan into a set of cartessian coordinated points. The options in \"insertionOptions\" are considered in this method.")
            .def("loadFromRangeScan", &CSimplePointsMap_loadFromRangeScan2, "Transform the range scan into a set of cartessian coordinated points. The options in \"insertionOptions\" are considered in this method.")
            .def("getPointFast", &CSimplePointsMap_getPointFast, "Get all the data fields for one point: X Y Z")
            .def("getPointAllFieldsFast", &CSimplePointsMap_getPointAllFieldsFast, "Get all the data fields for one point as a tuple: (X Y Z)")
            .def("setPointFast", &CSimplePointsMap::setPointFast, "Set all the data fields for one point: X Y Z")
            .def("getSize", &CSimplePointsMap_getSize, "Get current size.")
        ;
    }

    // CSimpleMap
    {
        class_<CSimpleMap>("CSimpleMap", init<>())
            .def("saveToFile", &CSimpleMap::saveToFile, "Save this object to a .simplemap binary file (compressed with gzip)")
            .def("loadFromFile", &CSimpleMap::loadFromFile, "Load the contents of this object from a .simplemap binary file (possibly compressed with gzip).")
            .def("insert", &CSimpleMap_insert, "Add a new pair to the sequence. The objects are copied, so original ones can be free if desired after insertion.")
            .def_readwrite("size", &CSimpleMap::size)
        ;
    }

// FIXME
#ifdef MRPT_BELOW_1_3
// TMetricMapInitializer
    {
        scope s = class_<TMetricMapInitializerWrap>("TMetricMapInitializer", init<TRuntimeClassId*>())
            .def("set_COccupancyGridMap2D", &TMetricMapInitializer_set_COccupancyGridMap2D)
            .def_readwrite("occupancyGridMap2D_options", &TMetricMapInitializer::occupancyGridMap2D_options)
        ;

        // TOccGridMap2DOptions
        class_<TMetricMapInitializer::TOccGridMap2DOptions>("TOccGridMap2DOptions", init<>())
            .def_readwrite("min_x", &TMetricMapInitializer::TOccGridMap2DOptions::min_x)
            .def_readwrite("max_x", &TMetricMapInitializer::TOccGridMap2DOptions::max_x)
            .def_readwrite("min_y", &TMetricMapInitializer::TOccGridMap2DOptions::min_y)
            .def_readwrite("max_y", &TMetricMapInitializer::TOccGridMap2DOptions::max_y)
            .def_readwrite("resolution", &TMetricMapInitializer::TOccGridMap2DOptions::resolution)
            .def_readwrite("insertionOpts", &TMetricMapInitializer::TOccGridMap2DOptions::insertionOpts)
            .def_readwrite("likelihoodOpts", &TMetricMapInitializer::TOccGridMap2DOptions::likelihoodOpts)
        ;
    }

    // TSetOfMetricMapInitializers
    {
        class_<TSetOfMetricMapInitializers>("TSetOfMetricMapInitializers", init<>())
            .def("size", &TSetOfMetricMapInitializers::size)
            .def("push_back", &TSetOfMetricMapInitializers::push_back)
            .def("clear", &TSetOfMetricMapInitializers::clear)
            .def_readwrite("options", &TSetOfMetricMapInitializers::options)
        ;
    }

    // CMultiMetricMap
    {
        scope s = class_<CMultiMetricMap>("CMultiMetricMap", init<optional<TSetOfMetricMapInitializers*> >())
        ;
        // TOptions
        class_<CMultiMetricMap::TOptions, bases<CLoadableOptions> >("TOptions", init<>())
        ;
    }
#else
    // CMultiMetricMap
    {
        scope s = class_<CMultiMetricMap>("CMultiMetricMap", init<>())
            .def("getAs3DObject", &CMultiMetricMap_getAs3DObject, "Returns a 3D object representing the map.")
        ;
    }
#endif
    // CMultiMetricMapPDF
    {
        scope s = class_<CMultiMetricMapPDF>("CMultiMetricMapPDF", init<>())
            .def("getLastPose", &CMultiMetricMapPDF_getLastPose, "Return the last robot pose for the i'th particle.")
            .def("getPath", &CMultiMetricMapPDF_getPath, "Return the path (in absolute coordinate poses) for the i'th particle.")
            .def("getEstimatedPosePDFAtTime", &CMultiMetricMapPDF_getEstimatedPosePDFAtTime, "Returns the estimate of the robot pose as a particles PDF for the instant of time \"timeStep\", from 0 to N-1.")
            .def("getCurrentMostLikelyMetricMap", &CMultiMetricMapPDF::getCurrentMostLikelyMetricMap, return_value_policy<reference_existing_object>(), "Returns a pointer to the current most likely map (associated to the most likely particle).")
            .def_readwrite("options", &CMultiMetricMapPDF::options)
        ;

        // TConfigParams
        class_<CMultiMetricMapPDF::TPredictionParams, bases<CLoadableOptions> >("TPredictionParams", init<>())
            .def_readwrite("pfOptimalProposal_mapSelection", &CMultiMetricMapPDF::TPredictionParams::pfOptimalProposal_mapSelection)
            .def_readwrite("ICPGlobalAlign_MinQuality", &CMultiMetricMapPDF::TPredictionParams::ICPGlobalAlign_MinQuality)
            .def_readwrite("update_gridMapLikelihoodOptions", &CMultiMetricMapPDF::TPredictionParams::update_gridMapLikelihoodOptions)
            .def_readwrite("KLD_params", &CMultiMetricMapPDF::TPredictionParams::KLD_params)
            .def_readwrite("icp_params", &CMultiMetricMapPDF::TPredictionParams::icp_params)
        ;
    }

}

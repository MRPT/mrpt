/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/*  Plane-based Map (PbMap) library
 *  Construction of plane-based maps and localization in it from RGBD Images.
 *  Writen by Eduardo Fernandez-Moral. See docs for <a
 * href="group__mrpt__pbmap__grp.html" >mrpt-pbmap</a>
 */

#include "pbmap-precomp.h"  // precomp. hdr

#if MRPT_HAS_PCL

#include <mrpt/math/types_math.h>  // Eigen

//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/pbmap/PbMapMaker.h>

#include <mutex>
#include <iostream>

//#define _VERBOSE 0

using namespace std;
using namespace Eigen;
using namespace mrpt::pbmap;

std::mutex CS_visualize;

// Bhattacharyya histogram distance function
double BhattacharyyaDist(std::vector<float>& hist1, std::vector<float>& hist2)
{
	assert(hist1.size() == hist2.size());
	double BhattachDist;
	double BhattachDist_aux = 0.0;
	for (unsigned i = 0; i < hist1.size(); i++)
		BhattachDist_aux += sqrt(hist1[i] * hist2[i]);

	BhattachDist = sqrt(1 - BhattachDist_aux);

	return BhattachDist;
}

/*!Some parameters to specify input/output and some thresholds.*/
struct config_pbmap
{
	// [global]
	bool input_from_rawlog;
	std::string rawlog_path;
	bool record_rawlog;
	float color_threshold;
	float intensity_threshold;
	float hue_threshold;

	// [plane_segmentation]
	float dist_threshold;  // Maximum distance to the plane between neighbor
	// 3D-points
	float angle_threshold;  //  = 0.017453 * 4.0 // Maximum angle between
	//  contiguous 3D-points
	float minInliersRate;  // Minimum ratio of inliers/image points required

	// [map_construction]
	bool use_color;  // Add color information to the planes
	float proximity_neighbor_planes;  // Two planar patches are considered
	// neighbors when the closest distance
	// between them is under
	// proximity_neighbor_planes
	//  float max_angle_normals; // (10º) Two planar patches that represent the
	//  same surface must have similar normals // QUITAR
	float max_cos_normal;
	float max_dist_center_plane;  // Two planar patches that represent the same
	// surface must have their center in the same
	// plane
	float proximity_threshold;  // Two planar patches that represent the same
	// surface must overlap or be nearby
	int graph_mode;  // This var selects the condition to create edges in the
	// graph, either proximity of planar patches or
	// co-visibility in a single frame

	// [semantics]
	bool inferStructure;  // Infer if the planes correspond to the floor,
	// ceiling or walls
	bool makeClusters;  // Should the PbMapMaker cluster the planes according to
	// their co-visibility

	// [localisation]
	bool detect_loopClosure;  // Run PbMapLocaliser in a different threads to
	// detect loop closures or preloaded PbMaps
	string config_localiser;

	// [serialize]
	std::string path_save_pbmap;
	bool save_registered_cloud;
	std::string path_save_registered_cloud;

} configPbMap;

void readConfigFile(const string& config_file_name)
{
	mrpt::config::CConfigFile config_file(config_file_name);

	// Plane segmentation
	configPbMap.dist_threshold = config_file.read_float(
		"plane_segmentation", "dist_threshold", 0.04, true);
	configPbMap.angle_threshold = config_file.read_float(
		"plane_segmentation", "angle_threshold", 0.069812, true);
	configPbMap.minInliersRate = config_file.read_float(
		"plane_segmentation", "minInliersRate", 0.01, true);

	// map_construction
	configPbMap.use_color =
		config_file.read_bool("map_construction", "use_color", false);
	configPbMap.color_threshold =
		config_file.read_float("map_construction", "color_threshold", 0.09);
	configPbMap.intensity_threshold = config_file.read_float(
		"map_construction", "intensity_threshold", 255.0);
	configPbMap.hue_threshold =
		config_file.read_float("unary", "hue_threshold", 0.25);
	configPbMap.proximity_neighbor_planes = config_file.read_float(
		"map_construction", "proximity_neighbor_planes", 1.0);
	configPbMap.graph_mode =
		config_file.read_int("map_construction", "graph_mode", 1);
	configPbMap.max_cos_normal = config_file.read_float(
		"map_construction", "max_cos_normal", 0.9848, true);
	configPbMap.max_dist_center_plane = config_file.read_float(
		"map_construction", "max_dist_center_plane", 0.1, true);
	configPbMap.proximity_threshold =
		config_file.read_float("map_construction", "proximity_threshold", 0.15);

	// [semantics]
	configPbMap.inferStructure =
		config_file.read_bool("semantics", "inferStructure", true);
	configPbMap.makeClusters =
		config_file.read_bool("semantics", "makeCovisibilityClusters", false);
	//  configPbMap.path_prev_pbmap =
	//  config_file.read_string("localisation","path_prev_pbmap","",true);

	// [localisation]
	configPbMap.detect_loopClosure =
		config_file.read_bool("localisation", "detect_loopClosure", true);
	if (configPbMap.detect_loopClosure)
		configPbMap.config_localiser = config_file.read_string(
			"localisation", "config_localiser", "", true);

	// serialize
	configPbMap.path_save_pbmap =
		config_file.read_string("serialize", "path_save_pbmap", "map");
	configPbMap.save_registered_cloud =
		config_file.read_bool("serialize", "save_registered_cloud", true);
	configPbMap.path_save_registered_cloud = config_file.read_string(
		"serialize", "path_save_registered_cloud",
		"/home/edu/Projects/PbMaps/PbMaps.txt");
	// cout << "path_save_registered_cloud " <<
	// configPbMap.path_save_registered_cloud << endl;

#ifdef _VERBOSE
	cout << "readConfigFile configPbMap.ini dist_threshold "
		 << configPbMap.dist_threshold << endl;
#endif
}

PbMapMaker::PbMapMaker(const string& config_file)
	: cloudViewer("Cloud Viewer"),
	  mpPbMapLocaliser(nullptr),
	  m_pbmaker_must_stop(false),
	  m_pbmaker_finished(false)
{
	// Load parameters
	ASSERT_FILE_EXISTS_(config_file);
	readConfigFile(config_file);

	mpPlaneInferInfo = new PlaneInferredInfo(mPbMap);

	if (configPbMap.detect_loopClosure)
		mpPbMapLocaliser =
			new PbMapLocaliser(mPbMap, configPbMap.config_localiser);

	if (configPbMap.makeClusters) clusterize = new SemanticClustering(mPbMap);

	pbmaker_hd = std::thread(&PbMapMaker::run, this);
}

bool PbMapMaker::arePlanesNearby(
	Plane& plane1, Plane& plane2, const float distThreshold)
{
	float distThres2 = distThreshold * distThreshold;

	// First we check distances between centroids and vertex to accelerate this
	// check
	if ((plane1.v3center - plane2.v3center).squaredNorm() < distThres2)
		return true;

	for (unsigned i = 1; i < plane1.polygonContourPtr->size(); i++)
		if ((getVector3fromPointXYZ(plane1.polygonContourPtr->points[i]) -
			 plane2.v3center)
				.squaredNorm() < distThres2)
			return true;

	for (unsigned j = 1; j < plane2.polygonContourPtr->size(); j++)
		if ((plane1.v3center -
			 getVector3fromPointXYZ(plane2.polygonContourPtr->points[j]))
				.squaredNorm() < distThres2)
			return true;

	for (unsigned i = 1; i < plane1.polygonContourPtr->size(); i++)
		for (unsigned j = 1; j < plane2.polygonContourPtr->size(); j++)
			if ((diffPoints(
					 plane1.polygonContourPtr->points[i],
					 plane2.polygonContourPtr->points[j]))
					.squaredNorm() < distThres2)
				return true;

	// If not found yet, search properly by checking distances:
	// a) Between an edge and a vertex
	// b) Between two edges (imagine two polygons on perpendicular planes)
	// c) Between a vertex and the inside of the polygon
	// d) Or the polygons intersect

	// a) & b)
	for (unsigned i = 1; i < plane1.polygonContourPtr->size(); i++)
		for (unsigned j = 1; j < plane2.polygonContourPtr->size(); j++)
			if (dist3D_Segment_to_Segment2(
					Segment(
						plane1.polygonContourPtr->points[i],
						plane1.polygonContourPtr->points[i - 1]),
					Segment(
						plane2.polygonContourPtr->points[j],
						plane2.polygonContourPtr->points[j - 1])) < distThres2)
				return true;

	// c)
	for (unsigned i = 1; i < plane1.polygonContourPtr->size(); i++)
		if (plane2.v3normal.dot(
				getVector3fromPointXYZ(plane1.polygonContourPtr->points[i]) -
				plane2.v3center) < distThreshold)
			if (isInHull(
					plane1.polygonContourPtr->points[i],
					plane2.polygonContourPtr))
				return true;

	for (unsigned j = 1; j < plane2.polygonContourPtr->size(); j++)
		if (plane1.v3normal.dot(
				getVector3fromPointXYZ(plane2.polygonContourPtr->points[j]) -
				plane1.v3center) < distThreshold)
			if (isInHull(
					plane2.polygonContourPtr->points[j],
					plane1.polygonContourPtr))
				return true;

	return false;
}

void PbMapMaker::checkProximity(Plane& plane, float proximity)
{
	for (unsigned i = 0; i < mPbMap.vPlanes.size(); i++)
	{
		if (plane.id == mPbMap.vPlanes[i].id) continue;

		if (plane.nearbyPlanes.count(mPbMap.vPlanes[i].id)) continue;

		if (arePlanesNearby(
				plane, mPbMap.vPlanes[i], proximity))  // If the planes are
		// closer than proximity
		// (in meters), then mark
		// them as neighbors
		{
			plane.nearbyPlanes.insert(mPbMap.vPlanes[i].id);
			mPbMap.vPlanes[i].nearbyPlanes.insert(plane.id);
		}
	}
}

void PbMapMaker::detectPlanesCloud(
	pcl::PointCloud<PointT>::Ptr& pointCloudPtr_arg, Eigen::Matrix4f& poseKF,
	double distThreshold, double angleThreshold, double minInliersF)
{
	boost::mutex::scoped_lock updateLock(mtx_pbmap_busy);

	unsigned minInliers = minInliersF * pointCloudPtr_arg->size();

#ifdef _VERBOSE
	cout << "detectPlanes in a cloud with " << pointCloudPtr_arg->size()
		 << " points " << minInliers << " minInliers\n";
#endif

	pcl::PointCloud<PointT>::Ptr pointCloudPtr_arg2(
		new pcl::PointCloud<PointT>);
	pcl::copyPointCloud(*pointCloudPtr_arg, *pointCloudPtr_arg2);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr alignedCloudPtr(
		new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::transformPointCloud(*pointCloudPtr_arg, *alignedCloudPtr, poseKF);

	{
		std::lock_guard<std::mutex> csl(CS_visualize);
		*mPbMap.globalMapPtr += *alignedCloudPtr;
		// Downsample voxel map's point cloud
		static pcl::VoxelGrid<pcl::PointXYZRGBA> grid;
		grid.setLeafSize(0.02, 0.02, 0.02);
		pcl::PointCloud<pcl::PointXYZRGBA> globalMap;
		grid.setInputCloud(mPbMap.globalMapPtr);
		grid.filter(globalMap);
		mPbMap.globalMapPtr->clear();
		*mPbMap.globalMapPtr = globalMap;
	}  // End CS

	pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
	ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
	ne.setMaxDepthChangeFactor(0.02f);  // For VGA: 0.02f, 10.0f
	ne.setNormalSmoothingSize(10.0f);
	ne.setDepthDependentSmoothing(true);

	pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
	mps.setMinInliers(minInliers);
	cout << "Params " << minInliers << " " << angleThreshold << " "
		 << distThreshold << endl;
	mps.setAngularThreshold(angleThreshold);  // (0.017453 * 2.0) // 3 degrees
	mps.setDistanceThreshold(distThreshold);  // 2cm

	pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(
		new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud(pointCloudPtr_arg2);
	ne.compute(*normal_cloud);

#ifdef _VERBOSE
	double plane_extract_start = pcl::getTime();
#endif
	mps.setInputNormals(normal_cloud);
	mps.setInputCloud(pointCloudPtr_arg2);

	std::vector<
		pcl::PlanarRegion<PointT>, aligned_allocator<pcl::PlanarRegion<PointT>>>
		regions;
	std::vector<pcl::ModelCoefficients> model_coefficients;
	std::vector<pcl::PointIndices> inlier_indices;
	pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
	std::vector<pcl::PointIndices> label_indices;
	std::vector<pcl::PointIndices> boundary_indices;
	mps.segmentAndRefine(
		regions, model_coefficients, inlier_indices, labels, label_indices,
		boundary_indices);

#ifdef _VERBOSE
	double plane_extract_end = pcl::getTime();
	std::cout << "Plane extraction took "
			  << double(plane_extract_end - plane_extract_start) << std::endl;
	//    std::cout << "Frame took " << double (plane_extract_end -
	//    normal_start) << std::endl;
	cout << regions.size() << " planes detected\n";
#endif

	// Create a vector with the planes detected in this keyframe, and calculate
	// their parameters (normal, center, pointclouds, etc.)
	// in the global reference
	vector<Plane> detectedPlanes;
	for (size_t i = 0; i < regions.size(); i++)
	{
		Plane plane;

		Vector3f centroid = regions[i].getCentroid();
		plane.v3center = compose(poseKF, centroid);
		plane.v3normal =
			poseKF.block(0, 0, 3, 3) * Vector3f(
										   model_coefficients[i].values[0],
										   model_coefficients[i].values[1],
										   model_coefficients[i].values[2]);
		//    plane.curvature = regions[i].getCurvature();
		//  assert(plane.v3normal*plane.v3center.transpose() <= 0);
		//    if(plane.v3normal*plane.v3center.transpose() <= 0)
		//      plane.v3normal *= -1;

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
		extract.setInputCloud(pointCloudPtr_arg2);
		extract.setIndices(
			boost::make_shared<const pcl::PointIndices>(inlier_indices[i]));
		extract.setNegative(false);
		extract.filter(
			*plane.planePointCloudPtr);  // Write the planar point cloud

		static pcl::VoxelGrid<pcl::PointXYZRGBA> plane_grid;
		plane_grid.setLeafSize(0.05, 0.05, 0.05);
		pcl::PointCloud<pcl::PointXYZRGBA> planeCloud;
		plane_grid.setInputCloud(plane.planePointCloudPtr);
		plane_grid.filter(planeCloud);
		plane.planePointCloudPtr->clear();
		pcl::transformPointCloud(planeCloud, *plane.planePointCloudPtr, poseKF);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr contourPtr(
			new pcl::PointCloud<pcl::PointXYZRGBA>);
		contourPtr->points = regions[i].getContour();
		plane_grid.setLeafSize(0.1, 0.1, 0.1);
		plane_grid.setInputCloud(contourPtr);
		plane_grid.filter(*plane.polygonContourPtr);
		//    plane.contourPtr->points = regions[i].getContour();
		//    pcl::transformPointCloud(*plane.contourPtr,*plane.polygonContourPtr,poseKF);
		pcl::transformPointCloud(*plane.polygonContourPtr, *contourPtr, poseKF);
		plane.calcConvexHull(contourPtr);
		plane.computeMassCenterAndArea();
		plane.areaVoxels = plane.planePointCloudPtr->size() * 0.0025;

#ifdef _VERBOSE
		cout << "Area plane region " << plane.areaVoxels << " of Chull "
			 << plane.areaHull << " of polygon "
			 << plane.compute2DPolygonalArea() << endl;
#endif

		// Check whether this region correspond to the same plane as a previous
		// one (this situation may happen when there exists a small
		// discontinuity in the observation)
		bool isSamePlane = false;
		for (size_t j = 0; j < detectedPlanes.size(); j++)
			if (areSamePlane(
					detectedPlanes[j], plane, configPbMap.max_cos_normal,
					configPbMap.max_dist_center_plane,
					configPbMap.proximity_threshold))  // The planes are merged
			// if they are the same
			{
				isSamePlane = true;

				mergePlanes(detectedPlanes[j], plane);

#ifdef _VERBOSE
				cout << "\tTwo regions support the same plane in the same "
						"KeyFrame\n";
#endif

				break;
			}
		if (!isSamePlane) detectedPlanes.push_back(plane);
	}

#ifdef _VERBOSE
	cout << detectedPlanes.size() << " Planes detected\n";
#endif

	// Merge detected planes with previous ones if they are the same
	size_t numPrevPlanes = mPbMap.vPlanes.size();
	//  set<unsigned> observedPlanes;
	observedPlanes.clear();
	{
		std::lock_guard<std::mutex> csl(CS_visualize);
		for (size_t i = 0; i < detectedPlanes.size(); i++)
		{
			// Check similarity with previous planes detected
			bool isSamePlane = false;
			auto itPlane = mPbMap.vPlanes.begin();
			//  if(frameQueue.size() != 12)
			for (size_t j = 0; j < numPrevPlanes;
				 j++, itPlane++)  // numPrevPlanes
			{
				// The planes are merged if they are the same
				if (areSamePlane(
						mPbMap.vPlanes[j], detectedPlanes[i],
						configPbMap.max_cos_normal,
						configPbMap.max_dist_center_plane,
						configPbMap.proximity_threshold))
				{
					isSamePlane = true;
					mergePlanes(mPbMap.vPlanes[j], detectedPlanes[i]);

					// Update proximity graph
					checkProximity(
						mPbMap.vPlanes[j],
						configPbMap
							.proximity_neighbor_planes);  // Detect neighbors

#ifdef _VERBOSE
					cout << "Previous plane " << mPbMap.vPlanes[j].id
						 << " area " << mPbMap.vPlanes[j].areaVoxels
						 << " of polygon "
						 << mPbMap.vPlanes[j].compute2DPolygonalArea() << endl;
#endif

					if (observedPlanes.count(mPbMap.vPlanes[j].id) ==
						0)  // If this plane has already been observed through a
					// previous partial plane in this same keyframe,
					// then we must not account twice in the observation
					// count
					{
						mPbMap.vPlanes[j].numObservations++;

						// Update co-visibility graph
						for (auto it = observedPlanes.begin();
							 it != observedPlanes.end(); it++)
							if (mPbMap.vPlanes[j].neighborPlanes.count(*it))
							{
								mPbMap.vPlanes[j].neighborPlanes[*it]++;
								mPbMap.vPlanes[*it].neighborPlanes
									[mPbMap.vPlanes[j]
										 .id]++;  // j = mPbMap.vPlanes[j]
							}
							else
							{
								mPbMap.vPlanes[j].neighborPlanes[*it] = 1;
								mPbMap.vPlanes[*it]
									.neighborPlanes[mPbMap.vPlanes[j].id] = 1;
							}

						observedPlanes.insert(mPbMap.vPlanes[j].id);
					}

#ifdef _VERBOSE
					cout << "Same plane\n";
#endif

					itPlane++;
					for (size_t k = j + 1; k < numPrevPlanes;
						 k++, itPlane++)  // numPrevPlanes
						if (areSamePlane(
								mPbMap.vPlanes[j], mPbMap.vPlanes[k],
								configPbMap.max_cos_normal,
								configPbMap.max_dist_center_plane,
								configPbMap.proximity_threshold))
						{
							// The planes are merged if they are the same
							mergePlanes(mPbMap.vPlanes[j], mPbMap.vPlanes[k]);

							mPbMap.vPlanes[j].numObservations +=
								mPbMap.vPlanes[k].numObservations;

							for (auto it =
									 mPbMap.vPlanes[k].nearbyPlanes.begin();
								 it != mPbMap.vPlanes[k].nearbyPlanes.end();
								 it++)
								mPbMap.vPlanes[*it].nearbyPlanes.erase(
									mPbMap.vPlanes[k].id);

							for (auto it =
									 mPbMap.vPlanes[k].neighborPlanes.begin();
								 it != mPbMap.vPlanes[k].neighborPlanes.end();
								 it++)
								mPbMap.vPlanes[it->first].neighborPlanes.erase(
									mPbMap.vPlanes[k].id);

							// Update plane index
							for (size_t h = k + 1; h < numPrevPlanes; h++)
								--mPbMap.vPlanes[h].id;

							for (size_t h = 0; h < numPrevPlanes; h++)
							{
								if (k == h) continue;

								for (auto it =
										 mPbMap.vPlanes[h].nearbyPlanes.begin();
									 it != mPbMap.vPlanes[h].nearbyPlanes.end();
									 it++)
									if (*it > mPbMap.vPlanes[k].id)
									{
										mPbMap.vPlanes[h].nearbyPlanes.insert(
											*it - 1);
										mPbMap.vPlanes[h].nearbyPlanes.erase(
											*it);
									}

								for (auto it = mPbMap.vPlanes[h]
												   .neighborPlanes.begin();
									 it !=
									 mPbMap.vPlanes[h].neighborPlanes.end();
									 it++)
									if (it->first > mPbMap.vPlanes[k].id)
									{
										mPbMap.vPlanes[h]
											.neighborPlanes[it->first - 1] =
											it->second;
										mPbMap.vPlanes[h].neighborPlanes.erase(
											it);
									}
							}

							mPbMap.vPlanes.erase(itPlane);
							--numPrevPlanes;

#ifdef _VERBOSE
							cout << "MERGE TWO PREVIOUS PLANES WHEREBY THE "
									"INCORPORATION OF A NEW REGION \n";
#endif
						}

					break;
				}
			}
			if (!isSamePlane)
			{
				detectedPlanes[i].id = mPbMap.vPlanes.size();
				detectedPlanes[i].numObservations = 1;
				detectedPlanes[i].bFullExtent = false;
				detectedPlanes[i].nFramesAreaIsStable = 0;
				//      detectedPlanes[i].calcMainColor(calcMainColor();
				if (configPbMap.makeClusters)
				{
					detectedPlanes[i].semanticGroup =
						clusterize->currentSemanticGroup;
					clusterize->groups[clusterize->currentSemanticGroup]
						.push_back(detectedPlanes[i].id);
				}

#ifdef _VERBOSE
				cout << "New plane " << detectedPlanes[i].id << " area "
					 << detectedPlanes[i].areaVoxels << " of polygon "
					 << detectedPlanes[i].areaHull << endl;
#endif

				// Update proximity graph
				checkProximity(
					detectedPlanes[i],
					configPbMap.proximity_neighbor_planes);  // Detect neighbors
				// with max
				// separation of
				// 1.0 meters

				// Update co-visibility graph
				for (auto it = observedPlanes.begin();
					 it != observedPlanes.end(); it++)
				{
					detectedPlanes[i].neighborPlanes[*it] = 1;
					mPbMap.vPlanes[*it].neighborPlanes[detectedPlanes[i].id] =
						1;
				}

				observedPlanes.insert(detectedPlanes[i].id);

				mPbMap.vPlanes.push_back(detectedPlanes[i]);
			}
		}
	}

#ifdef _VERBOSE
	cout << "\n\tobservedPlanes: ";
	cout << observedPlanes.size() << " Planes observed\n";
	for (set<unsigned>::iterator it = observedPlanes.begin();
		 it != observedPlanes.end(); it++)
		cout << *it << " ";
	cout << endl;
#endif

	// For all observed planes
	for (auto it = observedPlanes.begin(); it != observedPlanes.end(); it++)
	{
		Plane& observedPlane = mPbMap.vPlanes[*it];

		// Calculate principal direction
		observedPlane.calcElongationAndPpalDir();

		////cout << "Update color\n";
		// Update color
		observedPlane.calcMainColor();

#ifdef _VERBOSE
		cout << "Plane " << observedPlane.id << " color\n"
			 << observedPlane.v3colorNrgb << endl;
#endif

		// Infer knowledge from the planes (e.g. do these planes represent the
		// floor, walls, etc.)
		if (configPbMap.inferStructure)
			mpPlaneInferInfo->searchTheFloor(poseKF, observedPlane);
	}  // End for obsevedPlanes

	// Search the floor plane
	if (mPbMap.FloorPlane !=
		-1)  // Verify that the observed planes centers are above the floor
	{
#ifdef _VERBOSE
		cout << "Verify that the observed planes centers are above the floor\n";
#endif

		for (auto it = observedPlanes.rbegin(); it != observedPlanes.rend();
			 it++)
		{
			if (static_cast<int>(*it) == mPbMap.FloorPlane) continue;
			if (mPbMap.vPlanes[mPbMap.FloorPlane].v3normal.dot(
					mPbMap.vPlanes[*it].v3center -
					mPbMap.vPlanes[mPbMap.FloorPlane].v3center) < -0.1)
			{
				if (mPbMap.vPlanes[mPbMap.FloorPlane].v3normal.dot(
						mPbMap.vPlanes[*it].v3normal) >
					0.99)  //(cos 8.1º = 0.99)
				{
					mPbMap.vPlanes[*it].label = "Floor";
					mPbMap.vPlanes[mPbMap.FloorPlane].label = "";
					mPbMap.FloorPlane = *it;
				}
				else
				{
					//            assert(false);
					mPbMap.vPlanes[mPbMap.FloorPlane].label = "";
					mPbMap.FloorPlane = -1;
					break;
				}
			}
		}
	}

	if (configPbMap.detect_loopClosure)
		for (auto it = observedPlanes.begin(); it != observedPlanes.end(); it++)
		{
			//    cout << "insert planes\n";
			if (mpPbMapLocaliser->vQueueObservedPlanes.size() < 10)
				mpPbMapLocaliser->vQueueObservedPlanes.push_back(*it);
		}

#ifdef _VERBOSE
	cout << "DetectedPlanesCloud finished\n";
#endif

	updateLock.unlock();
}

bool graphRepresentation = false;
void keyboardEventOccurred(
	const pcl::visualization::KeyboardEvent& event, void* viewer_void)
{
	if ((event.getKeySym() == "r" || event.getKeySym() == "R") &&
		event.keyDown())
	{
		graphRepresentation = !graphRepresentation;
	}
}

/*!Check if the the input plane is the same than this plane for some given angle
 * and distance thresholds.
 * If the planes are the same they are merged in this and the function returns
 * true. Otherwise it returns false.*/
bool PbMapMaker::areSamePlane(
	Plane& plane1, Plane& plane2, const float& cosAngleThreshold,
	const float& distThreshold, const float& proxThreshold)
{
	// Check that both planes have similar orientation
	if (plane1.v3normal.dot(plane2.v3normal) < cosAngleThreshold) return false;

	// Check the normal distance of the planes centers using their average
	// normal
	float dist_normal = plane1.v3normal.dot(plane2.v3center - plane1.v3center);
	//  if(fabs(dist_normal) > distThreshold ) // Avoid matching different
	//  parallel planes
	//    return false;
	float thres_max_dist =
		max(distThreshold,
			distThreshold * 2 * (plane2.v3center - plane1.v3center).norm());
	if (fabs(dist_normal) >
		thres_max_dist)  // Avoid matching different parallel planes
		return false;

	// Once we know that the planes are almost coincident (parallelism and
	// position)
	// we check that the distance between the planes is not too big
	return arePlanesNearby(plane1, plane2, proxThreshold);
}

void PbMapMaker::mergePlanes(Plane& updatePlane, Plane& discardPlane)
{
	// Update normal and center
	updatePlane.v3normal = updatePlane.areaVoxels * updatePlane.v3normal +
						   discardPlane.areaVoxels * discardPlane.v3normal;
	updatePlane.v3normal = updatePlane.v3normal / (updatePlane.v3normal).norm();
	// Update point inliers
	//  *updatePlane.polygonContourPtr += *discardPlane.polygonContourPtr; //
	//  Merge polygon points
	*updatePlane.planePointCloudPtr +=
		*discardPlane.planePointCloudPtr;  // Add the points of the new
	// detection and perform a voxel grid

	// Filter the points of the patch with a voxel-grid. This points are used
	// only for visualization
	static pcl::VoxelGrid<pcl::PointXYZRGBA> merge_grid;
	merge_grid.setLeafSize(0.05, 0.05, 0.05);
	pcl::PointCloud<pcl::PointXYZRGBA> mergeCloud;
	merge_grid.setInputCloud(updatePlane.planePointCloudPtr);
	merge_grid.filter(mergeCloud);
	updatePlane.planePointCloudPtr->clear();
	*updatePlane.planePointCloudPtr = mergeCloud;

	//  if(configPbMap.use_color)
	//    updatePlane.calcMainColor();

	*discardPlane.polygonContourPtr += *updatePlane.polygonContourPtr;
	updatePlane.calcConvexHull(discardPlane.polygonContourPtr);
	updatePlane.computeMassCenterAndArea();

	// Move the points to fulfill the plane equation
	updatePlane.forcePtsLayOnPlane();

	// Update area
	double area_recalc = updatePlane.planePointCloudPtr->size() * 0.0025;
	mpPlaneInferInfo->isFullExtent(updatePlane, area_recalc);
	updatePlane.areaVoxels = updatePlane.planePointCloudPtr->size() * 0.0025;
}

// Color = (red[i], grn[i], blu[i])
// The color order is: red, green, blue, yellow, pink, turquoise, orange,
// purple, dark green, beige
unsigned char red[10] = {255, 0, 0, 255, 255, 0, 255, 204, 0, 255};
unsigned char grn[10] = {0, 255, 0, 255, 0, 255, 160, 51, 128, 222};
unsigned char blu[10] = {0, 0, 255, 0, 255, 255, 0, 204, 0, 173};

double ared[10] = {1.0, 0, 0, 1.0, 1.0, 0, 1.0, 0.8, 0, 1.0};
double agrn[10] = {0, 1.0, 0, 1.0, 0, 1.0, 0.6, 0.2, 0.5, 0.9};
double ablu[10] = {0, 0, 1.0, 0, 1.0, 1.0, 0, 0.8, 0, 0.7};

void PbMapMaker::viz_cb(pcl::visualization::PCLVisualizer& viz)
{
	if (mPbMap.globalMapPtr->empty())
	{
		std::this_thread::sleep_for(10ms);
		return;
	}

	{
		std::lock_guard<std::mutex> csl(CS_visualize);

		// Render the data
		{
			viz.removeAllShapes();
			viz.removeAllPointClouds();

			char name[1024];

			//      if(graphRepresentation)
			//      {
			//        for(size_t i=0; i<mPbMap.vPlanes.size(); i++)
			//        {
			//          pcl::PointXYZ center(2*mPbMap.vPlanes[i].v3center[0],
			//          2*mPbMap.vPlanes[i].v3center[1],
			//          2*mPbMap.vPlanes[i].v3center[2]);
			//          double radius = 0.1 *
			//          sqrt(mPbMap.vPlanes[i].areaVoxels);
			//          sprintf (name, "sphere%u", static_cast<unsigned>(i));
			//          viz.addSphere (center, radius, ared[i%10], agrn[i%10],
			//          ablu[i%10], name);
			//
			//          if( !mPbMap.vPlanes[i].label.empty() )
			//              viz.addText3D (mPbMap.vPlanes[i].label, center, 0.1,
			//              ared[i%10], agrn[i%10], ablu[i%10],
			//              mPbMap.vPlanes[i].label);
			//          else
			//          {
			//            sprintf (name, "P%u", static_cast<unsigned>(i));
			//            viz.addText3D (name, center, 0.1, ared[i%10],
			//            agrn[i%10], ablu[i%10], name);
			//          }
			//
			//          // Draw edges
			//          if(!configPbMap.graph_mode) // Nearby neighbors
			//            for(set<unsigned>::iterator it =
			//            mPbMap.vPlanes[i].nearbyPlanes.begin(); it !=
			//            mPbMap.vPlanes[i].nearbyPlanes.end(); it++)
			//            {
			//              if(*it > mPbMap.vPlanes[i].id)
			//                break;
			//
			//              sprintf (name, "commonObs%u_%u",
			//              static_cast<unsigned>(i),
			//              static_cast<unsigned>(*it));
			//              pcl::PointXYZ
			//              center_it(2*mPbMap.vPlanes[*it].v3center[0],
			//              2*mPbMap.vPlanes[*it].v3center[1],
			//              2*mPbMap.vPlanes[*it].v3center[2]);
			//              viz.addLine (center, center_it, ared[i%10],
			//              agrn[i%10], ablu[i%10], name);
			//            }
			//          else
			//            for(map<unsigned,unsigned>::iterator it =
			//            mPbMap.vPlanes[i].neighborPlanes.begin(); it !=
			//            mPbMap.vPlanes[i].neighborPlanes.end(); it++)
			//            {
			//              if(it->first > mPbMap.vPlanes[i].id)
			//                break;
			//
			//              sprintf (name, "commonObs%u_%u",
			//              static_cast<unsigned>(i),
			//              static_cast<unsigned>(it->first));
			//              pcl::PointXYZ
			//              center_it(2*mPbMap.vPlanes[it->first].v3center[0],
			//              2*mPbMap.vPlanes[it->first].v3center[1],
			//              2*mPbMap.vPlanes[it->first].v3center[2]);
			//              viz.addLine (center, center_it, ared[i%10],
			//              agrn[i%10], ablu[i%10], name);
			//
			//              sprintf (name, "edge%u_%u",
			//              static_cast<unsigned>(i),
			//              static_cast<unsigned>(it->first));
			//              char commonObs[8];
			//              sprintf (commonObs, "%u", it->second);
			//              pcl::PointXYZ half_edge( (center_it.x+center.x)/2,
			//              (center_it.y+center.y)/2, (center_it.z+center.z)/2
			//              );
			//              viz.addText3D (commonObs, half_edge, 0.05, 1.0, 1.0,
			//              1.0, name);
			//            }
			//
			//        }
			//      }
			//      else
			{  // Regular representation

				if (graphRepresentation)
				{
					if (!viz.updatePointCloud(mPbMap.globalMapPtr, "cloud"))
						viz.addPointCloud(mPbMap.globalMapPtr, "cloud");
					return;
				}

				if (mpPbMapLocaliser != nullptr)
					if (mpPbMapLocaliser->alignedModelPtr)
					{
						if (!viz.updatePointCloud(
								mpPbMapLocaliser->alignedModelPtr, "model"))
							viz.addPointCloud(
								mpPbMapLocaliser->alignedModelPtr, "model");
					}

				sprintf(
					name, "PointCloud size %u",
					static_cast<unsigned>(mPbMap.globalMapPtr->size()));
				viz.addText(name, 10, 20);

				// pcl::ModelCoefficients plane_coefs;
				// plane_coefs.values[0] = mPbMap.vPlanes[0].v3normal[0];
				// plane_coefs.values[1] = mPbMap.vPlanes[0].v3normal[1];
				// plane_coefs.values[2] = mPbMap.vPlanes[0].v3normal[2];
				// plane_coefs.values[3] = -(mPbMap.vPlanes[0].v3normal .dot
				// (mPbMap.vPlanes[0].v3center) );
				// viz.addPlane (plane_coefs);

				for (size_t i = 0; i < mPbMap.vPlanes.size(); i++)
				{
					Plane& plane_i = mPbMap.vPlanes[i];
					sprintf(name, "normal_%u", static_cast<unsigned>(i));
					pcl::PointXYZ pt1, pt2;  // Begin and end points of normal's
					// arrow for visualization
					pt1 = pcl::PointXYZ(
						plane_i.v3center[0], plane_i.v3center[1],
						plane_i.v3center[2]);
					pt2 = pcl::PointXYZ(
						plane_i.v3center[0] + (0.5f * plane_i.v3normal[0]),
						plane_i.v3center[1] + (0.5f * plane_i.v3normal[1]),
						plane_i.v3center[2] + (0.5f * plane_i.v3normal[2]));
					viz.addArrow(
						pt2, pt1, ared[i % 10], agrn[i % 10], ablu[i % 10],
						false, name);

					// Draw Ppal diretion
					//          if( plane_i.elongation > 1.3 )
					//          {
					//            sprintf (name, "ppalComp_%u",
					//            static_cast<unsigned>(i));
					//            pcl::PointXYZ pt3 = pcl::PointXYZ (
					//            plane_i.v3center[0] + (0.2f *
					//            plane_i.v3PpalDir[0]),
					//                                                plane_i.v3center[1]
					//                                                + (0.2f *
					//                                                plane_i.v3PpalDir[1]),
					//                                                plane_i.v3center[2]
					//                                                + (0.2f *
					//                                                plane_i.v3PpalDir[2]));
					//            viz.addArrow (pt3, plane_i.pt1, ared[i%10],
					//            agrn[i%10], ablu[i%10], false, name);
					//          }

					//          if( !plane_i.label.empty() )
					//            viz.addText3D (plane_i.label, pt2, 0.1,
					//            ared[i%10], agrn[i%10], ablu[i%10],
					//            plane_i.label);
					//          else
					{
						sprintf(name, "n%u", static_cast<unsigned>(i));
						//            sprintf (name, "n%u_%u",
						//            static_cast<unsigned>(i),
						//            static_cast<unsigned>(plane_i.semanticGroup));
						viz.addText3D(
							name, pt2, 0.1, ared[i % 10], agrn[i % 10],
							ablu[i % 10], name);
					}

					//          sprintf (name, "planeRaw_%02u",
					//          static_cast<unsigned>(i));
					//          viz.addPointCloud
					//          (plane_i.planeRawPointCloudPtr, name);//
					//          contourPtr, planePointCloudPtr,
					//          polygonContourPtr

					//          if(!configPbMap.makeClusters)
					//          {
					sprintf(name, "plane_%02u", static_cast<unsigned>(i));
					//          if(plane_i.bDominantColor)
					{
						//          pcl::visualization::PointCloudColorHandlerCustom
						//          <PointT> color (plane_i.planePointCloudPtr,
						//          red[i%10], grn[i%10], blu[i%10]);
						////
						/// pcl::visualization::PointCloudColorHandlerCustom
						///< PointT> color (plane_i.planePointCloudPtr,
						/// red[plane_i.semanticGroup%10],
						/// grn[plane_i.semanticGroup%10],
						/// blu[plane_i.semanticGroup%10]);
						//          viz.addPointCloud
						//          (plane_i.planePointCloudPtr, color, name);
						//          viz.setPointCloudRenderingProperties
						//          (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
						//          2, name);
						//          }
						//          else
						//          {
						//            sprintf (name, "plane_%02u",
						//            static_cast<unsigned>(i));
						//            pcl::visualization::PointCloudColorHandlerCustom
						//            <PointT> color
						//            (plane_i.planePointCloudPtr,
						//            red[plane_i.semanticGroup%10],
						//            grn[plane_i.semanticGroup%10],
						//            blu[plane_i.semanticGroup%10]);

						double illum = 0;
						if (fabs(plane_i.v3colorNrgb[0] - 0.33) < 0.03 &&
							fabs(plane_i.v3colorNrgb[1] - 0.33) < 0.03 &&
							fabs(plane_i.v3colorNrgb[2] - 0.33) < 0.03 &&
							plane_i.dominantIntensity > 400)
							illum = 0.5;

						pcl::visualization::PointCloudColorHandlerCustom<PointT>
							color(
								plane_i.planePointCloudPtr,
								plane_i.v3colorNrgb[0] *
									(plane_i.dominantIntensity +
									 (755 - plane_i.dominantIntensity) * illum),
								plane_i.v3colorNrgb[1] *
									(plane_i.dominantIntensity +
									 (755 - plane_i.dominantIntensity) * illum),
								plane_i.v3colorNrgb[2] *
									(plane_i.dominantIntensity +
									 (755 - plane_i.dominantIntensity) *
										 illum));
						//            pcl::visualization::PointCloudColorHandlerCustom
						//            <PointT> color
						//            (plane_i.planePointCloudPtr,
						//                                                                              plane_i.v3colorNrgb[0] * plane_i.dominantIntensity,
						//                                                                              plane_i.v3colorNrgb[1] * plane_i.dominantIntensity,
						//                                                                              plane_i.v3colorNrgb[2] * plane_i.dominantIntensity);
						viz.addPointCloud(
							plane_i.planePointCloudPtr, color, name);
						viz.setPointCloudRenderingProperties(
							pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,
							name);
						//          }
					}
					//          else
					//            viz.addPointCloud (plane_i.planePointCloudPtr,
					//            name);// contourPtr, planePointCloudPtr,
					//            polygonContourPtr

					//          sprintf (name, "planeBorder_%02u",
					//          static_cast<unsigned>(i));
					//          pcl::visualization::PointCloudColorHandlerCustom
					//          <PointT> color2 (plane_i.contourPtr, 255, 255,
					//          255);
					//          viz.addPointCloud (plane_i.contourPtr, color2,
					//          name);// contourPtr, planePointCloudPtr,
					//          polygonContourPtr

					//          //Edges
					//          if(mPbMap.edgeCloudPtr->size() > 0)
					//          {
					//            sprintf (name, "planeEdge_%02u",
					//            static_cast<unsigned>(i));
					//            pcl::visualization::PointCloudColorHandlerCustom
					//            <PointT> color4 (mPbMap.edgeCloudPtr, 255,
					//            255, 0);
					//            viz.addPointCloud (mPbMap.edgeCloudPtr,
					//            color4, name);// contourPtr,
					//            planePointCloudPtr, polygonContourPtr
					//            viz.setPointCloudRenderingProperties
					//            (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
					//            5, name);
					//
					//            sprintf (name, "edge%u",
					//            static_cast<unsigned>(i));
					//            viz.addLine
					//            (mPbMap.edgeCloudPtr->points.front(),
					//            mPbMap.edgeCloudPtr->points.back(), ared[3],
					//            agrn[3], ablu[3], name);
					//          }

					sprintf(name, "approx_plane_%02d", int(i));
					viz.addPolygon<PointT>(
						plane_i.polygonContourPtr, 0.5 * red[i % 10],
						0.5 * grn[i % 10], 0.5 * blu[i % 10], name);
				}

				// if(configPbMap.makeClusters)
				//  for(map<unsigned, std::vector<unsigned> >::iterator
				//  it=clusterize->groups.begin(); it !=
				//  clusterize->groups.end(); it++)
				//    for(size_t i=0; i < it->second.size(); i++)
				//    {
				//      unsigned planeID = it->second[i];
				//      Plane &plane_i = mPbMap.vPlanes[planeID];
				//      sprintf (name, "plane_%02u",
				//      static_cast<unsigned>(planeID));
				//      pcl::visualization::PointCloudColorHandlerCustom
				//      <PointT> color (plane_i.planePointCloudPtr,
				//      red[planeID%10], grn[planeID%10], blu[planeID%10]);
				//      viz.addPointCloud (plane_i.planePointCloudPtr, color,
				//      name);// contourPtr, planePointCloudPtr,
				//      polygonContourPtr
				//      viz.setPointCloudRenderingProperties
				//      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
				//      name);
				//    }

				// Draw recognized plane labels
				if (mpPbMapLocaliser != nullptr)
					for (auto it = mpPbMapLocaliser->foundPlaces.begin();
						 it != mpPbMapLocaliser->foundPlaces.end(); it++)
						viz.addText3D(
							it->first, it->second, 0.3, 0.9, 0.9, 0.9,
							it->first);
			}
		}
	}
}

void PbMapMaker::run()
{
	cloudViewer.runOnVisualizationThread(
		boost::bind(&PbMapMaker::viz_cb, this, _1), "viz_cb");
	cloudViewer.registerKeyboardCallback(keyboardEventOccurred);

	size_t numPrevKFs = 0;
	size_t minGrowPlanes = 5;
	while (!m_pbmaker_must_stop)  // Stop loop if PbMapMaker
	{
		if (numPrevKFs == frameQueue.size())
		{
			std::this_thread::sleep_for(10ms);
		}
		else
		{
			// Assign pointCloud of last KF to the global map
			detectPlanesCloud(
				frameQueue.back().cloudPtr, frameQueue.back().pose,
				configPbMap.dist_threshold, configPbMap.angle_threshold,
				configPbMap.minInliersRate);

			if (configPbMap.makeClusters)
			{
				if (mPbMap.vPlanes.size() > minGrowPlanes)
				{
					// Evaluate the partition of the current groups with minNcut
					int size_partition =
						clusterize->evalPartition(observedPlanes);
					cout << "PARTITION SIZE " << size_partition << endl;
					assert(size_partition < 2);

					minGrowPlanes += 2;
				}
			}

			++numPrevKFs;
		}
	}

	//  saveInfoFiles(); // save watch statistics

	m_pbmaker_finished = true;
}

void PbMapMaker::serializePbMap(string path)
{
	boost::mutex::scoped_lock updateLock(mtx_pbmap_busy);

	mPbMap.savePbMap(path);

	updateLock.unlock();
}

bool PbMapMaker::stop_pbMapMaker()
{
	m_pbmaker_must_stop = true;
	while (!m_pbmaker_finished) std::this_thread::sleep_for(1ms);
	cout << "Waiting for PbMapMaker thread to die.." << endl;

	pbmaker_hd.join();

	return true;
}

PbMapMaker::~PbMapMaker()
{
	delete mpPlaneInferInfo;
	delete mpPbMapLocaliser;

	stop_pbMapMaker();

	cout << " .. PbMapMaker has died." << endl;
}

#endif

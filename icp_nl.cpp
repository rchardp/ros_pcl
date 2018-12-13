#include <iostream>
#include <cstdlib>
#include <sstream>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZRGBNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;


class RegConfig {
public:
  RegConfig():
    k(30),
    maxIters(30),
    maxCorrespondenceDistance(.5f),
    transformationEpsilon(1e-16f) {}
  int k;
  int maxIters;
  double maxCorrespondenceDistance;
  double transformationEpsilon;
};

class SegConfig {
public:
  SegConfig(): 
    negative(false),
    distanceThreshold(0.01f) {}
  bool negative;
  double distanceThreshold;
};

class VGConfig{
public:
  VGConfig():
    leafSize(0.01f) {}
  Eigen::Vector4f getLeafSize() {
    Eigen::Vector4f leaf_size;
    leaf_size[0] = leaf_size[1] = leaf_size[2] = leafSize;
    return leaf_size;
  }
  double leafSize;
};

class Config {
public:
  Config():
    firstWall(true),
    alpha(0.0f) {}
  RegConfig reg;
  VGConfig  vg;
  SegConfig seg;
  bool firstWall;
  double alpha;
};

// Global configuration
Config cfg;

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (PointCloudT::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->initCameraParameters ();
  return (viewer);
}

std::string fname(std::string& basename, int x) {
    std::stringstream ss;
    ss << basename << x << ".pcd";
    std::cerr << "loading [" <<ss.str() << "]" << std::endl;
    return ss.str();
}

PointCloudT::Ptr vgFilter(PointCloudT::Ptr cloud) {
  PointCloudT::Ptr t_cloud (new PointCloudT);
  pcl::VoxelGrid<PointT> vg;

  vg.setInputCloud (cloud);
  vg.setLeafSize(cfg.vg.leafSize, cfg.vg.leafSize, cfg.vg.leafSize);
  vg.filter(*t_cloud);

  return t_cloud;
}

PointCloudT::Ptr removeWall(PointCloudT::Ptr cloud) {
  PointCloudT::Ptr t_cloud (new PointCloudT);
  // created RandomSampleConsensus object
  pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);

  // prepare coefficients with plane equation
  // plane->values.resize (4);

  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (cfg.seg.distanceThreshold);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *plane);

  if (inliers->indices.size () == 0) {
    std::cerr << "Could not estimate a planar model for cloud." << std::endl;
    exit(-1);
  } else {
    std::cerr << "wall removed!" << std::endl;
  }

  pcl::ExtractIndices<PointT> extract;
  // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (cfg.seg.negative);
  extract.filter (*t_cloud);

  return t_cloud;
}

Eigen::Matrix4f align(PointCloudT::Ptr src, PointCloudT::Ptr tgt) {
  PointCloudT::Ptr output (new PointCloudT);
  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (cfg.reg.k);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (cfg.reg.transformationEpsilon);
  reg.setMaxCorrespondenceDistance (cfg.reg.maxCorrespondenceDistance);
  
  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);

  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (3);

  for (int i = 0; i < cfg.reg.maxIters; ++i) {
    std::cerr << "It [" << i << "]";

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

    //accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

    //if the difference between this transformation and the previous one
    //is smaller than the threshold, refine the process by reducing
    //the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();
    std::cerr << " - score: " << reg.getFitnessScore() << std::endl;

  }

  // Get the transformation from target to source
  return Ti.inverse();
}

PointCloudT::Ptr rotateY(PointCloudT::Ptr cloud) {
  PointCloudT::Ptr t_cloud (new PointCloudT);
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  transform.rotate (Eigen::AngleAxisf (cfg.alpha, Eigen::Vector3f::UnitY()));

  pcl::transformPointCloud (*cloud, *t_cloud, transform);
  return t_cloud;
}

int
main(int argc, char** argv)
{
  int argIdx;
  // initialize PointClouds
  PointCloudT::Ptr cloud1 (new PointCloudT),
                   cloud2 (new PointCloudT),
                   t_cloud1 (new PointCloudT),
                   t_cloud2 (new PointCloudT),
                   final (new PointCloudT);
  // load files
  argIdx = pcl::console::find_argument (argc, argv, "-f");
  if ( argIdx >= 0 ) {
    std::string filename(argv[argIdx+1]);
    pcl::io::loadPCDFile( fname(filename, 0), *cloud1 );
    pcl::io::loadPCDFile( fname(filename, 1), *cloud2 );
    std::cerr << filename << std::endl;
  } else {
    std::cerr << "Usage: " << argv[0] << " [options] " << std::endl
              << "options:" << std::endl
              << "-f <filename>" << std::endl
              << "-n <negative? (ransac)>" << std::endl
              << "-v <voxelgrid leaf size>" << std::endl
              << "-t <distance threshold (ransac)>" << std::endl
              << "-c <max correspondence distance (icp_nl)>" << std::endl
              << "-e <transformation epsilon (icp_nl)>" << std::endl
              << "-k <k neighbors (kdtree)>" << std::endl
              << "-i <max iters> (icp_nl)" << std::endl
              << "-w <remove wall? (1st cloud)>" << std::endl
              << "-a <angle (rotate 2nd cloud)>" << std::endl;
    exit(1);
  }

  if ( pcl::console::find_argument(argc, argv, "-n") >= 0 ) {
    cfg.seg.negative = true;
    std::cerr << "Setting negative to true" << std::endl;
  }

  if ( pcl::console::find_argument(argc, argv, "-w") >= 0 ) {
    cfg.firstWall = false;
    std::cerr << "Setting Remove first wall to true" << std::endl;
  }

  argIdx = pcl::console::find_argument (argc, argv, "-t");
  if (argIdx >= 0) {
    cfg.seg.distanceThreshold = atof(argv[argIdx+1]);
    std::cerr << "Setting Distance Threshold to: " << cfg.seg.distanceThreshold << std::endl;
  }

  argIdx = pcl::console::find_argument (argc, argv, "-v");
  if (argIdx >= 0) {
    cfg.vg.leafSize = atof(argv[argIdx+1]);
    std::cerr << "Setting Leaf Size to: " << cfg.vg.leafSize << std::endl;
  }

  argIdx = pcl::console::find_argument (argc, argv, "-c");
  if (argIdx >= 0) {
    cfg.reg.maxCorrespondenceDistance = atof(argv[argIdx+1]);
    std::cerr << "Setting Max Correspondence Distance to: " << cfg.reg.maxCorrespondenceDistance << std::endl;
  }

  argIdx = pcl::console::find_argument (argc, argv, "-e");
  if (argIdx >= 0) {
    cfg.reg.transformationEpsilon = atof(argv[argIdx+1]);
    std::cerr << "Setting Transformation Epsilon to: " << cfg.reg.transformationEpsilon << std::endl;
  }

  argIdx = pcl::console::find_argument (argc, argv, "-k");
  if (argIdx >= 0) {
    cfg.reg.k = atoi(argv[argIdx+1]);
    std::cerr << "Setting K neighbors to: " << cfg.reg.k << std::endl;
  }

  argIdx = pcl::console::find_argument (argc, argv, "-i");
  if (argIdx >= 0) {
    cfg.reg.maxIters = atoi(argv[argIdx+1]);
    std::cerr << "Setting Max Iters to: " << cfg.reg.maxIters << std::endl;
  }

  argIdx = pcl::console::find_argument (argc, argv, "-a");
  if (argIdx >= 0) {
    cfg.alpha = atof(argv[argIdx+1]);
    std::cerr << "Angle rotation to: " << cfg.alpha << std::endl;
    cloud2 = rotateY(cloud2);
  }
  
  // temporary clouds
  t_cloud1 = vgFilter(cloud1);
  t_cloud2 = vgFilter(cloud2);

  if (not cfg.firstWall) {
    t_cloud1 = removeWall(t_cloud1);
  }

  t_cloud2 = removeWall(t_cloud2);

  Eigen::Matrix4f finalTransform = align(t_cloud1, t_cloud2);

  // apply transformation to real cloud
  pcl::transformPointCloud (*cloud2, *final, finalTransform);
  // concat clouds (source & target)
  *final += *cloud1;

  // creates the visualization object
  boost::shared_ptr<pcl::visualization::PCLVisualizer> vis;
  vis = simpleVis(final);
  vis->spin();

  return 0;
 }
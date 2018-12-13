#include <iostream>
#include <sstream>
#include <cstdlib>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


boost::shared_ptr<pcl::visualization::PCLVisualizer>
rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	return (viewer);
}

int
main(int argc, char const *argv[]) {
	if(argc < 2) {
		PCL_ERROR( "Usage: %s <filename>[ .. <filename> ]\n", argv[0] );
		return -1;
	}

	int pc_count = argc-1;
	double theta;

	try {

		theta = 2*M_PI/pc_count;

		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB> );
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr c_cloud( new pcl::PointCloud<pcl::PointXYZRGB> );

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr t_cloud( new pcl::PointCloud<pcl::PointXYZRGB> );
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();

		for( int i = 0; i < pc_count; ++i ) {
			if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[i+1], *cloud) == -1) {
				PCL_ERROR ("Couldn't read file %s\n", argv[i+1]);
				return (-1);
			}

			transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));

			pcl::transformPointCloud (*cloud, *t_cloud, transform);

			if( not i )
				*c_cloud = *t_cloud;
			else
				*c_cloud += *t_cloud;
		}

	
		viewer = rgbVis( c_cloud );

		while (!viewer->wasStopped ()) {
			viewer->spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
	} catch(...) {
		PCL_ERROR( "Error!\n" );
	}
	
	
	return 0;
}

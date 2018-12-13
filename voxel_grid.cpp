#include <iostream>
#include <sstream>
#include <cstdlib>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGB PointT;

int
main (int argc, char** argv)
{
	double leafSize;
	std::stringstream ss;
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);


	if( argc != 3 ) {
		PCL_ERROR( "Usage: %s <fileName> <leafSize>\n", argv[0] );
		return -1;
	}

	if ( pcl::io::loadPCDFile<PointT>( argv[1], *cloud ) == -1 ) {
		PCL_ERROR( "Could not load file: '%s'\n", argv[1] );
		return -1;
	}
	
	leafSize = atof( argv[2] );
	// Create the filtering object
	pcl::VoxelGrid<PointT> vg;
	vg.setInputCloud (cloud);
	vg.setLeafSize (leafSize, leafSize, leafSize);
	vg.filter (*cloud_filtered);

	ss << "vg." << argv[1];
	
	if (pcl::io::savePCDFile(
		ss.str().c_str() , *cloud_filtered, true) == 0) {
		PCL_INFO( "Saved: %s\n", ss.str().c_str() );
	}

	return 0;
}

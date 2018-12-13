#include <iostream>
#include <cstdlib>
#include <sstream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGB PointT;

int
 main (int argc, char** argv)
{
	std::stringstream ss;
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

	if( argc != 4 ) {
		PCL_ERROR( "Usage: %s <filename> <axis> <value>\n", argv[0] );
		return -1;
	}
	
	if ( pcl::io::loadPCDFile<PointT>( argv[1], *cloud ) == -1 ) {
		PCL_ERROR( "Could not load file: '%s'\n", argv[1] );
		return -1;
	}

	// Create the filtering object
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ( argv[2] );
	pass.setFilterLimits (0.0, atof(argv[3]) );
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered);

	ss << "pass." << argv[1];

	if (pcl::io::savePCDFile( ss.str().c_str() , *cloud_filtered, true) == 0) {
		PCL_INFO( "Saved: %s\n", ss.str().c_str() );
	}

  return (0);
}

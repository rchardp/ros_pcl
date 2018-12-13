#include <iostream>
#include <sstream>
#include <cstdlib>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZRGB PointT;

int
main (int argc, char** argv)
{
	int meanK;
	std::stringstream ss;
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);


	if( argc != 3 ) {
		PCL_ERROR( "Usage: %s <fileName> <meanK>\n", argv[0] );
		return -1;
	}

	if ( pcl::io::loadPCDFile<PointT>( argv[1], *cloud ) == -1 ) {
		PCL_ERROR( "Could not load file: '%s'\n", argv[1] );
		return -1;
	}
	
	meanK = atoi( argv[2] );
	

	pcl::StatisticalOutlierRemoval<PointT> ol;
	ol.setInputCloud (cloud);
	// Number of neighbors to analyze
	ol.setMeanK ( meanK );
	// Standard deviation multiplier
	ol.setStddevMulThresh (1.0);
	/* all points who have a distance larger than 1 standard deviation
		of the mean distance to the query point will be marked as outliers and removed
	*/

	ol.setNegative( true );	// negative output -> true = outlier; false = inlier

	ol.filter (*cloud_filtered);	// The output is computed and stored in cloud_filtered

	ss << "ol." << argv[1];
	
	if (pcl::io::savePCDFile(
		ss.str().c_str() , *cloud_filtered, true) == 0) {
		PCL_INFO( "Saved: %s\n", ss.str().c_str() );
	}

	return 0;
}

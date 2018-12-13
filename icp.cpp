#include <iostream>
#include <sstream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZRGB PointT;

int
 main (int argc, char** argv)
{
	std::stringstream ss;
	pcl::PointCloud<PointT>::Ptr cloud_in (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr Final (new pcl::PointCloud<PointT>);

	if( argc != 3 ) {
		PCL_ERROR( "Usage: %s <fileNameTarget> <fileNameSource>", argv[0] );
		return -1;
	}

	// cargar pointcloud de entrada
	std::cout << "Loading " <<argv[1] << "..." << std::flush;

	if(pcl::io::loadPCDFile( argv[1], *cloud_in ) == -1) {
		PCL_ERROR( "[FAIL]\n", argv[1] );
		return -1;
	}
	PCL_INFO( "[ OK ]\n" );
	// cargar pointcloud para alinear
	std::cout << "Loading " <<argv[2] << "..." << std::flush;

	if(pcl::io::loadPCDFile( argv[2], *cloud_out ) == -1) {
		PCL_ERROR( "[FAIL]\n", argv[2] );
		return -1;
	}
	PCL_INFO( "[ OK ]\n" );

	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_out);
//	pcl::PointCloud<PointT> Final;
	icp.align(*Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	
	ss << "icp."<< argv[2];
	
	if (pcl::io::savePCDFile( ss.str(), *Final, true ) == 0) {
		PCL_INFO( "Saved: %s\n", ss.str().c_str() );
	}

	return (0);
}

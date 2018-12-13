#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>

typedef pcl::PointXYZRGB PointT;

int main(int argc, char const *argv[])
{
	
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	viewer.setBackgroundColor (0, 0, 0);
	viewer.addCoordinateSystem (1.0);
	viewer.initCameraParameters ();


	for( int i = 1; i < argc; ++i )
	{
		std::cout << argv[i] << "..." << std::flush;
		if(pcl::io::loadPCDFile( argv[i], *cloud ) == -1)
		{
			PCL_ERROR( "[FAIL]\n", argv[i] );
			return -1;
		}
		pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
		
		viewer.addPointCloud<PointT>( cloud, rgb, argv[i] );
		PCL_INFO( "[ OK ]\n" );
	}

	while (!viewer.wasStopped ())
	{
		viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return 0;
}

#include <iostream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


boost::shared_ptr<pcl::visualization::PCLVisualizer>
View_2Clouds (
  pcl::PCLPointCloud2::ConstPtr cloud1,
  pcl::PCLPointCloud2::ConstPtr cloud2 ) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->initCameraParameters ();

	pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud1( new pcl::PointCloud<pcl::PointXYZ>() );
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud2( new pcl::PointCloud<pcl::PointXYZ>() );
	pcl::fromPCLPointCloud2( *cloud1, *xyzCloud1 );
	pcl::fromPCLPointCloud2( *cloud2, *xyzCloud2 );

	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor (0, 0, 0, v1);
	viewer->addText("Point Cloud Original", 10, 10, "v1 text", v1);
	//pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgb1(xyzCloud1);
	viewer->addPointCloud (xyzCloud1, /*rgb1,*/ "sample cloud1", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor (0, 0, 0, v2);
	viewer->addText("Point Cloud Filtered", 10, 10, "v2 text", v2);
	//pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgb2(xyzCloud2);
	viewer->addPointCloud (xyzCloud2, /*rgb2,*/ "sample cloud2", v2);

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
	viewer->addCoordinateSystem (1.0);

	return(viewer);
}

void
voxel_grid(
	pcl::PCLPointCloud2::Ptr cloud,
	pcl::PCLPointCloud2::Ptr cloud_filtered )
{

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);	// leaf size of 1cm
	sor.filter (*cloud_filtered);			// The output is computed and stored in cloud_filtered

}

void
passthrough(
	pcl::PCLPointCloud2::Ptr cloud,
	pcl::PCLPointCloud2::Ptr cloud_filtered )
{

	pcl::PassThrough<pcl::PCLPointCloud2> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("x");			// The filter field name is set to the x coordinate
	pass.setFilterLimits (0.0, 1.0);		// The accepted interval values are set to (0.0;1.0)
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered);			// The output is computed and stored in cloud_filtered

}

void
outlier(
	pcl::PCLPointCloud2::Ptr cloud,
	pcl::PCLPointCloud2::Ptr cloud_filtered,
	bool negative=true )
{

	pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (50);				// Number of neighbors to analyze
	sor.setStddevMulThresh (1.0);	// Standard deviation multiplier
	/* all points who have a distance larger than 1 standard deviation
		of the mean distance to the query point will be marked as outliers and removed
	*/

	sor.setNegative( negative );	// negative output -> true = outlier; false = inlier

	sor.filter (*cloud_filtered);	// The output is computed and stored in cloud_filtered

}


int
main(int argc, char const *argv[]) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	pcl::PCLPointCloud2::Ptr cloud( new pcl::PCLPointCloud2 () );
	pcl::PCLPointCloud2::Ptr cloud_filtered( new pcl::PCLPointCloud2 () );

	if( argc != 3 ) {
		PCL_ERROR( "Usage: %s <option> <filename>\n", argv[0] );
		PCL_ERROR( "options:\n%s\n%s\n%s\n%s\n",
					"-v\tVoxelGrid",
					"-p\tPassThrough",
					"-o\tOutliers",
					"-i\tInliers"
				 );
		return -1;
	}

	pcl::PCDReader reader;

	reader.read( argv[2], *cloud );

	std::stringstream ss;
	ss << argv[1];

	/* VOXEL GRID */
	if( ss.str() == std::string("-v") ) {
		voxel_grid( cloud, cloud_filtered );
		}
	/* PASSTHORUGH */
	else if( ss.str() == std::string("-p") ) {
		passthrough( cloud, cloud_filtered );
		}
	/* INLIERS */
	else if( ss.str() == std::string("-i") ) {
		outlier( cloud, cloud_filtered, false );
		}
	/* OUTLIERS */
	else if( ss.str() == std::string("-o") ) {
		outlier( cloud, cloud_filtered);
		}
	else {
		PCL_ERROR( "Invalid Option: '%s'\n", argv[1] );
		return -1;
		}
	
	
	viewer = View_2Clouds(cloud,cloud_filtered);

	while(!viewer->wasStopped()) {
      viewer->spinOnce (100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
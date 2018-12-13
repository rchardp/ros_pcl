#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer>
rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)	
{
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
point_cloud()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr
		(new pcl::PointCloud<pcl::PointXYZRGB>);

	uint8_t r(255), g(15), b(15);
	for (float z(-1.0); z <= 1.0; z += 0.05)
	{
		for (float angle(0.0); angle <= 360.0; angle += 5.0)
		{
			pcl::PointXYZRGB point;
			point.x = .2+(0.5 * cosf (pcl::deg2rad(angle)));
			point.y = .1+(sinf (pcl::deg2rad(angle)));
			point.z = z;
			uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
				static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			point_cloud_ptr->points.push_back (point);
		}
		if (z < 0.0)
		{
			r -= 12;
			g += 12;
		}
		else
		{
			g -= 12;
			b += 12;
		}
	}

	point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
	point_cloud_ptr->height = 1;

	pcl::io::savePCDFileASCII ("test0.pcd", *point_cloud_ptr);

	return point_cloud_ptr;
}

int
main(int argc, char const *argv[])
{
	//std::cout << "Generating example point clouds.\n\n";
	/*if(argc != 2)
	{
		PCL_ERROR( "Usage: %s <pcd_file_name>\n", argv[0] );
		return -1;
	}*/
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_c( new pcl::PointCloud<pcl::PointXYZRGB> );
//	pcl::PointCloud<pcl::PointXYZRGB> cloud_a;

/*	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud_c) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file %s\n", argv[1]);
		return (-1);
	}*/

	cloud_c = point_cloud();
	//cloud_a = *cloud_c;
	//for( size_t i = 0; i < cloud_a.size(); ++i )
	//	cloud_a.points[i].x += 1;
	//*cloud_c += cloud_a;
	
	viewer = rgbVis( cloud_c );

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	return 0;
}

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

//#include <pcl/io/openni_grabber.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/cloud_viewer.h>

using namespace pcl;
using namespace std;

PointCloud<PointXYZRGB>::Ptr cloud_ptr( new PointCloud<PointXYZRGB> );
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//Grabber* openniGrabber;
bool saveCloud(false);

void
keyboardEventOccurred( const visualization::KeyboardEvent& event,
						void* nothing )
{
	if( event.getKeySym() == "space" && event.keyDown() )
		saveCloud = true;
}

// This function is called every time the device has new data.
void
grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
{
	if (! viewer->wasStopped())
		viewer->showCloud(cloud);
 
	if (saveCloud)
	{
		//stringstream stream;
		//stream << "inputCloud" << filesSaved << ".pcd";
		//string filename = stream.str();
		//if (io::savePCDFile(filename, *cloud, true) == 0)
		//{
		//	filesSaved++;
		cout << "Saved !" << endl;
		//}
		//else PCL_ERROR("Problem saving %s.\n", filename.c_str());
 
		saveCloud = false;
	}
}

void
PCL_callback(const sensor_msgs::PointCloud2 msg)
{
	pcl::fromROSMsg(msg, *cloud_ptr);
	pcl::visualization::PointCloudColorHandleRGBField<pcl::PointXYZRGB> rgb (cloud_ptr);
	viewer->removePointCloud();
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_ptr, rgb);
	viewer->spinOnce();
}

boost::shared_ptr<visualization::CloudViewer>
createViewer()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> v
		(new pcl::visualization::PCLVisualizer("PCL Viewer"));
	v->setBackgroundColor(0,0,0);
	v->addCoordinateSystem(1.0f);
	v->initCameraParameters();
	//boost::shared_ptr<visualization::CloudViewer> v
	//	(new visualization::CloudViewer("Viewer"));
	//v->registerKeyboardCallback( keyboardEventOccurred );
	return v;
}

int main(int argc, char const *argv[])
{
	//openniGrabber = new OpenNIGrabber();

	//boost::function<void (const PointCloud<PointXYZRGB>::ConstPtr&)> f =
	//	boost::bind(&grabberCallback, _1);
	//openniGrabber->registerCallback(f);

	viewer = createViewer();

	ros::init( argc, argv, "PCL_ROS" );
	ros::NodeHandle pcl_node;
	ros::Subscriber pcl_subscriber =
		pcl_node.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, PCL_callback);

	ros::spin();
	//openniGrabber->start();

	//while( !viewer->wasStopped() )
	//	boost::this_thread::sleep( boost::posix_time::seconds(1) );

	//openniGrabber->stop();

	return 0;
}
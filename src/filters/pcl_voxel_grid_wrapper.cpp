#include <boost/python.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclpy/wrapper.h>

typedef pcl::PointXYZRGB PointT;

class VoxelGridWrapper : public pcl::VoxelGrid<PointT>
{
public:
	VoxelGridWrapper() : VoxelGrid() {}
	void setInputCloud( std::string cloud )
	{
		sensor_msgs::PointCloud2 cloud2 = from_python<sensor_msgs::PointCloud2>( cloud );
		pcl::PointCloud<PointT>::Ptr _cloud( new pcl::PointCloud<PointT> );
		pcl::moveFromROSMsg( cloud2, *_cloud );
		VoxelGrid::setInputCloud( _cloud );
	}

	void setLeafSize( std::string lx, std::string ly, std::string lz )
	{
		std_msgs::Float32 _lx = from_python<std_msgs::Float32>( lx );
		std_msgs::Float32 _ly = from_python<std_msgs::Float32>( ly );
		std_msgs::Float32 _lz = from_python<std_msgs::Float32>( lz );
		VoxelGrid::setLeafSize( _lx.data, _ly.data, _lz.data );
	}

	std::string filter()
	{
		pcl::PointCloud<PointT> _cloud;
		sensor_msgs::PointCloud2 cloud2;
		VoxelGrid::filter( _cloud );
		pcl::toROSMsg( _cloud, cloud2 );
		return to_python( cloud2 );
	}
};

BOOST_PYTHON_MODULE( _pcl_voxel_grid_wrapper_cpp ) {
	boost::python::class_<VoxelGridWrapper> ( "VoxelGridWrapper", boost::python::init<>())
	.def( "setInputCloud", &VoxelGridWrapper::setInputCloud )
	.def( "setLeafSize", &VoxelGridWrapper::setLeafSize )
	.def( "filter", &VoxelGridWrapper::filter )
	;
}

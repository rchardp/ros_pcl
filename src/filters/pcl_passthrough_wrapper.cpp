#include <boost/python.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclpy/wrapper.h>

typedef pcl::PointXYZRGB PointT;

class PassThroughWrapper : public pcl::PassThrough<PointT>
{
public:
	PassThroughWrapper() : PassThrough() {}
	void setInputCloud( std::string cloud )
	{
		sensor_msgs::PointCloud2 cloud2 = from_python<sensor_msgs::PointCloud2>( cloud );
		pcl::PointCloud<PointT>::Ptr _cloud( new pcl::PointCloud<PointT> );
		pcl::moveFromROSMsg( cloud2, *_cloud );
		PassThrough::setInputCloud( _cloud );
	}

	void setFilterFieldName( std::string field_name)
	{
		std_msgs::String _field = from_python<std_msgs::String>( field_name );
		PassThrough::setFilterFieldName( _field.data );
	}

	void setFilterLimits( std::string limit_min, std::string limit_max )
	{
		std_msgs::Float32 _min =from_python<std_msgs::Float32>( limit_min );
		std_msgs::Float32 _max =from_python<std_msgs::Float32>( limit_max );
		PassThrough::setFilterLimits( _min.data, _max.data );
	}

	void setNegative( std::string negative )
	{
		std_msgs::Bool _neg = from_python<std_msgs::Bool>( negative );
		PassThrough::setNegative( _neg.data );
	}

	std::string filter()
	{
		pcl::PointCloud<PointT> _cloud;
		sensor_msgs::PointCloud2 cloud2;
		PassThrough::filter( _cloud );
		pcl::toROSMsg( _cloud, cloud2 );
		return to_python( cloud2 );
	}
};

BOOST_PYTHON_MODULE( _pcl_passthrough_wrapper_cpp ) {
	boost::python::class_<PassThroughWrapper> ( "PassThroughWrapper", boost::python::init<>())
	.def( "setInputCloud", &PassThroughWrapper::setInputCloud )
	.def( "setFilterFieldName", &PassThroughWrapper::setFilterFieldName )
	.def( "setFilterLimits", &PassThroughWrapper::setFilterLimits )
	.def( "setNegative", &PassThroughWrapper::setNegative )
	.def( "filter", &PassThroughWrapper::filter )
	;
}

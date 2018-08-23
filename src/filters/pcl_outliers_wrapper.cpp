#include <boost/python.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclpy/wrapper.h>

typedef pcl::PointXYZRGB PointT;

class StatisticalOutlierRemovalWrapper : public pcl::StatisticalOutlierRemoval<PointT>
{
public:
	StatisticalOutlierRemovalWrapper() : StatisticalOutlierRemoval() {}
	void setInputCloud( std::string cloud )
	{
		sensor_msgs::PointCloud2 cloud2 = from_python<sensor_msgs::PointCloud2>( cloud );
		pcl::PointCloud<PointT>::Ptr _cloud( new pcl::PointCloud<PointT> );
		pcl::moveFromROSMsg( cloud2, *_cloud );
		StatisticalOutlierRemoval::setInputCloud( _cloud );
	}

	void setMeanK( std::string nr_k )
	{
		std_msgs::Int32 _nr_k = from_python<std_msgs::Int32>( nr_k );
		StatisticalOutlierRemoval::setMeanK( _nr_k.data );
	}

	void setStddevMulThresh( std::string stddev_mult )
	{
		std_msgs::Float32 _stddev_mult =from_python<std_msgs::Float32>( stddev_mult );
		StatisticalOutlierRemoval::setStddevMulThresh( _stddev_mult.data );
	}

	void setNegative( std::string negative )
	{
		std_msgs::Bool _neg = from_python<std_msgs::Bool>( negative );
		StatisticalOutlierRemoval::setNegative( _neg.data );
	}

	std::string filter()
	{
		pcl::PointCloud<PointT> _cloud;
		sensor_msgs::PointCloud2 cloud2;
		StatisticalOutlierRemoval::filter( _cloud );
		pcl::toROSMsg( _cloud, cloud2 );
		return to_python( cloud2 );
	}
};

BOOST_PYTHON_MODULE( _pcl_outliers_wrapper_cpp ) {
	boost::python::class_<StatisticalOutlierRemovalWrapper> ( "StatisticalOutlierRemovalWrapper", boost::python::init<>())
	.def( "setInputCloud", &StatisticalOutlierRemovalWrapper::setInputCloud )
	.def( "setStddevMulThresh", &StatisticalOutlierRemovalWrapper::setStddevMulThresh )
	.def( "setMeanK", &StatisticalOutlierRemovalWrapper::setMeanK )
	.def( "setNegative", &StatisticalOutlierRemovalWrapper::setNegative )
	.def( "filter", &StatisticalOutlierRemovalWrapper::filter )
	;
}

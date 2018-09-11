
#include <pclpy/definitions.h>
#include <pcl/registration/icp.h>
#include <sstream>

class IterativeClosestPointWrapper : public pcl::IterativeClosestPoint<PointT, PointT>
{

public:
	IterativeClosestPointWrapper() : IterativeClosestPoint() {}

	void setInputSource(std::string cloud)
	{
		sensor_msgs::PointCloud2 cloud2 = from_python<sensor_msgs::PointCloud2>( cloud );
		pcl::PointCloud<PointT>::Ptr _cloud( new pcl::PointCloud<PointT> );
		pcl::moveFromROSMsg( cloud2, *_cloud );
		IterativeClosestPoint::setInputSource( _cloud );
	}

	void setInputTarget(std::string cloud)
	{
		sensor_msgs::PointCloud2 cloud2 = from_python<sensor_msgs::PointCloud2>( cloud );
		pcl::PointCloud<PointT>::Ptr _cloud( new pcl::PointCloud<PointT> );
		pcl::moveFromROSMsg( cloud2, *_cloud );
		IterativeClosestPoint::setInputTarget( _cloud );
	}

	void setMaxCorrespondenceDistance(std::string distance_threshold)
	{
		std_msgs::Float32 _dist = from_python<std_msgs::Float32>( distance_threshold );
		IterativeClosestPoint::setMaxCorrespondenceDistance( _dist.data );
	}

	std::string getMaxCorrespondenceDistance()
	{
		std_msgs::Float32 _dist;
		_dist.data = IterativeClosestPoint::getMaxCorrespondenceDistance();
		return to_python(_dist);
	}

	void setMaximumIterations(std::string nr_iterations)
	{
		std_msgs::Int32 _iters = from_python<std_msgs::Int32>( nr_iterations );
		IterativeClosestPoint::setMaximumIterations( _iters.data );
	}

	std::string getMaximumIterations()
	{
		std_msgs::Int32 _iters;
		_iters.data = IterativeClosestPoint::getMaximumIterations();
		return to_python( _iters );
	}

	std::string align()
	{
		pcl::PointCloud<PointT> _cloud;
		sensor_msgs::PointCloud2 cloud2;
		IterativeClosestPoint::align( _cloud );
		pcl::toROSMsg( _cloud, cloud2 );
		return to_python( cloud2 );
	}

};

BOOST_PYTHON_MODULE( _pcl_icp_wrapper_cpp ) {
	boost::python::class_<IterativeClosestPointWrapper> ( "IterativeClosestPointWrapper", boost::python::init<>())
	.def( "setInputSource", &IterativeClosestPointWrapper::setInputSource )
	.def( "setInputTarget", &IterativeClosestPointWrapper::setInputTarget )
	.def( "setMaxCorrespondenceDistance", &IterativeClosestPointWrapper::setMaxCorrespondenceDistance )
	.def( "getMaxCorrespondenceDistance", &IterativeClosestPointWrapper::getMaxCorrespondenceDistance )
	.def( "setMaximumIterations", &IterativeClosestPointWrapper::setMaximumIterations )
	.def( "getMaximumIterations", &IterativeClosestPointWrapper::getMaximumIterations )
	.def( "align", &IterativeClosestPointWrapper::align )
	;
}

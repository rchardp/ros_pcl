#include <pclpy/registration/icp.h>
#include <sstream>

IterativeClosestPointWrapper::IterativeClosestPointWrapper() :
	IterativeClosestPoint() {}

void IterativeClosestPointWrapper::setInputSource(std::string cloud)
{
	sensor_msgs::PointCloud2 cloud2 = from_python<sensor_msgs::PointCloud2>( cloud );
	pcl::PointCloud<PointT>::Ptr _cloud( new pcl::PointCloud<PointT> );
	pcl::moveFromROSMsg( cloud2, *_cloud );
	IterativeClosestPoint::setInputSource( _cloud );
}

void IterativeClosestPointWrapper::setInputTarget(std::string cloud)
{
	sensor_msgs::PointCloud2 cloud2 = from_python<sensor_msgs::PointCloud2>( cloud );
	pcl::PointCloud<PointT>::Ptr _cloud( new pcl::PointCloud<PointT> );
	pcl::moveFromROSMsg( cloud2, *_cloud );
	IterativeClosestPoint::setInputTarget( _cloud );
}

void IterativeClosestPointWrapper::setMaxCorrespondenceDistance(std::string distance_threshold)
{
	std_msgs::Float64 _dist = from_python<std_msgs::Float64>( distance_threshold );
	IterativeClosestPoint::setMaxCorrespondenceDistance( _dist.data );
}

std::string IterativeClosestPointWrapper::getMaxCorrespondenceDistance()
{
	std_msgs::Float64 _dist;
	_dist.data = IterativeClosestPoint::getMaxCorrespondenceDistance();
	return to_python(_dist);
}

void IterativeClosestPointWrapper::setMaximumIterations(std::string nr_iterations)
{
	std_msgs::Int32 _iters = from_python<std_msgs::Int32>( nr_iterations );
	IterativeClosestPoint::setMaximumIterations( _iters.data );
}

std::string IterativeClosestPointWrapper::getMaximumIterations()
{
	std_msgs::Int32 _iters;
	_iters.data = IterativeClosestPoint::getMaximumIterations();
	return to_python( _iters );
}

void IterativeClosestPointWrapper::setTransformationEpsilon(std::string epsilon) {
	std_msgs::Float64 _epsilon = from_python<std_msgs::Float64>( epsilon );
	IterativeClosestPoint::setTransformationEpsilon( _epsilon.data );
}

void IterativeClosestPointWrapper::setEuclideanFitnessEpsilon(std::string epsilon) {
	std_msgs::Float64 _epsilon = from_python<std_msgs::Float64>( epsilon );
	IterativeClosestPoint::setEuclideanFitnessEpsilon( _epsilon.data );
}

std::string IterativeClosestPointWrapper::align()
{
	pcl::PointCloud<PointT> _cloud;
	sensor_msgs::PointCloud2 cloud2;
	IterativeClosestPoint::align( _cloud );
	pcl::toROSMsg( _cloud, cloud2 );
	return to_python( cloud2 );
}

BOOST_PYTHON_MODULE( _pcl_icp_wrapper_cpp ) {
	boost::python::class_<IterativeClosestPointWrapper> ( "IterativeClosestPointWrapper", boost::python::init<>())
	.def( "setInputSource", &IterativeClosestPointWrapper::setInputSource )
	.def( "setInputTarget", &IterativeClosestPointWrapper::setInputTarget )
	.def( "setMaxCorrespondenceDistance", &IterativeClosestPointWrapper::setMaxCorrespondenceDistance )
	.def( "getMaxCorrespondenceDistance", &IterativeClosestPointWrapper::getMaxCorrespondenceDistance )
	.def( "setMaximumIterations", &IterativeClosestPointWrapper::setMaximumIterations )
	.def( "getMaximumIterations", &IterativeClosestPointWrapper::getMaximumIterations )
	.def( "setTransformationEpsilon", &IterativeClosestPointWrapper::setTransformationEpsilon )
	.def( "setEuclideanFitnessEpsilon", &IterativeClosestPointWrapper::setEuclideanFitnessEpsilon )
	.def( "align", &IterativeClosestPointWrapper::align )
	;
}

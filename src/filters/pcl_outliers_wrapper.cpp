#include <pclpy/filters/outliers.h>

StatisticalOutlierRemovalWrapper::StatisticalOutlierRemovalWrapper() :
	FilterWrapper() {}

void StatisticalOutlierRemovalWrapper::setMeanK( std::string nr_k )
{
	std_msgs::Int32 _nr_k = from_python<std_msgs::Int32>( nr_k );
	filter.setMeanK( _nr_k.data );
}

void StatisticalOutlierRemovalWrapper::setStddevMulThresh( std::string stddev_mult )
{
	std_msgs::Float32 _stddev_mult =from_python<std_msgs::Float32>( stddev_mult );
	filter.setStddevMulThresh( _stddev_mult.data );
}

void StatisticalOutlierRemovalWrapper::setNegative( std::string negative )
{
	std_msgs::Bool _neg = from_python<std_msgs::Bool>( negative );
	filter.setNegative( _neg.data );
}

BOOST_PYTHON_MODULE( _pcl_outliers_wrapper_cpp ) {
	boost::python::class_<StatisticalOutlierRemovalWrapper> ( "StatisticalOutlierRemovalWrapper", boost::python::init<>())
	.def( "setInputCloud", &StatisticalOutlierRemovalWrapper::setInputCloud )
	.def( "setStddevMulThresh", &StatisticalOutlierRemovalWrapper::setStddevMulThresh )
	.def( "setMeanK", &StatisticalOutlierRemovalWrapper::setMeanK )
	.def( "setNegative", &StatisticalOutlierRemovalWrapper::setNegative )
	.def( "filter", &StatisticalOutlierRemovalWrapper::applyFilter )
	;
}

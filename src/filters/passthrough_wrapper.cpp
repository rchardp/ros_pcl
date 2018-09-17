#include <pclpy/filters/passthrough.h>

PassThroughWrapper::PassThroughWrapper() :
	FilterWrapper() {}

void PassThroughWrapper::setFilterFieldName( std::string field_name)
{
	std_msgs::String _field = from_python<std_msgs::String>( field_name );
	filter.setFilterFieldName( _field.data );
}

void PassThroughWrapper::setFilterLimits( std::string limit_min, std::string limit_max )
{
	std_msgs::Float64 _min =from_python<std_msgs::Float64>( limit_min );
	std_msgs::Float64 _max =from_python<std_msgs::Float64>( limit_max );
	filter.setFilterLimits( _min.data, _max.data );
}

void PassThroughWrapper::setNegative( std::string negative )
{
	std_msgs::Bool _neg = from_python<std_msgs::Bool>( negative );
	filter.setNegative( _neg.data );
}

BOOST_PYTHON_MODULE( _pcl_passthrough_wrapper_cpp ) {
	boost::python::class_<PassThroughWrapper> ( "PassThroughWrapper", boost::python::init<>())
	.def( "setInputCloud", &PassThroughWrapper::setInputCloud )
	.def( "setFilterFieldName", &PassThroughWrapper::setFilterFieldName )
	.def( "setFilterLimits", &PassThroughWrapper::setFilterLimits )
	.def( "setNegative", &PassThroughWrapper::setNegative )
	.def( "filter", &PassThroughWrapper::applyFilter )
	;
}

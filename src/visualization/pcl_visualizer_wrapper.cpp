#include <pclpy/definitions.h>
#include <pclpy/visualization/pcl_visualizer.h>

#include <sstream>

using pcl::visualization::PointCloudColorHandlerRGBField;

int PCLVisualizerWrapper::clouds = 0;

PCLVisualizerWrapper::PCLVisualizerWrapper() :
	PCLVisualizer() {
	PCLVisualizer::initCameraParameters();
}

std::string PCLVisualizerWrapper::createViewPort( std::string xmin, std::string ymin,
														std::string xmax, std::string ymax ) {
	std_msgs::Float64 _xmin, _ymin, _xmax, _ymax;
	std_msgs::Int32 _viewport;

	_xmin = from_python<std_msgs::Float64>( xmin );
	_ymin = from_python<std_msgs::Float64>( ymin );
	_xmax = from_python<std_msgs::Float64>( xmax );
	_ymax = from_python<std_msgs::Float64>( ymax );
	_viewport.data = 0;

	PCLVisualizer::createViewPort( _xmin.data, _ymin.data,
																 _xmax.data, _ymax.data,
																 _viewport.data );
	return to_python( _viewport );
}

std::string PCLVisualizerWrapper::addPointCloud( std::string cloud, std::string viewport )
{
	std::stringstream ss;
	std_msgs::String ret;
	bool addSuccess;
	std_msgs::Int32 _viewport;
	sensor_msgs::PointCloud2 cloud2;

	_viewport = from_python<std_msgs::Int32>( viewport );

	cloud2 = from_python<sensor_msgs::PointCloud2>( cloud );
	pcl::PointCloud<PointT>::Ptr _cloud( new pcl::PointCloud<PointT> );
	pcl::moveFromROSMsg( cloud2, *_cloud );

	PointCloudColorHandlerRGBField<PointT> rgb( _cloud );
	ss << "$" << PCLVisualizerWrapper::clouds + 1;
	addSuccess = PCLVisualizer::addPointCloud( _cloud, rgb, ss.str(),
																						_viewport.data);
	if (addSuccess) {
		++PCLVisualizerWrapper::clouds;
		ret.data = ss.str();
	}
	return to_python( ret );
}

std::string PCLVisualizerWrapper::removePointCloud( std::string id, std::string viewport ) {
	std_msgs::Bool _ret;
	std_msgs::String _id;
	std_msgs::Int32 _viewport;

	_id = from_python<std_msgs::String>( id );
	_viewport = from_python<std_msgs::Int32>( viewport );
	_ret.data = PCLVisualizer::removePointCloud( _id.data,
																							_viewport.data );

	return to_python<std_msgs::Bool>( _ret );
}

void PCLVisualizerWrapper::setBackgroundColor( std::string r, std::string g, std::string b, std::string viewport )
{
	std_msgs::UInt8 _r, _g, _b;
	std_msgs::Int32 _viewport;

	_r = from_python<std_msgs::UInt8>( r );
	_g = from_python<std_msgs::UInt8>( g );
	_b = from_python<std_msgs::UInt8>( b );
	_viewport = from_python<std_msgs::Int32>( viewport );

	PCLVisualizer::setBackgroundColor( _r.data, _g.data, _b.data,
																		_viewport.data );
}
void PCLVisualizerWrapper::setWindowName( std::string winName )
{
	std_msgs::String _wname = from_python<std_msgs::String>( winName );
	PCLVisualizer::setWindowName( _wname.data );
}

std::string PCLVisualizerWrapper::wasStopped()
{
	std_msgs::Bool _wasStopped;
	_wasStopped.data = PCLVisualizer::wasStopped();
	return to_python<std_msgs::Bool>( _wasStopped );
}

void PCLVisualizerWrapper::spinOnce(std::string time, std::string force_redraw)
{
	std_msgs::Int32 _time = from_python<std_msgs::Int32>( time );
	std_msgs::Bool _force_redraw = from_python<std_msgs::Bool>( force_redraw );
	PCLVisualizer::spinOnce( _time.data, _force_redraw.data );
}

BOOST_PYTHON_MODULE( _pcl_visualizer_wrapper_cpp ) {
	boost::python::class_<PCLVisualizerWrapper> ( "PCLVisualizerWrapper", boost::python::init<>())
	.def( "createViewPort", &PCLVisualizerWrapper::createViewPort )
	.def( "addPointCloud", &PCLVisualizerWrapper::addPointCloud )
	.def( "removePointCloud", &PCLVisualizerWrapper::removePointCloud )
	.def( "setBackgroundColor", &PCLVisualizerWrapper::setBackgroundColor )
	.def( "setWindowName", &PCLVisualizerWrapper::setWindowName )
	.def( "spin", &PCLVisualizerWrapper::spin )
	.def( "spinOnce", &PCLVisualizerWrapper::spinOnce )
	.def( "wasStopped", &PCLVisualizerWrapper::wasStopped )
	.def( "close", &PCLVisualizerWrapper::close )
	;
}

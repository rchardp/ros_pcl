#include <pclpy/definitions.h>

#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <sstream>

using pcl::visualization::PCLVisualizer;
using pcl::visualization::PointCloudColorHandlerRGBField;

class PCLVisualizerWrapper : public PCLVisualizer
{

public:
	static int clouds;
	PCLVisualizerWrapper() : PCLVisualizer() {}

	std::string addPointCloud( std::string cloud )
	{
		std::stringstream ss;
		std_msgs::String ret;
		bool addSuccess;
		sensor_msgs::PointCloud2 cloud2 = from_python<sensor_msgs::PointCloud2>( cloud );
		pcl::PointCloud<PointT>::Ptr _cloud( new pcl::PointCloud<PointT> );
		pcl::moveFromROSMsg( cloud2, *_cloud );
		PointCloudColorHandlerRGBField<PointT> rgb( _cloud );
		ss << "$" << PCLVisualizerWrapper::clouds + 1;
		addSuccess = PCLVisualizer::addPointCloud( _cloud, rgb, ss.str() );
		if (addSuccess) {
			++PCLVisualizerWrapper::clouds;
			ret.data = ss.str();
		}
		//ret.data = ss.str();
		return to_python( ret );
	}

	std::string removePointCloud( std::string id ) {
		std_msgs::Bool _ret;
		std_msgs::String _id;
		_id = from_python<std_msgs::String>( id );
		_ret.data = PCLVisualizer::removePointCloud( _id.data );
		return to_python<std_msgs::Bool>( _ret );
	}

	void setBackgroundColor( std::string r, std::string g, std::string b )
	{
		std_msgs::UInt8 _r, _g, _b;
		_r = from_python<std_msgs::UInt8>( r );
		_g = from_python<std_msgs::UInt8>( g );
		_b = from_python<std_msgs::UInt8>( b );
		PCLVisualizer::setBackgroundColor( _r.data, _g.data, _b.data );
	}
	void setWindowName( std::string winName )
	{
		std_msgs::String _wname = from_python<std_msgs::String>( winName );
		PCLVisualizer::setWindowName( _wname.data );
	}

	std::string wasStopped()
	{
		std_msgs::Bool _wasStopped;
		_wasStopped.data = PCLVisualizer::wasStopped();
		return to_python<std_msgs::Bool>( _wasStopped );
	}

	void spinOnce(std::string time, std::string force_redraw)
	{
		std_msgs::Int32 _time = from_python<std_msgs::Int32>( time );
		std_msgs::Bool _force_redraw = from_python<std_msgs::Bool>( force_redraw );
		PCLVisualizer::spinOnce( _time.data, _force_redraw.data );
	}

};

int PCLVisualizerWrapper::clouds = 0;

BOOST_PYTHON_MODULE( _pcl_visualizer_wrapper_cpp ) {
	boost::python::class_<PCLVisualizerWrapper> ( "PCLVisualizerWrapper", boost::python::init<>())
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

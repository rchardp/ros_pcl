#include <boost/python.hpp>
#include <pclpy/wrapper.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>

class PcdIOWrapper {

public:
	PcdIOWrapper() {} // empty constructor

	std::string loadPCDFile( std::string fileName ) {
		sensor_msgs::PointCloud2 cloud2; // pointcloud2
		std_msgs::String t_filename =
			from_python<std_msgs::String>( fileName );
		int ret =
			pcl::io::loadPCDFile( t_filename.data, cloud2 );
		if ( ret == -1 )
		{
			PCL_ERROR( "Could not load file: '%s'\n", t_filename.data.c_str() );
		}

		return to_python( cloud2 );
	}

	std::string savePCDFile( std::string fileName, std::string cloud ) {
		sensor_msgs::PointCloud2 cloud2 =
			from_python<sensor_msgs::PointCloud2>( cloud );
		std_msgs::String t_filename =
			from_python<std_msgs::String>( fileName );
		int ret =
			pcl::io::savePCDFile( t_filename.data, cloud2 );
		
		std_msgs::Bool b_ret;
			b_ret.data = (ret != -1 );

		return to_python( b_ret );
	}
};

BOOST_PYTHON_MODULE( _pcd_io_wrapper_cpp ) {
	boost::python::class_<PcdIOWrapper> ( "PcdIOWrapper", boost::python::init<>())
	.def( "savePCDFile", &PcdIOWrapper::savePCDFile )
	.def( "loadPCDFile", &PcdIOWrapper::loadPCDFile )
	;
}

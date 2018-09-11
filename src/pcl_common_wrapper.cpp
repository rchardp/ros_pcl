#include <pclpy/definitions.h>

class PointCloudWrapper
{
public:
	PointCloudWrapper() {}

	std::string concatenatePointCloud(std::string cloud1, std::string cloud2) {
		sensor_msgs::PointCloud2 sCloud1 = from_python<sensor_msgs::PointCloud2>( cloud1 ),
														 sCloud2 = from_python<sensor_msgs::PointCloud2>( cloud2 ),
														 sCloudOut;
		pcl::concatenatePointCloud ( sCloud1, sCloud2, sCloudOut );

		return to_python( sCloudOut );
	}
};

BOOST_PYTHON_MODULE( _pcl_common_wrapper_cpp ) {
	boost::python::class_<PointCloudWrapper> ( "PointCloudWrapper", boost::python::init<>())
	.def( "concatenatePointCloud", &PointCloudWrapper::concatenatePointCloud )
	;
}

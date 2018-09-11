
#include <pclpy/definitions.h>
#include <pclpy/filterWrapper.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::VoxelGrid<PointT> VGFilter;

class VoxelGridWrapper : public FilterWrapper< VGFilter >
{
public:
	VoxelGridWrapper() : FilterWrapper() {}

	void setLeafSize( std::string lx, std::string ly, std::string lz )
	{
		std_msgs::Float32 _lx = from_python<std_msgs::Float32>( lx );
		std_msgs::Float32 _ly = from_python<std_msgs::Float32>( ly );
		std_msgs::Float32 _lz = from_python<std_msgs::Float32>( lz );
		filter.setLeafSize( _lx.data, _ly.data, _lz.data );
	}

};

BOOST_PYTHON_MODULE( _pcl_voxel_grid_wrapper_cpp ) {
	boost::python::class_<VoxelGridWrapper> ( "VoxelGridWrapper", boost::python::init<>())
	.def( "setInputCloud", &VoxelGridWrapper::setInputCloud )
	.def( "setLeafSize", &VoxelGridWrapper::setLeafSize )
	.def( "filter", &VoxelGridWrapper::applyFilter )
	;
}

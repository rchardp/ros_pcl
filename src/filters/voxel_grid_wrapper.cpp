#include <pclpy/filters/voxel_grid.h>

VoxelGridWrapper::VoxelGridWrapper() :
	FilterWrapper() {}

void VoxelGridWrapper::setLeafSize( std::string lx, std::string ly, std::string lz )
{
	std_msgs::Float64 _lx = from_python<std_msgs::Float64>( lx );
	std_msgs::Float64 _ly = from_python<std_msgs::Float64>( ly );
	std_msgs::Float64 _lz = from_python<std_msgs::Float64>( lz );
	filter.setLeafSize( _lx.data, _ly.data, _lz.data );
}

BOOST_PYTHON_MODULE( _pcl_voxel_grid_wrapper_cpp ) {
	boost::python::class_<VoxelGridWrapper> ( "VoxelGridWrapper", boost::python::init<>())
	.def( "setInputCloud", &VoxelGridWrapper::setInputCloud )
	.def( "setLeafSize", &VoxelGridWrapper::setLeafSize )
	.def( "filter", &VoxelGridWrapper::applyFilter )
	;
}

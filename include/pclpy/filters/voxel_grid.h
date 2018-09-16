#ifndef _PCLPY_FILTER_VOXEL_GRID_H
#define _PCLPY_FILTER_VOXEL_GRID_H

#include <pclpy/definitions.h>
#include <pclpy/filters/filterWrapper.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::VoxelGrid<PointT> VGFilter;

class VoxelGridWrapper : public FilterWrapper< VGFilter >
{
public:
	VoxelGridWrapper();
	void setLeafSize( std::string lx, std::string ly, std::string lz );
};

#endif // _PCLPY_FILTER_VOXEL_GRID_H

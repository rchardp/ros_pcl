#ifndef FILTER_WRAPPER_H
#define FILTER_WRAPPER_H

#include <pclpy/definitions.h>

template<class T>
class FilterWrapper {
public:
	FilterWrapper() {}
	void setInputCloud( std::string cloud )
	{
		sensor_msgs::PointCloud2 cloud2 = from_python<sensor_msgs::PointCloud2>( cloud );
		pcl::PointCloud<PointT>::Ptr _cloud( new pcl::PointCloud<PointT> );
		pcl::moveFromROSMsg( cloud2, *_cloud );
		filter.setInputCloud( _cloud );
	}
	std::string applyFilter()
	{
		pcl::PointCloud<PointT> _cloud;
		sensor_msgs::PointCloud2 cloud2;
		filter.filter( _cloud );
		pcl::toROSMsg( _cloud, cloud2 );
		return to_python( cloud2 );
	}
protected:
	T filter;
};

#endif // FILTER_WRAPPER_H

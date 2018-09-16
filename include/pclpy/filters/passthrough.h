#ifndef _PCLPY_FILTER_PASSTHROUGH_H
#define _PCLPY_FILTER_PASSTHROUGH_H

#include <pclpy/definitions.h>
#include <pclpy/filters/filterWrapper.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PassThrough<PointT> PassFilter;

class PassThroughWrapper : public FilterWrapper< PassFilter >
{
public:
	PassThroughWrapper();
	void setFilterFieldName( std::string field_name);
	void setFilterLimits( std::string limit_min, std::string limit_max );
	void setNegative( std::string negative );
};

#endif // _PCLPY_FILTER_PASSTHROUGH_H

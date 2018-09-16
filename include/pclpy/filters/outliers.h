#ifndef _PCLPY_FILTER_OUTLIER_H
#define _PCLPY_FILTER_OUTLIER_H

#include <pclpy/definitions.h>
#include <pclpy/filters/filterWrapper.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::StatisticalOutlierRemoval<PointT> OutlierFilter;

class StatisticalOutlierRemovalWrapper : public FilterWrapper< OutlierFilter >
{
public:
	StatisticalOutlierRemovalWrapper();
	void setMeanK( std::string nr_k );
	void setStddevMulThresh( std::string stddev_mult );
	void setNegative( std::string negative );
};

#endif // _PCLPY_FILTER_OUTLIER_H

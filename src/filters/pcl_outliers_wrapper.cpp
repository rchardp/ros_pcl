
#include <pclpy/definitions.h>
#include <pclpy/filterWrapper.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::StatisticalOutlierRemoval<PointT> OutlierFilter;

class StatisticalOutlierRemovalWrapper : public FilterWrapper< OutlierFilter >
{
public:
	StatisticalOutlierRemovalWrapper() : FilterWrapper() {}

	void setMeanK( std::string nr_k )
	{
		std_msgs::Int32 _nr_k = from_python<std_msgs::Int32>( nr_k );
		filter.setMeanK( _nr_k.data );
	}

	void setStddevMulThresh( std::string stddev_mult )
	{
		std_msgs::Float32 _stddev_mult =from_python<std_msgs::Float32>( stddev_mult );
		filter.setStddevMulThresh( _stddev_mult.data );
	}

	void setNegative( std::string negative )
	{
		std_msgs::Bool _neg = from_python<std_msgs::Bool>( negative );
		filter.setNegative( _neg.data );
	}

};

BOOST_PYTHON_MODULE( _pcl_outliers_wrapper_cpp ) {
	boost::python::class_<StatisticalOutlierRemovalWrapper> ( "StatisticalOutlierRemovalWrapper", boost::python::init<>())
	.def( "setInputCloud", &StatisticalOutlierRemovalWrapper::setInputCloud )
	.def( "setStddevMulThresh", &StatisticalOutlierRemovalWrapper::setStddevMulThresh )
	.def( "setMeanK", &StatisticalOutlierRemovalWrapper::setMeanK )
	.def( "setNegative", &StatisticalOutlierRemovalWrapper::setNegative )
	.def( "filter", &StatisticalOutlierRemovalWrapper::applyFilter )
	;
}

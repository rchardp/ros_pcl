#ifndef _PCLPY_ICP_H
#define _PCLPY_ICP_H

#include <pclpy/definitions.h>
#include <pcl/registration/icp.h>

class IterativeClosestPointWrapper : public pcl::IterativeClosestPoint<PointT, PointT>
{
public:
	IterativeClosestPointWrapper();
	void setInputSource(std::string cloud);
	void setInputTarget(std::string cloud);
	void setMaxCorrespondenceDistance(std::string distance_threshold);
	std::string getMaxCorrespondenceDistance();
	void setMaximumIterations(std::string nr_iterations);
	std::string getMaximumIterations();
	void setTransformationEpsilon(std::string epsilon);
	void setEuclideanFitnessEpsilon(std::string epsilon);
	std::string align();
};

#endif // _PCLPY_ICP_H

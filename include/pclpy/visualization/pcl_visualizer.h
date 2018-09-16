#ifndef _PCLPY_PCL_VISUALIZER_H
#define _PCLPY_PCL_VISUALIZER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <string>

using pcl::visualization::PCLVisualizer;

class PCLVisualizerWrapper : public PCLVisualizer
{
public:
	static int clouds;
	PCLVisualizerWrapper();
	std::string createViewPort( std::string xmin, std::string ymin,
															std::string xmax, std::string ymax );
	std::string addPointCloud( std::string cloud, std::string viewport );
	std::string removePointCloud( std::string id, std::string viewport );
	void setBackgroundColor( std::string r, std::string g, std::string b, std::string viewport );
	void setWindowName( std::string winName );
	std::string wasStopped();
	void spinOnce(std::string time, std::string force_redraw);
};

#endif // _PCLPY_PCL_VISUALIZER_H

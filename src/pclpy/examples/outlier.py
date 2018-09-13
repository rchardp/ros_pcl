from pclpy.pcdio import loadPCDFile
from pclpy.visualization import PCLVisualizer
from pclpy.filters import StatisticalOutlierRemoval

cloud = loadPCDFile('inputCloud.pcd')

sor = StatisticalOutlierRemoval()
sor.setInputCloud(cloud)
sor.setMeanK(100)
sor.setStddevMulThresh(1.0)

filtered_cloud = _sor.filter()

vis = PCLVisualizer()

vis.addPointCloud(filtered_cloud)
vis.spin()

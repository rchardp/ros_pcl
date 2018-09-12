from pclpy.pcdio import loadPCDFile
from pclpy.visualization import PCLVisualizer
from pclpy.filters import StatisticalOutlierRemoval

cloud = loadPCDFile('inputCloud0.pcd')

_sor = StatisticalOutlierRemoval()
_sor.setInputCloud(cloud)
_sor.setMeanK(100)
_sor.setStddevMulThresh(1.0)

fcloud = _sor.filter()

vis = PCLVisualizer()

vis.addPointCloud(fcloud)
vis.spin()

from pclpy.visualization import PCLVisualizer
from pclpy.pcdio import loadPCDFile

vis = PCLVisualizer()

cloud = loadPCDFile('inputCloud.pcd')

vis.addPointCloud(cloud)

vis.spin()

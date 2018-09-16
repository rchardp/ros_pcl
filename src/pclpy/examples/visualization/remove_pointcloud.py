from pclpy.visualization import PCLVisualizer
from pclpy.pcdio import loadPCDFile

vis = PCLVisualizer()

cloud1 = loadPCDFile('inputCloud1.pcd')
cloud2 = loadPCDFile('inputCloud2.pcd')

id1 = vis.addPointCloud(cloud1)
id2 = vis.addPointCloud(cloud2)

vis.spin()

vis.removePointCloud(id2)

vis.spin()

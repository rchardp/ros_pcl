from pclpy.visualization import PCLVisualizer
from pclpy.pcdio import loadPCDFile

vis = PCLVisualizer()
vis.setBackgroundColor(59, 156, 150)
vis.setWindowName('My custom title')

cloud = loadPCDFile('inputCloud1.pcd')

vis.addPointCloud(cloud)

vis.spin()

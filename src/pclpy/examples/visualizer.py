from pclpy.visualization import PCLVisualizer

vis = PCLVisualizer()

cloud = loadPCDFile('inputCloud.pcd')

vis.addPointCloud(cloud)

vis.spin()

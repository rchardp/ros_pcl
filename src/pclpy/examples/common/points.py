import pclpy

cloud = loadPCDFile('inputCloud.pcd')

for point in cloud.points:
	print( point )

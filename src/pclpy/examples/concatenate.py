import pclpy

cloud1 = loadPCDFile('inputCloud1.pcd')
cloud2 = loadPCDFile('inputCloud2.pcd')

clouds = cloud1 + cloud2

print(cloud1, cloud2, clouds, sep='\n')

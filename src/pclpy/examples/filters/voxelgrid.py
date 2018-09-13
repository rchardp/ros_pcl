from __future__ import print_function
from pclpy.pcdio import loadPCDFile
from pclpy.filters import VoxelGrid

cloud = loadPCDFile('inputCloud.pcd')

print( 'Points before filtering: {} data points {}'.format(\
        cloud.width * cloud.height, cloud.field_names ) )

vg = VoxelGrid()
vg.setInputCloud(cloud)
vg.setLeafSize(0.01, 0.01, 0.01)

filtered_cloud = vg.filter()

print( 'Points after filtering: {} data points {}'.format(\
        cloud.width * cloud.height, cloud.field_names ) )

from pclpy.visualization import PCLVisualizer
from pclpy.filters import VoxelGrid
from pclpy.pcdio import loadPCDFile

i_cloud = loadPCDFile('cloud0.pcd')

vg = VoxelGrid()
vg.setInputCloud(i_cloud)
vg.setLeafSize(.01, .01, .01)
r_cloud = vg.filter()

vis = PCLVisualizer()

v1 = vis.createViewPort( 0., 0., .5, 1. )
v2 = vis.createViewPort( .5, 0., 1., 1. )

vis.setBackgroundColor( 255, 0, 0, v1 )
vis.setBackgroundColor( 0, 255, 0, v2 )

vis.addPointCloud( i_cloud, v1 )
vis.addPointCloud( r_cloud, v2 )

vis.spin()

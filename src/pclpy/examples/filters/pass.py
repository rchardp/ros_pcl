from pclpy.pcdio import loadPCDFile
from pclpy.visualization import PCLVisualizer
from pclpy.filters import PassThrough

cloud = loadPCDFile('inputCloud.pcd')

_pass = PassThrough()
_pass.setInputCloud(cloud)
_pass.setFilterFieldName('x')
_pass.setFilterLimits( 0.0, 1.5 )

filtered_cloud = _pass.filter()

vis = PCLVisualizer()

vis.addPointCloud(filtered_cloud)
vis.spin()

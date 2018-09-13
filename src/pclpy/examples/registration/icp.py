from pclpy.pcdio import loadPCDFile
from pclpy.visualization import PCLVisualizer
from pclpy.filters import PassThrough
from pclpy.registration import IterativeClosestPoint

pcd_files = [ 'inputCloud0', 'inputCloud1' ]

clouds = [ loadPCDFile(x+'.pcd') for x in pcd_files ]

_pass = PassThrough()
_pass.setFilterFieldName('z')
_pass.setFilterLimits( 0.0, 5.0 )

# filtrar todas
for i in range(len(clouds)):
	_pass.setInputCloud(clouds[i])
	clouds[i] = _pass.filter()

icp = IterativeClosestPoint()

vis = PCLVisualizer()

t_cloud = clouds[0]
vis.addPointCloud(t_cloud)

for i in range( 1, len(clouds) ):
	icp.setInputSource( clouds[i] )
	icp.setInputTarget( t_cloud )
	t_cloud = icp.align()
	vis.addPointCloud( t_cloud )
vis.spin()

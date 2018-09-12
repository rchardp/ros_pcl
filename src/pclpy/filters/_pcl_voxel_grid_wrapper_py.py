import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from .._wrapper import pyWrapper

from pclpy._pcl_voxel_grid_wrapper_cpp import VoxelGridWrapper

class VoxelGrid(object):
	def __init__(self):
		self._voxel_grid = VoxelGridWrapper()
		self._to_cpp = pyWrapper._to_cpp
		self._from_cpp = pyWrapper._from_cpp

	def setInputCloud(self, cloud):
		'''Set the input cloud
		Parameters
		----------
		- cloud : cloud to filter
		'''
		if not isinstance( cloud, PointCloud2 ):
			rospy.ROSException( 'cloud is not a PointCloud2' )

		_cloud = self._to_cpp( cloud )

		self._voxel_grid.setInputCloud( _cloud )

	def setLeafSize(self, lx, ly, lz):
		'''Set the voxel grid leaf size
		Parameters
		----------
		- lx : leaf size for X
		- ly : leaf size for Y
		- lz : leaf size for Z
		'''
		if not isinstance( lx, float ):
			rospy.ROSException( 'lx is not float' )
		if not isinstance( ly, float ):
			rospy.ROSException( 'ly is not float' )
		if not isinstance( lz, float ):
			rospy.ROSException( 'lz is not float' )

		_lx = self._to_cpp( Float32(lx) )
		_ly = self._to_cpp( Float32(ly) )
		_lz = self._to_cpp( Float32(lz) )
		self._voxel_grid.setLeafSize( _lx, _ly, _lz )

	def filter(self):
		'''Calls the filtering method
		Returns the filtered dataset
		'''
		_cloud = self._from_cpp(self._voxel_grid.filter(), PointCloud2)
		return _cloud

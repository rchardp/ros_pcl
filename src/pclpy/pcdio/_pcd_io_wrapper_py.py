import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Bool
from .._wrapper import pyWrapper

from pcl_py._pcd_io_wrapper_cpp import PcdIOWrapper

class PcdIO(object):
	def __init__(self):
		self._pcd_io = PcdIOWrapper()
		self._to_cpp = pyWrapper._to_cpp
		self._from_cpp = pyWrapper._from_cpp

	def savePCDFile(self, filename, cloud):
		'''Save PCD file
		Returns the operation success (True, False)
		Parameters
		----------
		- filename: filename to save the pcd file
		- cloud:    cloud to save
		'''
		if not isinstance( filename, str ):
			rospy.ROSException( 'filename is not str' )

		if not isinstance( cloud, PointCloud2 ):
			rospy.ROSException( 'cloud is not a PointCloud2' )

		str_cloud = self._to_cpp( cloud )
		str_filename = self._to_cpp( String(filename) )
		s_ret = self._pcd_io.savePCDFile( str_filename, str_cloud )
		ret = self._from_cpp( s_ret, Bool )
		return ret.data

	def loadPCDFile(self, filename):
		'''Load PCD file
		Returns the resultant PointCloud2
		Parameters
		----------
		- filename: the name of the file to load
		'''
		if not isinstance( filename, str ):
			rospy.ROSException( 'filename is not str' )

		str_filename = self._to_cpp( String(filename) )
		str_cloud = self._pcd_io.loadPCDFile( str_filename )
		return self._from_cpp( str_cloud, PointCloud2 )

_pcdio = PcdIO()
savePCDFile = _pcdio.loadPCDFile
loadPCDFile = _pcdio.loadPCDFile

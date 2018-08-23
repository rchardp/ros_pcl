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
		Parameters
		----------
		- filename: filename to save the pcd file
		- cloud:    cloud to save
		'''
		if not isinstance( filename, str ):
			rospy.ROSException( 'Argument1 is not str' )

		if not isinstance( cloud, PointCloud2 ):
			rospy.ROSException( 'Argument2 is not PointCloud2' )

		str_cloud = self._to_cpp( cloud )
		str_filename = self._to_cpp( String(filename) )
		s_ret = self._pcd_io.savePCDFile( str_filename, str_cloud )
		ret = self._from_cpp( s_ret, Bool )
		return ret.data

	def loadPCDFile(self, filename):
		'''Load PCD file
		Parameters
		----------
		- filename: filename to read the pcd file
		'''
		if not isinstance( filename, str ):
			rospy.ROSException( 'Argument1 is not str' )

		str_filename = self._to_cpp( String(filename) )
		str_cloud = self._pcd_io.loadPCDFile( str_filename )
		return self._from_cpp( str_cloud, PointCloud2 )

pcdio = PcdIO()
savePCDFile = pcdio.loadPCDFile
loadPCDFile = pcdio.loadPCDFile

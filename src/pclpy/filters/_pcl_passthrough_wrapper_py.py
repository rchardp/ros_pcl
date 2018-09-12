import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Bool, String
from .._wrapper import pyWrapper

from pclpy._pcl_passthrough_wrapper_cpp import PassThroughWrapper

class PassThrough(object):
	def __init__(self):
		self._passthrough = PassThroughWrapper()
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

		self._passthrough.setInputCloud( _cloud )

	def setFilterFieldName( self, field_name ):
		'''Set the field to be used for filtering data
		Parameters
		----------
		- field_name : The name of the field
		'''
		if not isinstance( field_name, str ):
			rospy.ROSException( 'field_name is not str' )
		_field = self._to_cpp( String(field_name) )
		self._passthrough.setFilterFieldName( _field )

	def setFilterLimits( self, limit_min, limit_max ):
		'''Set the numerical limits for the field
		Parameters
		----------
		- limit_min : The minimum allowed field value
		- limit_max : The maximum allowed field value
		'''
		if not isinstance( limit_min, float ):
			rospy.ROSException( 'limit_min is not float' )
		if not isinstance( limit_max, float ):
			rospy.ROSException( 'limit_max is not float' )
		_min = self._to_cpp( Float32(limit_min) )
		_max = self._to_cpp( Float32(limit_max) )
		self._passthrough.setFilterLimits( _min, _max )

	def setNegative( self, negative ):
		'''Set whether the regular (or inverted) conditions for points filtering should apply
		Parameters
		----------
		- negative : false = normal filter behavior (default), true = inverted behavior
		'''
		if not isinstance( negative, bool ):
			rospy.ROSException( 'negative is not bool' )
		_neg = self._to_cpp( Bool(negative) )
		self._passthrough.setNegative( _neg )

	def filter(self):
		'''Calls the filtering method
		Returns the filtered dataset
		'''
		_cloud = self._from_cpp(self._passthrough.filter(), PointCloud2)
		return _cloud

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Bool, Int32
from .._wrapper import pyWrapper

from pclpy._pcl_outliers_wrapper_cpp import StatisticalOutlierRemovalWrapper

class StatisticalOutlierRemoval(object):
	def __init__(self):
		self._outliers = StatisticalOutlierRemovalWrapper()
		self._to_cpp = pyWrapper._to_cpp
		self._from_cpp = pyWrapper._from_cpp

	def setInputCloud(self, cloud):
		'''Set the input cloud
		Parameters
		----------
		- cloud : cloud to filter
		'''
		if not isinstance( cloud, PointCloud2 ):
			rospy.ROSException( 'Argument1 is not PointCloud2' )

		_cloud = self._to_cpp( cloud )

		self._outliers.setInputCloud( _cloud )

	def setMeanK( self, nr_k ):
		'''Set the number of nearest neighbors to use for mean distance estimation
		Parameters
		----------
		- nr_k : The number of points to use for mean distance estimation
		'''
		if not isinstance( nr_k, int ):
			rospy.ROSException( 'Argument1 is not int' )
		_nr_k = self._to_cpp( Int32(nr_k) )
		self._outliers.setMeanK( _nr_k )

	def setStddevMulThresh( self, stddev_mult):
		'''Set the standard deviation multiplier for the distance threshold calculation
		Parameters
		----------
		- stddev_mult : The standard deviation multiplier
		'''
		if not isinstance( stddev_mult, float ):
			rospy.ROSException( 'Argument1 is not float' )

		_stddev_mult = self._to_cpp( Float32(stddev_mult) )
		self._outliers.setStddevMulThresh( _stddev_mult )

	def setNegative( self, negative ):
		'''Set whether the regular (or inverted) conditions for points filtering should apply
		Parameters
		----------
		- negative : false = normal filter behavior (default), true = inverted behavior
		'''
		if not isinstance( negative, bool ):
			rospy.ROSException( 'Argument1 is not bool' )
		_neg = self._to_cpp( Bool(negative) )
		self._outliers.setNegative( _neg )

	def filter(self):
		'''Calls the filtering method and returns the filtered dataset
		'''
		_cloud = self._from_cpp(self._outliers.filter(), PointCloud2)
		return _cloud

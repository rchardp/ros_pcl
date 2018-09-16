import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, Int32
from .._wrapper import pyWrapper

from pclpy._pcl_icp_wrapper_cpp import IterativeClosestPointWrapper

class IterativeClosestPoint(object):
	def __init__(self, wname=''):
		self._icp = IterativeClosestPointWrapper()
		self._to_cpp = pyWrapper._to_cpp
		self._from_cpp = pyWrapper._from_cpp

	def setInputSource(self, cloud):
		'''Provide a pointer to the input source
		Parameters
		----------
		- cloud : the input cloud source
		'''
		if not isinstance( cloud, PointCloud2 ):
			rospy.ROSException( 'cloud is not PointCloud2' )

		s_cloud = self._to_cpp(cloud)
		self._icp.setInputSource( s_cloud )

	def setInputTarget(self, cloud):
		'''Provide a pointer to the input target
		Parameters
		----------
		- cloud : the input point cloud target
		'''
		if not isinstance( cloud, PointCloud2 ):
			rospy.ROSException( 'cloud is not PointCloud2' )

		s_cloud = self._to_cpp(cloud)
		self._icp.setInputTarget( s_cloud )

	def setMaxCorrespondenceDistance(self, distance_threshold):
		'''Set the maximum distance threshold between two correspondent points in source <-> target
		Parameters
		----------
		- distance_threshold : the maximum distance threshold between a point and its nearest
							...neighbor correspondent in order to be considered in the alignment
							   process
		'''
		if not isinstance(distance_threshold, float):
			rospy.ROSException( 'distance_threshold is not float' )
		s_dist = self._to_cpp( Float64(distance_threshold) )
		self._icp.setMaxCorrespondenceDistance( s_dist )

	def getMaxCorrespondenceDistance(self):
		'''Get the maximum distance threshold between two correspondent points in source <-> target
		'''
		_dist = self._from_cpp( self._icp.getMaxCorrespondenceDistance(), Float64 )
		return _dist.data

	def setMaximumIterations(self, nr_iterations):
		'''Set the maximum number of iterations the internal optimization should run for
		Parameters
		----------
		- nr_iterations : the maximum number of iterations the internal optimization should run for
		'''
		if not isinstance(nr_iterations, int):
			rospy.ROSException( 'nr_iterations is not int' )
		s_iters = self._to_cpp( Int32(nr_iterations) )
		self._icp.setMaximumIterations(s_iters)

	def getMaximumIterations(self):
		'''Get the maximum number of iterations the internal optimization should run for, as set by the user
		'''
		_iters = self._from_cpp( self._icp.getMaximumIterations(), Int32 )
		return _iters.data

	def setTransformationEpsilon(self, epsilon):
		'''Set the transformation epsilon (maximum allowable translation squared difference between two consecutive transformations)
		Parameters
		----------
		- epsilon : the transformation epsilon in order for an optimization to be considered as having converged to the final solution
		'''
		if not isinstance(epsilon, float):
			rospy.ROSException( 'epsilon is not float' )
		s_epsilon = self._to_cpp( Float64(epsilon) )
		self._icp.setTransformationEpsilon(s_epsilon)

	def setEuclideanFitnessEpsilon(self, epsilon):
		'''Set the maximum allowed Euclidean error between two consecutive steps in the ICP loop
		Parameters
		----------
		- epsilon : the maximum allowed distance error before the algorithm will be considered to have converged
		'''
		if not isinstance(epsilon, float):
			rospy.ROSException( 'epsilon is not float' )
		s_epsilon = self._to_cpp( Float64(epsilon) )
		self._icp.setEuclideanFitnessEpsilon(s_epsilon)

	def align(self):
		'''Perform the alignment
		'''
		_cloud = self._from_cpp( self._icp.align(), PointCloud2 )
		return _cloud

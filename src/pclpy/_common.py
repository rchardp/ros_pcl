from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from collections import namedtuple

from pclpy._pcl_common_wrapper_cpp import PointCloudWrapper
from ._wrapper import pyWrapper

_pclWrapper = PointCloudWrapper()

@property
def _points(self):
	'''Get the data array where all points are stored
	'''
	field_names = [ f.name for f in self.fields ]
	Point = namedtuple("Point", field_names)
	return [Point._make(l) for l in pc2.read_points(self)]

def _concatenatePointCloud( cloud1, cloud2 ):
	'''Concatenate two PointCloud2 and return the resultant point cloud
	Parameters
	----------
	- cloud1 : the first input point cloud dataset
	- cloud2 : the second input point cloud dataset
	'''
	if not isinstance( cloud1, PointCloud2 ):
		rospy.ROSException( 'Argument1 is not PointCloud2' )
	if not isinstance( cloud2, PointCloud2 ):
		rospy.ROSException( 'Argument2 is not PointCloud2' )

	_cloud1 = pyWrapper._to_cpp( cloud1 )
	_cloud2 = pyWrapper._to_cpp( cloud2 )

	_cloudOut = _pclWrapper.concatenatePointCloud( _cloud1, _cloud2 )

	return pyWrapper._from_cpp( _cloudOut, PointCloud2 )

PointCloud2.points = _points
PointCloud2.__add__ = _concatenatePointCloud

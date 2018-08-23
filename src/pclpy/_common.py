from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from collections import namedtuple

@property
def _points(self):
	field_names = [ f.name for f in self.fields ]
	Point = namedtuple("Point", field_names)
	return [Point._make(l) for l in pc2.read_points(self)]

PointCloud2.points = _points

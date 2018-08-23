from StringIO import StringIO
from std_msgs.msg import String


class pyWrapper(object):
	@staticmethod
	def _to_cpp(msg):
		'''Return a serialized string from a ROS message
		Parameters
		----------
		- msg: a ROS message instance.
		'''
		buf = StringIO()
		msg.serialize(buf)
		return buf.getvalue()
	
	@staticmethod
	def _from_cpp(str_msg, cls):
		'''Return a ROS message from a serialized string
		Parameters
		----------
		- str_msg: str, serialized message
		- cls: ROS message class, e.g.sensor_msgs.msg.PointCloud2
		'''
		msg = cls()
		return msg.deserialize(str_msg)


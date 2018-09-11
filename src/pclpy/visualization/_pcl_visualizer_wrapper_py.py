import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, UInt8, Int32, Bool
from .._wrapper import pyWrapper

from pclpy._pcl_visualizer_wrapper_cpp import PCLVisualizerWrapper

class PCLVisualizer(object):
	def __init__(self, wname=''):
		self._viewer = PCLVisualizerWrapper()
		self._to_cpp = pyWrapper._to_cpp
		self._from_cpp = pyWrapper._from_cpp
		if wname: self.setWindowName(wname)

	def setBackgroundColor(self, r, g, b):
		'''Set background color
		Parameters
		----------
		- r : red channel
		- g : green channel
		- b : blue channel
		'''
		if not (isinstance( r, int ) and 0 <= r <= 255):
			rospy.ROSException( 'r is not a 8 bytes uint' )

		if not (isinstance( g, int ) and 0 <= g <= 255):
			rospy.ROSException( 'g is not a 8 bytes uint' )

		if not (isinstance( b, int ) and 0 <= b <= 255):
			rospy.ROSException( 'b is not a 8 bytes uint' )

		_r = self._to_cpp( UInt8(r) )
		_g = self._to_cpp( UInt8(g) )
		_b = self._to_cpp( UInt8(b) )

		self._viewer.setBackgroundColor( _r, _g, _b )

	def setWindowName(self, winName):
		'''Set Window Name
		Parameters
		----------
		- winName: window name
		'''
		if not isinstance( winName, str ):
			rospy.ROSException( 'winName is not str' )

		_wname = self._to_cpp( String(winName) )
		self._viewer.setWindowName( _wname )

	def addPointCloud(self, cloud):
		'''Add Point Cloud to Screen
		Returns a cloud_id
		Parameters
		----------
		- cloud : PointCloud2 to display
		'''
		if not isinstance( cloud, PointCloud2 ):
			rospy.ROSException( 'cloud is not a PointCloud2' )
		s_cloud = self._to_cpp( cloud )
		s_ret = self._viewer.addPointCloud( s_cloud )
		ret = self._from_cpp(s_ret, String)
		return ret.data

	def removePointCloud(self, cloud_id):
		'''Remove Point Cloud from Screen
		Returns the operation success
		Parameters
		----------
		- cloud_id : the point cloud object id (taken from addPointCloud)
		'''
		if not isinstance( cloud_id, str ):
			rospy.ROSException( 'cloud_id is not str' )
		_cloud_id = self._to_cpp( String(cloud_id) )
		ret = self._viewer.removePointCloud(_cloud_id)
		_ret = self._from_cpp( ret, Bool )
		return _ret.data

	def spin(self):
		'''Spin Visualizer
		Calls the interactor and runs an internal loop
		'''
		self._viewer.spin()

	def spinOnce(self, time=1, force_redraw=False):
		'''Spin Once Visualizer
		Calls the interactor and updates the screen once time
		Parameters
		----------
		- time : How long (in ms) should the visualization run
		- force_redraw : force the visualizer to redraw the screen
		'''
		if not (isinstance( time, int ) and time >= 0 ):
			rospy.ROSException( 'time is not uint' )
		if not isinstance( force_redraw, bool ):
			rospy.ROSException( 'force_redraw is not bool' )

		s_time = self._to_cpp( Int32(time) )
		s_force_redraw = self._to_cpp( Bool(force_redraw) )
		self._viewer.spinOnce(s_time, s_force_redraw)

	def close(self):
		'''Stop Visualizer
		'''
		self._viewer.close()

	def wasStopped(self):
		'''Returns true when the user tried to close the window
		'''
		_stopped = self._from_cpp( self._viewer.wasStopped(), Bool )
		return _stopped.data

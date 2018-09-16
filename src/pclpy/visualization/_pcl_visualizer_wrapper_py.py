import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, UInt8, Int32, Bool, Float32
from .._wrapper import pyWrapper

from pclpy._pcl_visualizer_wrapper_cpp import PCLVisualizerWrapper

class PCLVisualizer(object):
	def __init__(self, wname=''):
		self._viewer = PCLVisualizerWrapper()
		self._to_cpp = pyWrapper._to_cpp
		self._from_cpp = pyWrapper._from_cpp
		if wname: self.setWindowName(wname)

	def setBackgroundColor(self, r, g, b, viewport=0):
		'''Set the viewport's background color
		Parameters
		----------
		- r : the red component of the RGB color
		- g : the green component of the RGB color
		- b : the blue component of the RGB color
		- viewport : the viewport (default: all)
		'''
		if not (isinstance( r, int ) and 0 <= r <= 255):
			rospy.ROSException( 'r is not a 8 bytes uint' )

		if not (isinstance( g, int ) and 0 <= g <= 255):
			rospy.ROSException( 'g is not a 8 bytes uint' )

		if not (isinstance( b, int ) and 0 <= b <= 255):
			rospy.ROSException( 'b is not a 8 bytes uint' )

		if not isinstance(viewport, int):
			rospy.ROSException( 'viewport is not int' )

		_r = self._to_cpp( UInt8(r) )
		_g = self._to_cpp( UInt8(g) )
		_b = self._to_cpp( UInt8(b) )
		_viewport = self._to_cpp( Int32(viewport) )

		self._viewer.setBackgroundColor( _r, _g, _b, _viewport )

	def setWindowName(self, winName):
		'''Set the visualizer window name
		Parameters
		----------
		- winName: the name of the window
		'''
		if not isinstance( winName, str ):
			rospy.ROSException( 'winName is not str' )

		_wname = self._to_cpp( String(winName) )
		self._viewer.setWindowName( _wname )

	def createViewPort(self, xmin, ymin, xmax, ymax, viewport=0):
		'''Create a new viewport from [xmin,ymin] -> [xmax,ymax]
		Returns the viewport id created
		Parameters
		----------
		- xmin : the minimum X coordinate for the viewport (0.0 <= 1.0)
		- ymin : the minimum Y coordinate for the viewport (0.0 <= 1.0)
		- xmax : the maximum X coordinate for the viewport (0.0 <= 1.0)
		- ymax : the maximum Y coordinate for the viewport (0.0 <= 1.0)
		- viewport : the id of the new viewport (default: auto)
		'''
		if not (isinstance( xmin, float ) and 0 <= xmin <= 1):
			rospy.ROSException( 'xmin is not float or not in range 0..1' )
		if not (isinstance( ymin, float ) and 0 <= ymin <= 1):
			rospy.ROSException( 'ymin is not float or not in range 0..1' )
		if not (isinstance( xmax, float ) and 0 <= xmax <= 1):
			rospy.ROSException( 'xmax is not float or not in range 0..1' )
		if not (isinstance( ymax, float ) and 0 <= ymax <= 1):
			rospy.ROSException( 'ymax is not float or not in range 0..1' )
		if not isinstance( viewport, int ):
			rospy.ROSException( 'viewport is not int' )

		_xmin = self._to_cpp( Float32(xmin) )
		_ymin = self._to_cpp( Float32(ymin) )
		_xmax = self._to_cpp( Float32(xmax) )
		_ymax = self._to_cpp( Float32(ymax) )
		_viewport = self._to_cpp( Int32(viewport) )

		_viewport = self._viewer.createViewPort( _xmin, _ymin, _xmax, _ymax, _viewport )
		return self._from_cpp(_viewport, Int32).data

	def addPointCloud(self, cloud, viewport=0):
		'''Add Point Cloud to Screen
		Returns a cloud_id
		Parameters
		----------
		- cloud : the input cloud data set
		- viewport : the view port where the Point Cloud should be added (default: all)
		'''
		if not isinstance( cloud, PointCloud2 ):
			rospy.ROSException( 'cloud is not a PointCloud2' )

		if not isinstance(viewport, int):
			rospy.ROSException( 'viewport is not int' )

		_cloud = self._to_cpp( cloud )
		_viewport = self._to_cpp( Int32(viewport) )
		_ret = self._viewer.addPointCloud( _cloud, _viewport )
		ret = self._from_cpp(_ret, String)
		return ret.data

	def removePointCloud(self, cloud_id, viewport=0):
		'''Remove Point Cloud from Screen
		Returns the operation success
		Parameters
		----------
		- cloud_id : the point cloud object id (taken from addPointCloud)
		- viewport : view port from where the Point Cloud should be removed (default: all)
		'''
		if not isinstance( cloud_id, str ):
			rospy.ROSException( 'cloud_id is not str' )

		if not isinstance(viewport, int):
			rospy.ROSException( 'viewport is not int' )

		_cloud_id = self._to_cpp( String(cloud_id) )
		_viewport = self._to_cpp( Int32(viewport) )
		ret = self._viewer.removePointCloud(_cloud_id, _viewport)
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

		_time = self._to_cpp( Int32(time) )
		_force_redraw = self._to_cpp( Bool(force_redraw) )
		self._viewer.spinOnce(_time, _force_redraw)

	def close(self):
		'''Stop Visualizer
		'''
		self._viewer.close()

	def wasStopped(self):
		'''Returns true when the user tried to close the window
		'''
		_stopped = self._from_cpp( self._viewer.wasStopped(), Bool )
		return _stopped.data

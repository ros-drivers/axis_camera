import rospy
import threading

from std_msgs.msg import Bool, Float32, Int32

from axis_camera.cfg import CameraConfig
from axis_camera.msg import PTZ, PointInRectangle
# we already run one Server in axis_ptz for backwards compatibility, so we need to use a custom one allowing for subnamsepacing
# TODO remove this once the backwards compatibility layer is thrown away
from axis_camera.dynamic_reconfigure_server2 import Server


class AxisCameraController(object):
    """
    A class serving as the Python and ROS controller of the Axis camera.

    It provides numerous topics/methods using which the camera can be controlled via VAPIX.
    Each topic is only subscribed only if the camera reports capabilities required for executing the topic's
    functionality.

    Currently provided topics are:
    - control/pan_tilt_zoom/absolute
    - control/pan_tilt_zoom/relative
    - control/pan_tilt_zoom/velocity
    - control/pan_tilt/absolute
    - control/pan_tilt/relative
    - control/pan_tilt/velocity
    - control/pan/absolute
    - control/tilt/absolute
    - control/zoom/absolute
    - control/pan/relative
    - control/tilt/relative
    - control/zoom/relative
    - control/pan/velocity
    - control/tilt/velocity
    - control/zoom/velocity
    - control/look_at
    - control/camera/focus/auto
    - control/camera/focus/absolute
    - control/camera/focus/relative
    - control/camera/focus/velocity
    - control/camera/iris/auto
    - control/camera/iris/absolute
    - control/camera/iris/relative
    - control/camera/iris/velocity
    - control/camera/brightness/absolute
    - control/camera/brightness/relative
    - control/camera/brightness/velocity
    - control/camera/backlight_compensation
    - control/camera/ir_cut_filter/auto
    - control/camera/ir_cut_filter/use
    """

    def __init__(self, api, axis, flip_vertically=False, flip_horizontally=False, mirror_horizontally=False):
        """
        Create the controller.
        :param api: The VAPIX instance to be used to communicate with the camera.
        :type api: axis_camera.VAPIX
        :param axis: The parameter holding class.
        :type axis: axis.Axis
        :param flip_vertically: If True, flip the controls vertically (for ceiling mounted cameras).
        :type flip_vertically: bool
        :param flip_horizontally: If True, flip the controls horizontally (for ceiling mounted cameras).
        :type flip_horizontally: bool
        :param mirror_horizontally: If True, mirror the controls horizontally (for backwards mounted cameras).
        :type mirror_horizontally: bool
        """
        self._api = api
        self._axis = axis
        self._flip_vertically = flip_vertically
        self._flip_horizontally = flip_horizontally
        self._mirror_horizontally = mirror_horizontally

        self._parameter_update_mutex = threading.Lock()
        self._executing_parameter_update = False
        self._reconfigure_mutex = threading.Lock()
        self._executing_reconfigure = False

        self._autofocus = True
        self._focus = 0
        self._autoiris = True
        self._iris = 0
        self._brightness = 5000
        self._backlight = False
        self._ir_cut_filter = True

        self._dynamic_reconfigure_server = Server(CameraConfig, self._reconfigure, "axis_ptz_driver")

        # set up the topics this node listens on
        # the callbacks are created as lambda wrappers around the Python API functions of this controller so that we
        # do not blow this file with more unnecessary methods
        if self._api.has_capabilities('AbsolutePan', 'AbsoluteTilt', 'AbsoluteZoom'):
            self._absolute_ptz_subscriber = rospy.Subscriber(
                "control/pan_tilt_zoom/absolute", PTZ, self._call_with_ptz_message(self.set_ptz), queue_size=100)

        if self._api.has_capabilities('RelativePan', 'RelativeTilt', 'RelativeZoom'):
            self._relative_ptz_subscriber = rospy.Subscriber(
                "control/pan_tilt_zoom/relative", PTZ, self._call_with_ptz_message(self.adjust_ptz), queue_size=100)

        if self._api.has_capabilities('ContinuousPan', 'ContinuousTilt', 'ContinuousZoom'):
            self._velocity_ptz_subscriber = rospy.Subscriber(
                "control/pan_tilt_zoom/velocity", PTZ, self._call_with_ptz_message(self.set_ptz_velocity),
                queue_size=100)

        if self._api.has_capabilities('AbsolutePan', 'AbsoluteTilt'):
            self._absolute_pan_tilt_subscriber = rospy.Subscriber(
                "control/pan_tilt/absolute", PTZ, self._call_with_pt_message(self.set_pan_tilt), queue_size=100)

        if self._api.has_capabilities('RelativePan', 'RelativeTilt'):
            self._relative_pan_tilt_subscriber = rospy.Subscriber(
                "control/pan_tilt/relative", PTZ, self._call_with_pt_message(self.adjust_pan_tilt), queue_size=100)

        if self._api.has_capabilities('ContinuousPan', 'ContinuousTilt'):
            self._velocity_pan_tilt_subscriber = rospy.Subscriber(
                "control/pan_tilt/velocity", PTZ, self._call_with_pt_message(self.set_pan_tilt_velocity),
                queue_size=100)

        if self._api.has_capability('AbsolutePan'):
            self._absolute_pan_subscriber = rospy.Subscriber(
                "control/pan/absolute", Float32, self._call_with_simple_message_data(self.set_pan), queue_size=100)

        if self._api.has_capability('AbsoluteTilt'):
            self._absolute_tilt_subscriber = rospy.Subscriber(
                "control/tilt/absolute", Float32, self._call_with_simple_message_data(self.set_tilt), queue_size=100)

        if self._api.has_capability('AbsoluteZoom'):
            self._absolute_zoom_subscriber = rospy.Subscriber(
                "control/zoom/absolute", Int32, self._call_with_simple_message_data(self.set_zoom), queue_size=100)

        if self._api.has_capability('RelativePan'):
            self._relative_pan_subscriber = rospy.Subscriber(
                "control/pan/relative", Float32, self._call_with_simple_message_data(self.adjust_pan), queue_size=100)

        if self._api.has_capability('RelativeTilt'):
            self._relative_tilt_subscriber = rospy.Subscriber(
                "control/tilt/relative", Float32, self._call_with_simple_message_data(self.adjust_tilt), queue_size=100)

        if self._api.has_capability('RelativeZoom'):
            self._relative_zoom_subscriber = rospy.Subscriber(
                "control/zoom/relative", Int32, self._call_with_simple_message_data(self.adjust_zoom), queue_size=100)

        if self._api.has_capability('ContinuousPan'):
            self._pan_velocity_subscriber = rospy.Subscriber(
                "control/pan/velocity", Int32, self._call_with_simple_message_data(self.set_pan_velocity),
                queue_size=100)

        if self._api.has_capability('ContinuousTilt'):
            self._tilt_velocity_subscriber = rospy.Subscriber(
                "control/tilt/velocity", Int32,
                self._call_with_simple_message_data(self.set_tilt_velocity), queue_size=100)

        if self._api.has_capability('ContinuousZoom'):
            self._zoom_velocity_subscriber = rospy.Subscriber(
                "control/zoom/velocity", Int32,
                self._call_with_simple_message_data(self.set_zoom_velocity), queue_size=100)

        if self._api.has_capabilities('AbsolutePan', 'AbsoluteTilt', 'AbsoluteZoom'):
            self._look_at_subscriber = rospy.Subscriber(
                "control/look_at", PointInRectangle, self._call_with_pir_message(self.look_at), queue_size=100)

        if self._api.has_capability('AutoFocus'):
            self._autofocus_subscriber = rospy.Subscriber(
                "control/camera/focus/auto", Bool, self._call_with_simple_message_data(self.set_autofocus), queue_size=100)

        if self._api.has_capability('AbsoluteFocus'):
            self._focus_subscriber = rospy.Subscriber(
                "control/camera/focus/absolute", Float32,
                self._call_with_simple_message_data(self.set_focus), queue_size=100)

        if self._api.has_capability('RelativeFocus'):
            self._focus_relative_subscriber = rospy.Subscriber(
                "control/camera/focus/relative", Float32,
                self._call_with_simple_message_data(self.adjust_focus), queue_size=100)

        if self._api.has_capability('ContinuousFocus'):
            self._focus_velocity_subscriber = rospy.Subscriber(
                "control/camera/focus/velocity", Float32,
                self._call_with_simple_message_data(self.set_focus_velocity), queue_size=100)

        if self._api.has_capability('AutoIris'):
            self._autoiris_subscriber = rospy.Subscriber(
                "control/camera/iris/auto", Bool, self._call_with_simple_message_data(self.set_autoiris), queue_size=100)

        if self._api.has_capability('AbsoluteIris'):
            self._iris_subscriber = rospy.Subscriber(
                "control/camera/iris/absolute", Float32,
                self._call_with_simple_message_data(self.set_iris), queue_size=100)

        if self._api.has_capability('RelativeIris'):
            self._iris_relative_subscriber = rospy.Subscriber(
                "control/camera/iris/relative", Float32,
                self._call_with_simple_message_data(self.adjust_iris), queue_size=100)

        if self._api.has_capability('ContinuousIris'):
            self._iris_velocity_subscriber = rospy.Subscriber(
                "control/camera/iris/velocity", Float32,
                self._call_with_simple_message_data(self.set_iris_velocity), queue_size=100)

        if self._api.has_capability('AbsoluteBrightness'):
            self._brightness_subscriber = rospy.Subscriber(
                "control/camera/brightness/absolute", Float32,
                self._call_with_simple_message_data(self.set_brightness), queue_size=100)

        if self._api.has_capability('RelativeBrightness'):
            self._brightness_relative_subscriber = rospy.Subscriber(
                "control/camera/brightness/relative", Float32,
                self._call_with_simple_message_data(self.adjust_brightness), queue_size=100)

        if self._api.has_capability('ContinuousIris'):
            self._brightness_velocity_subscriber = rospy.Subscriber(
                "control/camera/brightness/velocity", Float32,
                self._call_with_simple_message_data(self.set_brightness_velocity), queue_size=100)

        if self._api.has_capability('BackLight'):
            self._use_backlight_subscriber = rospy.Subscriber(
                "control/camera/backlight_compensation", Bool,
                self._call_with_simple_message_data(self.use_backlight_compensation), queue_size=100)

        if self._api.has_capabilities('IrCutFilter', 'AutoIrCutFilter'):
            self._use_ir_cut_filter_auto_subscriber = rospy.Subscriber(
                "control/camera/ir_cut_filter/auto", Bool,
                self._call_with_simple_message_data(self.set_ir_cut_filter_auto), queue_size=100)

        if self._api.has_capabilities('IrCutFilter'):
            self._use_ir_cut_filter_use_subscriber = rospy.Subscriber(
                "control/camera/ir_cut_filter/use", Bool,
                self._call_with_simple_message_data(self.set_ir_cut_filter_use), queue_size=100)

    def set_ptz(self, pan, tilt, zoom):
        """Command the PTZ unit with an absolute pose taking into account flips and mirroring.
        :param pan: The desired pan. In None, pan is not commanded at all.
                    The value is automatically normalized to <-180,+180>
        :type pan: float
        :param tilt: The desired tilt. In None, tilt is not commanded at all.
                     The value is automatically normalized to <-180,+180>
        :type tilt: float
        :param zoom: The desired zoom level. In None, zoom is not commanded at all.
        :type pan: int
        :return: The pan, tilt and zoom values that were really applied (e.g. the cropped and normalized input)
        :rtype: tuple

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: This call doesn't wait for the command to finish execution (which might take a while if e.g. a move
                  from -170 to +170 pan is requested).

        .. note:: Since the pan and tilt values are periodic and normalization takes place in this function, you can
                  simply call this command in loops like e.g. `for pan in range(0,3600,30): move_ptz_absolute(pan)`
                  to rotate the camera 10 times.
        """
        (pan, tilt) = self._apply_flip_and_mirror_absolute(pan, tilt)
        return self._api.move_ptz_absolute(pan, tilt, zoom)

    def adjust_ptz(self, pan, tilt, zoom):
        """Command the PTZ unit with a relative pose shift taking into account flips and mirroring.
        :param pan: The pan change. In None or 0, pan remains unchanged.
                    The value is automatically normalized to <-360,+360>. May be negative.
        :type pan: float
        :param tilt: The tilt change. In None or 0, tilt remains unchanged.
                     The value is automatically normalized to <-360,+360>. May be negative.
        :type tilt: float
        :param zoom: The zoom change. In None or 0, zoom remains unchanged. May be negative.
        :type pan: int
        :return: The pan, tilt and zoom change values that were really applied (e.g. the cropped and normalized input)
        :rtype: tuple

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: This call doesn't wait for the command to finish execution (which might take a while if e.g. a move
                  by 300 deg pan is requested).
        """
        (pan, tilt) = self._apply_flip_and_mirror_relative(pan, tilt)
        return self._api.move_ptz_relative(pan, tilt, zoom)

    def set_ptz_velocity(self, pan, tilt, zoom):
        """Command the PTZ unit velocity in terms of pan, tilt and zoom taking into account flips and mirroring.
        :param pan: Pan speed. In None or 0, pan remains unchanged. Pan speed is aperiodic (can be higher than 360).
                    May be negative.
        :type pan: int
        :param tilt: Tilt speed. In None or 0, tilt remains unchanged. Tilt speed is aperiodic (can be higher than 360).
                    May be negative.
        :type tilt: int
        :param zoom: Zoom speed. In None or 0, zoom remains unchanged. May be negative.
        :type pan: int
        :return: The pan, tilt and zoom change values that were really applied (e.g. the cropped and normalized input)
        :rtype: tuple

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: This call doesn't wait for the command to finish execution.

        .. note:: It is not clearly stated in the API, but practically any following absolute/relative pose command
                  sets velocity to zero.
        """
        (pan, tilt) = self._apply_flip_and_mirror_velocity(pan, tilt)
        return self._api.set_ptz_velocity(pan, tilt, zoom)

    def set_pan_tilt(self, pan, tilt):
        """Command the PTZ unit with an absolute pan and tilt taking into account flips and mirroring.
        :param pan: The desired pan. In None, pan is not commanded at all.
                    The value is automatically normalized to <-180,+180>
        :type pan: float
        :param tilt: The desired tilt. In None, tilt is not commanded at all.
                     The value is automatically normalized to <-180,+180>
        :type tilt: float
        :return: The pan and tilt values that were really applied (e.g. the cropped and normalized input)
        :rtype: tuple

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: This call doesn't wait for the command to finish execution (which might take a while if e.g. a move
                  from -170 to +170 pan is requested).

        .. note:: Since the pan and tilt values are periodic and normalization takes place in this function, you can
                  simply call this command in loops like e.g. `for pan in range(0,3600,30): move_ptz_absolute(pan)`
                  to rotate the camera 10 times.
        """
        (pan, tilt) = self._apply_flip_and_mirror_absolute(pan, tilt)
        return self._api.move_ptz_absolute(pan=pan, tilt=tilt)[slice(0, 2)]

    def adjust_pan_tilt(self, pan, tilt):
        """Command the PTZ unit with a relative pan and tilt taking into account flips and mirroring.
        :param pan: The pan change. In None or 0, pan remains unchanged.
                    The value is automatically normalized to <-360,+360>. May be negative.
        :type pan: float
        :param tilt: The tilt change. In None or 0, tilt remains unchanged.
                     The value is automatically normalized to <-360,+360>. May be negative.
        :type tilt: float
        :return: The pan and tilt change values that were really applied (e.g. the cropped and normalized input)
        :rtype: tuple

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: This call doesn't wait for the command to finish execution (which might take a while if e.g. a move
                  by 300 deg pan is requested).
        """
        (pan, tilt) = self._apply_flip_and_mirror_relative(pan, tilt)
        return self._api.move_ptz_relative(pan=pan, tilt=tilt)[slice(0, 2)]

    def set_pan_tilt_velocity(self, pan, tilt):
        """Command the PTZ unit velocity in terms of pan, tilt and zoom taking into account flips and mirroring.
        :param pan: Pan speed. In None or 0, pan remains unchanged. Pan speed is aperiodic (can be higher than 360).
                    May be negative.
        :type pan: int
        :param tilt: Tilt speed. In None or 0, tilt remains unchanged. Tilt speed is aperiodic (can be higher than 360).
                    May be negative.
        :type tilt: int
        :return: The pan and tilt velocity values that were really applied (e.g. the cropped and normalized input)
        :rtype: tuple

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: This call doesn't wait for the command to finish execution.

        .. note:: It is not clearly stated in the API, but practically any following absolute/relative pose command
                  sets velocity to zero.
        """
        (pan, tilt) = self._apply_flip_and_mirror_velocity(pan, tilt)
        return self._api.set_ptz_velocity(pan=pan, tilt=tilt)[slice(0, 2)]

    def set_pan(self, pan):
        """Command an absolute pan taking into account flips and mirroring.
        :param pan: The desired pan. The value is automatically normalized to <-180,+180>
        :type pan: float
        :return: The pan value that was really applied (e.g. the cropped and normalized input)
        :rtype: float

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: This call doesn't wait for the command to finish execution (which might take a while if e.g. a move
                  from -170 to +170 pan is requested).

        .. note:: Since the pan and tilt values are periodic and normalization takes place in this function, you can
                  simply call this command in loops like e.g. `for pan in range(0,3600,30): move_ptz_absolute(pan)`
                  to rotate the camera 10 times.
        """
        (pan, tilt) = self._apply_flip_and_mirror_absolute(pan, 0)
        return self._api.move_ptz_absolute(pan=pan)[0]

    def set_tilt(self, tilt):
        """Command an absolute tilt taking into account flips and mirroring.
        :param tilt: The desired tilt. The value is automatically normalized to <-180,+180>
        :type tilt: float
        :return: The tilt value that was really applied (e.g. the cropped and normalized input)
        :rtype: float

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: This call doesn't wait for the command to finish execution (which might take a while if e.g. a move
                  from -170 to +170 pan is requested).

        .. note:: Since the pan and tilt values are periodic and normalization takes place in this function, you can
                  simply call this command in loops like e.g. `for pan in range(0,3600,30): move_ptz_absolute(pan)`
                  to rotate the camera 10 times.
        """
        (pan, tilt) = self._apply_flip_and_mirror_absolute(0, tilt)
        return self._api.move_ptz_absolute(tilt=tilt)[1]

    def set_zoom(self, zoom):
        """Command an absolute zoom.
        :param zoom: The desired zoom level.
        :type zoom: int
        :return: The zoom value that was really applied (e.g. the cropped and normalized input)
        :rtype: int

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: This call doesn't wait for the command to finish execution (which might take a while if e.g. a move
                  from -170 to +170 pan is requested).

        .. note:: Since the pan and tilt values are periodic and normalization takes place in this function, you can
                  simply call this command in loops like e.g. `for pan in range(0,3600,30): move_ptz_absolute(pan)`
                  to rotate the camera 10 times.
        """
        return self._api.move_ptz_absolute(zoom=zoom)[2]

    def adjust_pan(self, pan):
        """Command a relative pan taking into account flips and mirroring.
        :param pan: The pan change. The value is automatically normalized to <-360,+360>. May be negative.
        :type pan: float
        :return: The pan change value that was really applied (e.g. the cropped and normalized input)
        :rtype: float

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: This call doesn't wait for the command to finish execution (which might take a while if e.g. a move
                  by 300 deg pan is requested).
        """
        (pan, _) = self._apply_flip_and_mirror_relative(pan, 0)
        return self._api.move_ptz_relative(pan=pan)[0]

    def adjust_tilt(self, tilt):
        """Command a relative tilt taking into account flips and mirroring.
        :param tilt: The tilt change. The value is automatically normalized to <-360,+360>. May be negative.
        :type tilt: float
        :return: The tilt change value that was really applied (e.g. the cropped and normalized input)
        :rtype: float

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: This call doesn't wait for the command to finish execution (which might take a while if e.g. a move
                  by 300 deg pan is requested).
        """
        (_, tilt) = self._apply_flip_and_mirror_relative(0, tilt)
        return self._api.move_ptz_relative(tilt=tilt)[1]

    def adjust_zoom(self, zoom):
        """Command a relative zoom.
        :param zoom: The zoom change. In None or 0, zoom remains unchanged. May be negative.
        :type zoom: int
        :return: The zoom change value that was really applied (e.g. the cropped and normalized input)
        :rtype: int

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: This call doesn't wait for the command to finish execution (which might take a while if e.g. a move
                  by 300 deg pan is requested).
        """
        return self._api.move_ptz_relative(zoom=zoom)[2]

    def set_pan_velocity(self, pan):
        """Command pan velocity.
        :param pan: Pan speed. Pan speed is aperiodic (can be higher than 360). May be negative.
        :type pan: int
        :return: The pan velocity that was really applied (e.g. the cropped and normalized input)
        :rtype: int

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: This call doesn't wait for the command to finish execution.

        .. note:: It is not clearly stated in the API, but practically any following absolute/relative pose command
                  sets velocity to zero.
        """
        (pan, _) = self._apply_flip_and_mirror_velocity(pan, 0)
        return self._api.set_ptz_velocity(pan=pan)[0]

    def set_tilt_velocity(self, tilt):
        """Command tilt velocity.
        :param tilt: Tilt speed. Tilt speed is aperiodic (can be higher than 360). May be negative.
        :type tilt: int
        :return: The tilt velocity that was really applied (e.g. the cropped and normalized input)
        :rtype: int

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: This call doesn't wait for the command to finish execution.

        .. note:: It is not clearly stated in the API, but practically any following absolute/relative pose command
                  sets velocity to zero.
        """
        (_, tilt) = self._apply_flip_and_mirror_velocity(0, tilt)
        return self._api.set_ptz_velocity(tilt=tilt)[1]

    def set_zoom_velocity(self, zoom):
        """Command zoom velocity.
        :param zoom: Zoom speed. May be negative.
        :type zoom: int
        :return: The zoom velocity that was really applied (e.g. the cropped and normalized input)
        :rtype: int

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: This call doesn't wait for the command to finish execution.

        .. note:: It is not clearly stated in the API, but practically any following absolute/relative pose command
                  sets velocity to zero.
        """
        return self._api.set_ptz_velocity(zoom=zoom)[2]

    def look_at(self, x, y, image_width, image_height):
        """Point the camera center to a point with the given coordinates in the camera image.
        :param x: X coordinate of the look-at point.
        :type x: int
        :param y: X coordinate of the look-at point.
        :type y: int
        :param image_width: Width of the image in pixels.
        :type image_width: int
        :param image_height: Height of the image in pixels.
        :type image_height: int

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error
        """
        self._api.look_at(x, y, image_width, image_height)

    def set_autofocus(self, use):
        """Command the camera to use/stop using autofocus.
        :param use: True: use autofocus; False: do not use it.
        :type use: bool
        :return: use
        :rtype: bool

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error
        """
        self._api.use_autofocus(use)
        self._autofocus = use
        self._send_parameter_update("autofocus", use)
        return use

    def set_focus(self, focus, set_also_autofocus=True):
        """Set focus to the desired value (implies turning off autofocus).
        :param focus: The desired focus value.
        :type focus: int
        :param set_also_autofocus: If True and autofocus is on, turn it off first.
        :type set_also_autofocus: bool
        :return: The focus value that was really applied (e.g. the cropped and normalized input)
        :rtype: int

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error
        """
        focus = self._api.set_focus(focus)
        self._focus = focus
        self._send_parameter_update("focus", focus)
        if set_also_autofocus:
            self._autofocus = False
            self._send_parameter_update("autofocus", False)
        return focus

    def adjust_focus(self, amount, set_also_autofocus=True):
        """Add the desired amount to the focus value (implies turning off autofocus).
        :param amount: The desired focus change amount.
        :type amount: int
        :param set_also_autofocus: If True and autofocus is on, turn it off first.
        :type set_also_autofocus: bool
        :return: The focus change amount that was really applied (e.g. the cropped and normalized input)
        :rtype: int

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error
        """
        amount = self._api.adjust_focus(amount)
        self._focus += amount
        self._send_parameter_update("focus", self._focus)
        if set_also_autofocus:
            self._autofocus = False
            self._send_parameter_update("autofocus", False)
        return amount

    def set_focus_velocity(self, velocity):
        """Set the focus "speed" (implies turning off autofocus).
        :param velocity: The desired focus velocity.
        :type velocity: int
        :return: The focus velocity that was really applied (e.g. the cropped and normalized input)
        :rtype: int

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error
        """
        return self._api.set_focus_velocity(velocity)
        # TODO self.focus updating

    def set_autoiris(self, use):
        """Command the camera to use/stop using auto iris adjustment.
        :param use: True: use auto iris adjustment; False: do not use it.
        :type use: bool
        :return: use
        :rtype: bool

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error
        """
        self._api.use_autoiris(use)
        self._autoiris = use
        self._send_parameter_update("autoiris", use)
        return use

    def set_iris(self, iris, set_also_autoiris=True):
        """Set iris to the desired value (implies turning off autoiris).
        :param iris: The desired iris value.
        :type iris: int
        :param set_also_autoiris: If True and autoiris is on, turn it off first.
        :type set_also_autoiris: bool
        :return: The iris value that was really applied (e.g. the cropped and normalized input)
        :rtype: int

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error
        """
        iris = self._api.set_iris(iris)
        self._iris = iris
        self._send_parameter_update("iris", iris)
        if set_also_autoiris:
            self._autoiris = False
            self._send_parameter_update("autoiris", False)
        return iris

    def adjust_iris(self, amount, set_also_autoiris=True):
        """Add the desired amount to the iris value (implies turning off autoiris).
        :param amount: The desired iris change amount.
        :type amount: int
        :param set_also_autoiris: If True and autoiris is on, turn it off first.
        :type set_also_autoiris: bool
        :return: The iris change amount that was really applied (e.g. the cropped and normalized input)
        :rtype: int

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error
        """
        amount = self._api.adjust_iris(amount)
        self._iris += amount
        self._send_parameter_update("iris", self._iris)
        if set_also_autoiris:
            self._autoiris = False
            self._send_parameter_update("autoiris", False)
        return amount

    def set_iris_velocity(self, velocity):
        """Set the iris "speed" (implies turning off autoiris).
        :param velocity: The desired iris velocity.
        :type velocity: int
        :return: The iris velocity that was really applied (e.g. the cropped and normalized input)
        :rtype: int

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error
        """
        return self._api.set_iris_velocity(velocity)
        # TODO self.iris updating

    def set_brightness(self, brightness):
        """Set brightness to the desired value.
        :param brightness: The desired brightness value.
        :type brightness: int
        :return: The brightness value that was really applied (e.g. the cropped and normalized input)
        :rtype: int

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: The brightness setting has no effect on Axis 214 PTZ.
        """
        brightness = self._api.set_brightness(brightness)
        self._brightness = brightness
        self._send_parameter_update("brightness", brightness)
        return brightness

    def adjust_brightness(self, amount):
        """Add the desired amount to the brightness value.
        :param amount: The desired brightness change amount.
        :type amount: int
        :return: The brightness change amount that was really applied (e.g. the cropped and normalized input)
        :rtype: int

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: The brightness setting has no effect on Axis 214 PTZ.
        """
        amount = self._api.adjust_brightness(amount)
        self._brightness += amount
        self._send_parameter_update("brightness", self._brightness)
        return amount

    def set_brightness_velocity(self, velocity):
        """Set the brightness "speed".
        :param velocity: The desired brightness velocity.
        :type velocity: int
        :return: The brightness velocity that was really applied (e.g. the cropped and normalized input)
        :rtype: int

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error

        .. note:: The brightness setting has no effect on Axis 214 PTZ.
        """
        return self._api.set_brightness_velocity(velocity)
        # TODO self.brightness updating

    def use_backlight_compensation(self, use):
        """Command the camera to use/stop using backlight compensation (requires autoiris=on set before).
        :param use: True: use backlight compensation; False: do not use it.
        :type use: bool
        :return: use
        :rtype: bool

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error
        """
        if self._autoiris:  # the compensation needs autoiris to be active
            self._api.use_backlight_compensation(use)
            self._backlight = use
        else:
            use = False

        self._send_parameter_update("backlight", use)

        return use

    def set_ir_cut_filter_auto(self, use):
        """Command the camera to use auto infrared filter.
        :param use: True: use automatic infrared filter;
                    False: use always.
        :type use: bool
        :return: True if the filter is always used, None if it is set to auto.
        :rtype: bool

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error
        """
        use = (None if use else True)
        # auto IR filter requires autoiris to be active
        if self._autoiris or use is not None:
            self._api.use_ir_cut_filter(use)
            self._ir_cut_filter = use
        else:
            use = True

        self._send_parameter_update("ircutfilter", "auto" if (use is None) else "on")

        return use

    def set_ir_cut_filter_use(self, use):
        """Command the camera to use/stop using infrared filter.
        :param use: Whether to use the filter.
        :type use: bool
        :return: use
        :rtype: bool

        :raises: RuntimeError if the camera doesn't have capabilities to execute the given command.
        :raises: IOError, urllib2.URLError on network communication error
        """
        self._api.use_ir_cut_filter(use)
        self._ir_cut_filter = use

        self._send_parameter_update("ircutfilter", "on" if use else "off")

        return use

    def _reconfigure(self, config, level, subname):
        """
        Dynamic reconfigure callback.
        :param config: The config to be used.
        :type config: dict
        :return: The config with really used values.
        :rtype: dict
        """
        if self._executing_reconfigure or self._executing_parameter_update or self._axis._executing_reconfigure:
            return config

        with self._reconfigure_mutex:
            self._executing_reconfigure = True

            try:
                if config.autofocus != self._autofocus:
                    self.set_autofocus(config.autofocus)
            except (IOError, ValueError, RuntimeError) as e:
                rospy.logwarn("Could not apply dynamic reconfigure for autofocus. Cause: %r" % e)

            try:
                if config.focus != self._focus:
                    self.set_focus(config.focus)
            except (IOError, ValueError, RuntimeError) as e:
                rospy.logwarn("Could not apply dynamic reconfigure for focus. Cause: %r" % e)

            try:
                if config.autoiris != self._autoiris:
                    self.set_autoiris(config.autoiris)
            except (IOError, ValueError, RuntimeError) as e:
                rospy.logwarn("Could not apply dynamic reconfigure for autoiris. Cause: %r" % e)

            try:
                if config.iris != self._iris:
                    self.set_iris(config.iris)
            except (IOError, ValueError, RuntimeError) as e:
                rospy.logwarn("Could not apply dynamic reconfigure for iris. Cause: %r" % e)

            try:
                if config.brightness != self._brightness:
                    self.set_brightness(config.brightness)
            except (IOError, ValueError, RuntimeError) as e:
                rospy.logwarn("Could not apply dynamic reconfigure for brightness. Cause: %r" % e)

            try:
                if config.backlight != self._backlight:
                    self.use_backlight_compensation(config.backlight)
            except (IOError, ValueError, RuntimeError) as e:
                rospy.logwarn("Could not apply dynamic reconfigure for backlight compenstaion. Cause: %r" % e)

            try:
                ir = None if config.ircutfilter == "auto" else (config.ircutfilter == "on")
                if ir != self._ir_cut_filter:
                    if ir is None or self._ir_cut_filter is None:
                        self.set_ir_cut_filter_auto(ir is None)
                    if ir is not None:
                        self.set_ir_cut_filter_use(ir)
            except (IOError, ValueError, RuntimeError) as e:
                rospy.logwarn("Could not apply dynamic reconfigure for IR cut filter. Cause: %r" % e)

            config.autofocus = self._autofocus
            config.focus = self._focus
            config.autoiris = self._autoiris
            config.iris = self._iris
            config.brightness = self._brightness
            config.backlight = self._backlight
            config.ircutfilter = ("auto" if self._ir_cut_filter is None else ("on" if self._ir_cut_filter else "off"))

            self._executing_reconfigure = False

            return config

    # Helper functions

    def _send_parameter_update(self, parameter, value):
        """
        Notify the parameter change via dynamic reconfigure.
        :param parameter: The parameter that has changed.
        :type parameter: basestring
        :param value: New value of the parameter.
        :type value: Any
        """
        if self._executing_parameter_update:
            return

        with self._parameter_update_mutex:
            update = dict()
            update[parameter] = value

            self._executing_parameter_update = True

            if hasattr(self._axis, 'srv') and self._axis.srv is not None and not self._axis._executing_reconfigure:
                # axis.srv is instantiated after camera controller
                self._axis.srv.update_configuration(update)

            if hasattr(self, '_dynamic_reconfigure_server') and self._dynamic_reconfigure_server is not None \
                    and not self._executing_reconfigure:
                # this server can also be initialized later
                self._dynamic_reconfigure_server.update_configuration(update)

            self._executing_parameter_update = False

    def _apply_flip_and_mirror_absolute(self, pan, tilt):
        """
        Apply flipping and mirroring to absolute pan and tilt command.
        :param pan: The input pan.
        :type pan: float
        :param tilt: The input tilt.
        :type tilt: float
        :return: The corrected pan and tilt.
        :rtype: tuple
        """
        if self._flip_vertically:
            tilt = -tilt
        if self._flip_horizontally:
            pan = 180 - pan
        if self._mirror_horizontally:
            pan = -pan

        return pan, tilt

    def _apply_flip_and_mirror_relative(self, pan, tilt):
        """
        Apply flipping and mirroring to relative pan and tilt command.
        :param pan: The input pan.
        :type pan: float
        :param tilt: The input tilt.
        :type tilt: float
        :return: The corrected pan and tilt.
        :rtype: tuple
        """
        if self._flip_vertically:
            tilt = -tilt
        if self._flip_horizontally:
            pan = -pan
        if self._mirror_horizontally:
            pan = -pan

        return pan, tilt

    def _apply_flip_and_mirror_velocity(self, pan, tilt):
        """
        Apply flipping and mirroring to velocity pan and tilt command.
        :param pan: The input pan.
        :type pan: float
        :param tilt: The input tilt.
        :type tilt: float
        :return: The corrected pan and tilt.
        :rtype: tuple
        """
        if self._flip_vertically:
            tilt = -tilt
        if self._flip_horizontally:
            pan = -pan
        if self._mirror_horizontally:
            pan = -pan

        return pan, tilt

    @staticmethod
    def _call_with_simple_message_data(func):
        """
        Create a one-arg lambda that extracts the "data" field from its argument and passes it to the given function.
        :param func: The function to call.
        :type func: function
        :return: The lambda function.
        :rtype: function
        """
        return lambda (msg): func(msg.data)

    @staticmethod
    def _call_with_ptz_message(func):
        """
        Create a one-arg lambda that extracts the "pan", "tilt" and "zoom" fields from its argument and passes them to
        the given function.
        :param func: The function to call.
        :type func: function
        :return: The lambda function.
        :rtype: function
        """
        return lambda (msg): func(msg.pan, msg.tilt, msg.zoom)

    @staticmethod
    def _call_with_pt_message(func):
        """
        Create a one-arg lambda that extracts the "pan" and "tilt" fields from its argument and passes them to the given
        function.
        :param func: The function to call.
        :type func: function
        :return: The lambda function.
        :rtype: function
        """
        return lambda (msg): func(msg.pan, msg.tilt)

    @staticmethod
    def _call_with_pir_message(func):
        """
        Create a one-arg lambda that extracts the "x", "y", "width" and "height" fields from its argument and passes
        them to the given function.
        :param func: The function to call.
        :type func: function
        :return: The lambda function.
        :rtype: function
        """
        return lambda (msg): func(msg.x, msg.y, msg.image_width, msg.image_height)
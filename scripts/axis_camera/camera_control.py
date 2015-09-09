import rospy
# we already run one Server in axis_ptz for backwards compatibility, so we need to use a custom one allowing for subnamsepacing
from axis_camera.dynamic_reconfigure_server2 import Server
from std_msgs.msg import Bool, Float32, Int32
from dynamic_reconfigure.msg import Config, BoolParameter, IntParameter, DoubleParameter, StrParameter

from axis_camera.cfg import CameraConfig
from axis_camera.msg import PTZ, PointInRectangle


class AxisCameraController(object):
    def __init__(self, api, flip_vertically=False, flip_horizontally=False, mirror_horizontally=False):

        self.api = api
        self.flip_vertically = flip_vertically
        self.flip_horizontally = flip_horizontally
        self.mirror_horizontally = mirror_horizontally

        self._parameter_updates_publisher_deprecated = rospy.Publisher("axis_ptz/parameter_updates", Config, queue_size=100)
        self._parameter_updates_publisher = rospy.Publisher("axis_ptz_driver/parameter_updates", Config, queue_size=100)

        self.autofocus = True
        self.focus = 0
        self.autoiris = True
        self.iris = 0
        self.brightness = 5000
        self.backlight = False
        self.ir_cut_filter = True

        self.dynamic_reconfigure_server = Server(CameraConfig, self._reconfigure, "axis_ptz_driver")

        # set up the topics this node listens on
        # the callbacks are created as lambda wrappers around the normal functions of this controller
        if self.api.has_capabilities('AbsolutePan', 'AbsoluteTilt', 'AbsoluteZoom'):
            self.absolute_ptz_subscriber = rospy.Subscriber(
                "control/pan_tilt_zoom/absolute", PTZ, self._call_with_ptz_message(self.set_ptz), queue_size=100)

        if self.api.has_capabilities('RelativePan', 'RelativeTilt', 'RelativeZoom'):
            self.relative_ptz_subscriber = rospy.Subscriber(
                "control/pan_tilt_zoom/relative", PTZ, self._call_with_ptz_message(self.adjust_ptz), queue_size=100)

        if self.api.has_capabilities('ContinuousPan', 'ContinuousTilt', 'ContinuousZoom'):
            self.velocity_ptz_subscriber = rospy.Subscriber(
                "control/pan_tilt_zoom/velocity", PTZ, self._call_with_ptz_message(self.set_ptz_velocity), queue_size=100)

        if self.api.has_capabilities('AbsolutePan', 'AbsoluteTilt'):
            self.absolute_pan_tilt_subscriber = rospy.Subscriber(
                "control/pan_tilt/absolute", PTZ, self._call_with_pt_message(self.set_pan_tilt), queue_size=100)

        if self.api.has_capabilities('RelativePan', 'RelativeTilt'):
            self.relative_pan_tilt_subscriber = rospy.Subscriber(
                "control/pan_tilt/relative", PTZ, self._call_with_pt_message(self.adjust_pan_tilt), queue_size=100)

        if self.api.has_capabilities('ContinuousPan', 'ContinuousTilt'):
            self.velocity_pan_tilt_subscriber = rospy.Subscriber(
                "control/pan_tilt/velocity", PTZ, self._call_with_pt_message(self.set_pan_tilt_velocity), queue_size=100)

        if self.api.has_capability('AbsolutePan'):
            self.absolute_pan_subscriber = rospy.Subscriber(
                "control/pan/absolute", Float32, self._call_with_simple_message_data(self.set_pan), queue_size=100)

        if self.api.has_capability('AbsoluteTilt'):
            self.absolute_tilt_subscriber = rospy.Subscriber(
                "control/tilt/absolute", Float32, self._call_with_simple_message_data(self.set_tilt), queue_size=100)

        if self.api.has_capability('AbsoluteZoom'):
            self.absolute_zoom_subscriber = rospy.Subscriber(
                "control/zoom/absolute", Int32, self._call_with_simple_message_data(self.set_zoom), queue_size=100)

        if self.api.has_capability('RelativePan'):
            self.relative_pan_subscriber = rospy.Subscriber(
                "control/pan/relative", Float32, self._call_with_simple_message_data(self.adjust_pan), queue_size=100)

        if self.api.has_capability('RelativeTilt'):
            self.relative_tilt_subscriber = rospy.Subscriber(
                "control/tilt/relative", Float32, self._call_with_simple_message_data(self.adjust_tilt), queue_size=100)

        if self.api.has_capability('RelativeZoom'):
            self.relative_zoom_subscriber = rospy.Subscriber(
                "control/zoom/relative", Int32, self._call_with_simple_message_data(self.adjust_zoom), queue_size=100)

        if self.api.has_capability('ContinuousPan'):
            self.velocity_pan_subscriber = rospy.Subscriber(
                "control/pan/velocity", Int32, self._call_with_simple_message_data(self.set_pan_velocity), queue_size=100)

        if self.api.has_capability('ContinuousTilt'):
            self.velocity_tilt_subscriber = rospy.Subscriber(
                "control/tilt/velocity", Int32,
                self._call_with_simple_message_data(self.set_tilt_velocity), queue_size=100)

        if self.api.has_capability('ContinuousZoom'):
            self.velocity_zoom_subscriber = rospy.Subscriber(
                "control/zoom/velocity", Int32,
                self._call_with_simple_message_data(self.set_zoom_velocity), queue_size=100)

        if self.api.has_capabilities('AbsolutePan', 'AbsoluteTilt', 'AbsoluteZoom'):
            self.look_at_subscriber = rospy.Subscriber(
                "control/look_at", PointInRectangle, self._call_with_pir_message(self.look_at), queue_size=100)

        if self.api.has_capability('AutoFocus'):
            self.autofocus_subscriber = rospy.Subscriber(
                "control/camera/focus/auto", Bool, self._call_with_simple_message_data(self.set_autofocus), queue_size=100)

        if self.api.has_capability('AbsoluteFocus'):
            self.focus_subscriber = rospy.Subscriber(
                "control/camera/focus/absolute", Float32,
                self._call_with_simple_message_data(self.set_focus), queue_size=100)

        if self.api.has_capability('RelativeFocus'):
            self.focus_relative_subscriber = rospy.Subscriber(
                "control/camera/focus/relative", Float32,
                self._call_with_simple_message_data(self.adjust_focus), queue_size=100)

        if self.api.has_capability('ContinuousFocus'):
            self.focus_velocity_subscriber = rospy.Subscriber(
                "control/camera/focus/velocity", Float32,
                self._call_with_simple_message_data(self.set_focus_velocity), queue_size=100)

        if self.api.has_capability('AutoIris'):
            self.autoiris_subscriber = rospy.Subscriber(
                "control/camera/iris/auto", Bool, self._call_with_simple_message_data(self.set_autoiris), queue_size=100)

        if self.api.has_capability('AbsoluteIris'):
            self.iris_subscriber = rospy.Subscriber(
                "control/camera/iris/absolute", Float32,
                self._call_with_simple_message_data(self.set_iris), queue_size=100)

        if self.api.has_capability('RelativeIris'):
            self.iris_relative_subscriber = rospy.Subscriber(
                "control/camera/iris/relative", Float32,
                self._call_with_simple_message_data(self.adjust_iris), queue_size=100)

        if self.api.has_capability('ContinuousIris'):
            self.iris_velocity_subscriber = rospy.Subscriber(
                "control/camera/iris/velocity", Float32,
                self._call_with_simple_message_data(self.set_iris_velocity), queue_size=100)

        if self.api.has_capability('AbsoluteBrightness'):
            self.brightness_subscriber = rospy.Subscriber(
                "control/camera/brightness/absolute", Float32,
                self._call_with_simple_message_data(self.set_brightness), queue_size=100)

        if self.api.has_capability('RelativeBrightness'):
            self.brightness_relative_subscriber = rospy.Subscriber(
                "control/camera/brightness/relative", Float32,
                self._call_with_simple_message_data(self.adjust_brightness), queue_size=100)

        if self.api.has_capability('ContinuousIris'):
            self.brightness_velocity_subscriber = rospy.Subscriber(
                "control/camera/brightness/velocity", Float32,
                self._call_with_simple_message_data(self.set_brightness_velocity), queue_size=100)

        if self.api.has_capability('BackLight'):
            self.use_backlight_subscriber = rospy.Subscriber(
                "control/camera/backlight_compensation", Bool,
                self._call_with_simple_message_data(self.use_backlight_compensation), queue_size=100)

        if self.api.has_capabilities('IrCutFilter', 'AutoIrCutFilter'):
            self.use_ir_cut_filter_auto_subscriber = rospy.Subscriber(
                "control/camera/ir_cut_filter/auto", Bool,
                self._call_with_simple_message_data(self.set_ir_cut_filter_auto), queue_size=100)

        if self.api.has_capabilities('IrCutFilter'):
            self.use_ir_cut_filter_use_subscriber = rospy.Subscriber(
                "control/camera/ir_cut_filter/use", Bool,
                self._call_with_simple_message_data(self.set_ir_cut_filter_use), queue_size=100)

    def set_ptz(self, pan, tilt, zoom):
        (pan, tilt) = self._apply_flip_and_mirror_absolute(pan, tilt)
        self.api.move_ptz_absolute(pan, tilt, zoom)

    def adjust_ptz(self, pan, tilt, zoom):
        (pan, tilt) = self._apply_flip_and_mirror_relative(pan, tilt)
        self.api.move_ptz_relative(pan, tilt, zoom)

    def set_ptz_velocity(self, pan, tilt, zoom):
        (pan, tilt) = self._apply_flip_and_mirror_velocity(pan, tilt)
        self.api.set_ptz_velocity(pan, tilt, zoom)

    def set_pan_tilt(self, pan, tilt):
        (pan, tilt) = self._apply_flip_and_mirror_absolute(pan, tilt)
        self.api.move_ptz_absolute(pan=pan, tilt=tilt)

    def adjust_pan_tilt(self, pan, tilt):
        (pan, tilt) = self._apply_flip_and_mirror_relative(pan, tilt)
        self.api.move_ptz_relative(pan=pan, tilt=tilt)

    def set_pan_tilt_velocity(self, pan, tilt):
        (pan, tilt) = self._apply_flip_and_mirror_velocity(pan, tilt)
        self.api.set_ptz_velocity(pan=pan, tilt=tilt)

    def set_pan(self, pan):
        (pan, tilt) = self._apply_flip_and_mirror_absolute(pan, 0)
        self.api.move_ptz_absolute(pan=pan)

    def set_tilt(self, tilt):
        (pan, tilt) = self._apply_flip_and_mirror_absolute(0, tilt)
        self.api.move_ptz_absolute(tilt=tilt)

    def set_zoom(self, zoom):
        self.api.move_ptz_absolute(zoom=zoom)

    def adjust_pan(self, pan):
        (pan, tilt) = self._apply_flip_and_mirror_relative(pan, 0)
        self.api.move_ptz_relative(pan=pan)

    def adjust_tilt(self, tilt):
        (pan, tilt) = self._apply_flip_and_mirror_relative(0, tilt)
        self.api.move_ptz_relative(tilt=tilt)

    def adjust_zoom(self, zoom):
        self.api.move_ptz_relative(zoom=zoom)

    def set_pan_velocity(self, pan):
        (pan, tilt) = self._apply_flip_and_mirror_velocity(pan, 0)
        self.api.set_ptz_velocity(pan=pan)

    def set_tilt_velocity(self, tilt):
        (pan, tilt) = self._apply_flip_and_mirror_velocity(0, tilt)
        self.api.set_ptz_velocity(tilt=tilt)

    def set_zoom_velocity(self, zoom):
        self.api.set_ptz_velocity(zoom=zoom)

    def look_at(self, x, y, image_width, image_height):
        self.api.look_at(x, y, image_width, image_height)

    def set_autofocus(self, use, send_parameter_updates=True):
        self.api.use_autofocus(use)
        self.autofocus = use
        self._send_parameter_update("autofocus", use)

    def set_focus(self, focus):
        focus = self.api.set_focus(focus)
        self.autofocus = False
        self.focus = focus
        self._send_parameter_update("focus", focus)
        self._send_parameter_update("autofocus", False)
        return focus

    def adjust_focus(self, amount):
        amount = self.api.adjust_focus(amount)
        self.focus += amount
        self._send_parameter_update("focus", self.focus)
        self._send_parameter_update("autofocus", False)
        return amount

    def set_focus_velocity(self, velocity):
        self.api.set_focus_velocity(velocity)
        # TODO self.focus updating

    def set_autoiris(self, use):
        self.api.use_autoiris(use)
        self.autoiris = use
        self._send_parameter_update("autoiris", use)

    def set_iris(self, iris):
        iris = self.api.set_iris(iris)
        self.iris = iris
        self.autoiris = False
        self._send_parameter_update("iris", iris)
        self._send_parameter_update("autoiris", False)
        return iris

    def adjust_iris(self, amount):
        amount = self.api.adjust_iris(amount)
        self.iris += amount
        self._send_parameter_update("iris", self.iris)
        self._send_parameter_update("autoiris", False)
        return amount

    def set_iris_velocity(self, velocity):
        self.api.set_iris_velocity(velocity)
        # TODO self.iris updating

    def set_brightness(self, brightness):
        brightness = self.api.set_brightness(brightness)
        self.brightness = brightness
        self._send_parameter_update("brightness", brightness)
        return brightness

    def adjust_brightness(self, amount):
        amount = self.api.adjust_brightness(amount)
        self.brightness += amount
        self._send_parameter_update("brightness", self.brightness)
        return amount

    def set_brightness_velocity(self, velocity):
        self.api.set_brightness_velocity(velocity)
        # TODO self.brightness updating

    def use_backlight_compensation(self, use):
        if self.autoiris:  # the compensation needs autoiris to be active
            self.api.use_backlight_compensation(use)
            self.backlight = use
        else:
            use = False

        self._send_parameter_update("backlight", use)

        return use

    def set_ir_cut_filter_auto(self, use):
        use = (None if use else True)
        # auto IR filter requires autoiris to be active
        if self.autoiris or use is not None:
            self.api.use_ir_cut_filter(use)
            self.ir_cut_filter = use
        else:
            use = True

        self._send_parameter_update("ircutfilter", "auto" if (use is None) else "on")

        return use

    def set_ir_cut_filter_use(self, use):
        self.api.use_ir_cut_filter(use)
        self.ir_cut_filter = use

        self._send_parameter_update("ircutfilter", "on" if use else "off")

        return use

    def _reconfigure(self, config, level, subname):
        try:
            if config.autofocus != self.autofocus:
                self.set_autofocus(config.autofocus)
        except (IOError, ValueError, RuntimeError) as e:
            rospy.logwarn("Could not apply dynamic reconfigure for autofocus. Cause: %s" % repr(e))

        try:
            if config.focus != self.focus:
                self.set_focus(config.focus)
        except (IOError, ValueError, RuntimeError) as e:
            rospy.logwarn("Could not apply dynamic reconfigure for focus. Cause: %s" % repr(e))

        try:
            if config.autoiris != self.autoiris:
                self.set_autoiris(config.autoiris)
        except (IOError, ValueError, RuntimeError) as e:
            rospy.logwarn("Could not apply dynamic reconfigure for autoiris. Cause: %s" % repr(e))

        try:
            if config.iris != self.iris:
                self.set_iris(config.iris)
        except (IOError, ValueError, RuntimeError) as e:
            rospy.logwarn("Could not apply dynamic reconfigure for iris. Cause: %s" % repr(e))

        try:
            if config.brightness != self.brightness:
                self.set_brightness(config.brightness)
        except (IOError, ValueError, RuntimeError) as e:
            rospy.logwarn("Could not apply dynamic reconfigure for brightness. Cause: %s" % repr(e))

        try:
            if config.backlight != self.backlight:
                self.use_backlight_compensation(config.backlight)
        except (IOError, ValueError, RuntimeError) as e:
            rospy.logwarn("Could not apply dynamic reconfigure for backlight compenstaion. Cause: %s" % repr(e))

        try:
            ir = None if config.ircutfilter == "auto" else (config.ircutfilter == "on")
            if ir != self.ir_cut_filter:
                if ir is None or self.ir_cut_filter is None:
                    self.set_ir_cut_filter_auto(config.ircutfilter)
                if ir is not None:
                    self.set_ir_cut_filter_use(ir)
        except (IOError, ValueError, RuntimeError) as e:
            rospy.logwarn("Could not apply dynamic reconfigure for IR cut filter. Cause: %s" % repr(e))

        config.autofocus = self.autofocus
        config.focus = self.focus
        config.autoiris = self.autoiris
        config.iris = self.iris
        config.brightness = self.brightness
        config.backlight = self.backlight
        config.ircutfilter = ("auto" if self.ir_cut_filter is None else ("on" if self.ir_cut_filter else "off"))

        return config

    # Helper functions

    def _send_parameter_update(self, parameter, value):
        config = Config()
        if type(value) == bool:
            config.bools = [BoolParameter(parameter, value)]
        elif type(value) == int:
            config.ints = [IntParameter(parameter, value)]
        elif type(value) == float:
            config.doubles = [DoubleParameter(parameter, value)]
        elif isinstance(value, basestring):
            config.strs = [StrParameter(parameter, value)]

        self._parameter_updates_publisher.publish(config)
        self._parameter_updates_publisher_deprecated.publish(config)
        rospy.set_param("axis_ptz/" + parameter, value)
        rospy.set_param("axis_ptz_driver/" + parameter, value)

    def _apply_flip_and_mirror_absolute(self, pan, tilt):
        if self.flip_vertically:
            tilt = -tilt
        if self.flip_horizontally:
            pan = 180 - pan
        if self.mirror_horizontally:
            pan = -pan

        return pan, tilt

    def _apply_flip_and_mirror_relative(self, pan, tilt):
        if self.flip_vertically:
            tilt = -tilt
        if self.flip_horizontally:
            pan = -pan
        if self.mirror_horizontally:
            pan = -pan

        return pan, tilt

    def _apply_flip_and_mirror_velocity(self, pan, tilt):
        if self.flip_vertically:
            tilt = -tilt
        if self.flip_horizontally:
            pan = -pan
        if self.mirror_horizontally:
            pan = -pan

        return pan, tilt

    @staticmethod
    def _call_with_simple_message_data(func):
        return lambda (msg): func(msg.data)

    @staticmethod
    def _call_with_ptz_message(func):
        return lambda (msg): func(msg.pan, msg.tilt, msg.zoom)

    @staticmethod
    def _call_with_pt_message(func):
        return lambda (msg): func(msg.pan, msg.tilt)

    @staticmethod
    def _call_with_pir_message(func):
        return lambda (msg): func(msg.x, msg.y, msg.image_width, msg.image_height)
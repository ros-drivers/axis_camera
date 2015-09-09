import rospy
from std_msgs.msg import Bool, Float32, Int32

from axis_camera.msg import PTZ


class AxisCameraController(object):
    def __init__(self, api):
        super(AxisCameraController, self).__init__()

        self.api = api

        if self.api.has_capabilities('AbsolutePan', 'AbsoluteTilt', 'AbsoluteZoom'):
            self.absolute_ptz_subscriber = rospy.Subscriber("control/pan_tilt_zoom/absolute", PTZ, self.absolute_ptz_callback, queue_size=100)

        if self.api.has_capabilities('RelativePan', 'RelativeTilt', 'RelativeZoom'):
            self.relative_ptz_subscriber = rospy.Subscriber("control/pan_tilt_zoom/relative", PTZ, self.relative_ptz_callback, queue_size=100)

        if self.api.has_capabilities('ContinuousPan', 'ContinuousTilt', 'ContinuousZoom'):
            self.velocity_ptz_subscriber = rospy.Subscriber("control/pan_tilt_zoom/velocity", PTZ, self.velocity_ptz_callback, queue_size=100)

        if self.api.has_capabilities('AbsolutePan', 'AbsoluteTilt'):
            self.absolute_pan_tilt_subscriber = rospy.Subscriber("control/pan_tilt/absolute", PTZ, self.absolute_pan_tilt_callback, queue_size=100)

        if self.api.has_capabilities('RelativePan', 'RelativeTilt'):
            self.relative_pan_tilt_subscriber = rospy.Subscriber("control/pan_tilt/relative", PTZ, self.relative_pan_tilt_callback, queue_size=100)

        if self.api.has_capabilities('ContinuousPan', 'ContinuousTilt'):
            self.velocity_pan_tilt_subscriber = rospy.Subscriber("control/pan_tilt/velocity", PTZ, self.velocity_pan_tilt_callback, queue_size=100)

        if self.api.has_capability('AbsolutePan'):
            self.absolute_pan_subscriber = rospy.Subscriber("control/pan/absolute", Float32, self.absolute_pan_callback, queue_size=100)

        if self.api.has_capability('AbsoluteTilt'):
            self.absolute_tilt_subscriber = rospy.Subscriber("control/tilt/absolute", Float32, self.absolute_tilt_callback, queue_size=100)

        if self.api.has_capability('AbsoluteZoom'):
            self.absolute_zoom_subscriber = rospy.Subscriber("control/zoom/absolute", Int32, self.absolute_zoom_callback, queue_size=100)

        if self.api.has_capability('RelativePan'):
            self.relative_pan_subscriber = rospy.Subscriber("control/pan/relative", Float32, self.relative_pan_callback, queue_size=100)

        if self.api.has_capability('RelativeTilt'):
            self.relative_tilt_subscriber = rospy.Subscriber("control/tilt/relative", Float32, self.relative_tilt_callback, queue_size=100)

        if self.api.has_capability('RelativeZoom'):
            self.relative_zoom_subscriber = rospy.Subscriber("control/zoom/relative", Int32, self.relative_zoom_callback, queue_size=100)

        if self.api.has_capability('ContinuousPan'):
            self.velocity_pan_subscriber = rospy.Subscriber("control/pan/velocity", Int32, self.velocity_pan_callback, queue_size=100)

        if self.api.has_capability('ContinuousTilt'):
            self.velocity_tilt_subscriber = rospy.Subscriber("control/tilt/velocity", Int32, self.velocity_tilt_callback, queue_size=100)

        if self.api.has_capability('ContinuousZoom'):
            self.velocity_zoom_subscriber = rospy.Subscriber("control/zoom/velocity", Int32, self.velocity_zoom_callback, queue_size=100)

        if self.api.has_capability('AutoFocus'):
            self.autofocus_subscriber = rospy.Subscriber("control/camera/focus/auto", Bool, self.autofocus_callback, queue_size=100)

        if self.api.has_capability('AbsoluteFocus'):
            self.focus_subscriber = rospy.Subscriber("control/camera/focus/absolute", Float32, self.focus_callback, queue_size=100)

        if self.api.has_capability('RelativeFocus'):
            self.focus_relative_subscriber = rospy.Subscriber("control/camera/focus/relative", Float32, self.focus_relative_callback, queue_size=100)

        if self.api.has_capability('ContinuousFocus'):
            self.focus_velocity_subscriber = rospy.Subscriber("control/camera/focus/velocity", Float32, self.focus_velocity_callback, queue_size=100)

        if self.api.has_capability('AutoIris'):
            self.autoiris_subscriber = rospy.Subscriber("control/camera/iris/auto", Bool, self.autoiris_callback, queue_size=100)

        if self.api.has_capability('AbsoluteIris'):
            self.iris_subscriber = rospy.Subscriber("control/camera/iris/absolute", Float32, self.iris_callback, queue_size=100)

        if self.api.has_capability('RelativeIris'):
            self.iris_relative_subscriber = rospy.Subscriber("control/camera/iris/relative", Float32, self.iris_relative_callback, queue_size=100)

        if self.api.has_capability('ContinuousIris'):
            self.iris_velocity_subscriber = rospy.Subscriber("control/camera/iris/velocity", Float32, self.iris_velocity_callback, queue_size=100)

        if self.api.has_capability('AbsoluteBrightness'):
            self.brightness_subscriber = rospy.Subscriber("control/camera/brightness/absolute", Float32, self.brightness_callback, queue_size=100)

        if self.api.has_capability('RelativeBrightness'):
            self.brightness_relative_subscriber = rospy.Subscriber("control/camera/brightness/relative", Float32, self.brightness_relative_callback, queue_size=100)

        if self.api.has_capability('ContinuousIris'):
            self.brightness_velocity_subscriber = rospy.Subscriber("control/camera/brightness/velocity", Float32, self.brightness_velocity_callback, queue_size=100)

        if self.api.has_capability('BackLight'):
            self.use_backlight_subscriber = rospy.Subscriber("control/camera/backlight_compensation", Bool, self.backlight_compensation_callback, queue_size=100)

        if self.api.has_capabilities('IrCutFilter', 'AutoIrCutFilter'):
            self.use_ir_cut_filter_auto_subscriber = rospy.Subscriber("control/camera/ir_cut_filter/auto", Bool, self.ir_cut_filter_auto_callback, queue_size=100)

        if self.api.has_capabilities('IrCutFilter'):
            self.use_ir_cut_filter_use_subscriber = rospy.Subscriber("control/camera/ir_cut_filter/use", Bool, self.ir_cut_filter_use_callback, queue_size=100)

    def absolute_ptz_callback(self, ptz_message):
        self.api.move_ptz_absolute(ptz_message.pan, ptz_message.tilt, ptz_message.zoom)

    def relative_ptz_callback(self, ptz_message):
        self.api.move_ptz_relative(ptz_message.pan, ptz_message.tilt, ptz_message.zoom)

    def velocity_ptz_callback(self, ptz_message):
        self.api.set_ptz_velocity(ptz_message.pan, ptz_message.tilt, ptz_message.zoom)

    def absolute_pan_tilt_callback(self, pan_tilt):
        self.api.move_ptz_absolute(pan=pan_tilt.pan, tilt=pan_tilt.tilt)

    def relative_pan_tilt_callback(self, pan_tilt):
        self.api.move_ptz_relative(pan=pan_tilt.pan, tilt=pan_tilt.tilt)

    def velocity_pan_tilt_callback(self, pan_tilt):
        self.api.set_ptz_velocity(pan=pan_tilt.pan, tilt=pan_tilt.tilt)

    def absolute_pan_callback(self, pan):
        self.api.move_ptz_absolute(pan=pan.data)

    def absolute_tilt_callback(self, tilt):
        self.api.move_ptz_absolute(tilt=tilt.data)

    def absolute_zoom_callback(self, zoom):
        self.api.move_ptz_absolute(zoom=zoom.data)

    def relative_pan_callback(self, pan):
        self.api.move_ptz_relative(pan=pan.data)

    def relative_tilt_callback(self, tilt):
        self.api.move_ptz_relative(tilt=tilt.data)

    def relative_zoom_callback(self, zoom):
        self.api.move_ptz_relative(zoom=zoom.data)

    def velocity_pan_callback(self, pan):
        self.api.set_ptz_velocity(pan=pan.data)

    def velocity_tilt_callback(self, tilt):
        self.api.set_ptz_velocity(tilt=tilt.data)

    def velocity_zoom_callback(self, zoom):
        self.api.set_ptz_velocity(zoom=zoom.data)

    def autofocus_callback(self, use):
        self.api.use_autofocus(use.data)

    def focus_callback(self, focus):
        self.api.set_focus(focus.data)

    def focus_relative_callback(self, amount):
        self.api.adjust_focus(amount.data)

    def focus_velocity_callback(self, velocity):
        self.api.set_focus_velocity(velocity.data)

    def autoiris_callback(self, use):
        self.api.use_autoiris(use.data)

    def iris_callback(self, iris):
        self.api.set_iris(iris.data)

    def iris_relative_callback(self, amount):
        self.api.adjust_iris(amount.data)

    def iris_velocity_callback(self, velocity):
        self.api.set_iris_velocity(velocity.data)

    def brightness_callback(self, brightness):
        self.api.set_brightness(brightness.data)

    def brightness_relative_callback(self, amount):
        self.api.adjust_brightness(amount.data)

    def brightness_velocity_callback(self, velocity):
        self.api.set_brightness_velocity(velocity.data)

    def backlight_compensation_callback(self, use):
        self.api.use_backlight_compenstaion(use.data)

    def ir_cut_filter_auto_callback(self, use):
        self.api.use_ir_cut_filter(None if use.data else True)

    def ir_cut_filter_use_callback(self, use):
        self.api.use_ir_cut_filter(use.data)

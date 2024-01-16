import rclpy
from rclpy.node import Node
import requests, requests.auth
import threading
import os
import subprocess
from std_msgs.msg import Bool, Int32, Float32, String
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from axis_camera_ros2.cam_info_utils import CameraInfoManager, genCameraName
from axis_msgs.srv import SetInt, SetInt
from std_srvs.srv import SetBool
import urllib.request, urllib.error, urllib.parse
from urllib.parse import urlparse

class StreamThread(threading.Thread):
    def __init__(self, axis):
        threading.Thread.__init__(self)
        self.axis = axis
        self.axis.get_logger().warning("stream thread")
        self.daemon = True
        self.timeoutSeconds = 2.5
        self.i = 0

    def waitForHost(self):
        '''Wait until the host is actually online before we try to contact it.
        This reduces http related errors'''

        # ping syntax is different on Windows than Linux, so set the command accordingly
        if os.name == 'nt':
            cmd = f"ping -W 5 -n 1 {self.axis.hostname}".split()
        else:
            cmd = f"ping -W 5 -c 1 {self.axis.hostname}".split()

        self.axis.get_logger().info(f"Waiting until {self.axis.hostname} is online...")
        host_alive = subprocess.call(cmd) == 0
        rate = self.axis.create_rate(1)
        while not host_alive:
            rate.sleep()
            host_alive = subprocess.call(cmd) == 0

        self.axis.get_logger().info(f"{self.axis.hostname} is now online")

    def run(self):
        self.waitForHost()

        while(True):
            self.stream()

    def stream(self):
        while(True):
            self.formURL()
            #self.formURLrtsp()
            self.authenticate()
            
            if self.openURL():
            #if self.openURLrtps():
                self.publishFramesContinuously()

            #rclpy.sleep(2) # if stream stays intact we shouldn't get to this

    def formURL(self):
        self.url = 'http://%s/mjpg/video.mjpg' % self.axis.hostname
        self.url += "?fps=%d&resolution=%dx%d" % (self.axis.fps, self.axis.width,
                                                            self.axis.height)

        # support for Axis F34 multicamera switch
        if (self.axis.camera != 0):
            self.url += "&camera=%s" % str(self.axis.camera)

        self.axis.get_logger().debug('opening ' + str(self.axis))
    
    def formURLrtsp(self):
        self.url = 'rtsp://%s:%s@%s/axis-media/media.amp' % (self.axis.username, self.axis.password, self.axis.hostname)
        self.url += "?videocodec=h264&fps=%d&resolution=%dx%d" % (self.axis.fps, self.axis.width,
                                                            self.axis.height)

        # support for Axis F34 multicamera switch
        if (self.axis.camera != 0):
            self.url += "&camera=%s" % str(self.axis.camera)

        self.axis.get_logger().debug('opening ' + str(self.axis))

    def authenticate(self):
        '''only try to authenticate if user/pass configured.  I have not
        used this method (yet).'''
        if self.axis.password != '' and self.axis.username != '':
            # create a password manager
            password_mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()

            # Add the username and password, use default realm.
            top_level_url = "http://" + self.axis.hostname
            password_mgr.add_password(None, top_level_url, self.axis.username,
                                                            self.axis.password)
            if self.axis.use_encrypted_password:
                handler = urllib.request.HTTPDigestAuthHandler(password_mgr)
            else:
                handler = urllib.request.HTTPBasicAuthHandler(password_mgr)

            # create "opener" (OpenerDirector instance)
            opener = urllib.request.build_opener(handler)

            # ...and install it globally so it can be used with urlopen.
            urllib.request.install_opener(opener)

    def openURL(self):
        '''Open connection to Axis camera using http'''
        try:
            self.fp = urllib.request.urlopen(self.url, timeout=self.timeoutSeconds)
            #print(self.fp)
            return(True)
        except urllib.error.URLError as e:
            self.axis.get_logger().warning('Error opening URL %s' % (self.url) +
                            'Possible timeout.  Looping until camera appears')
            return(False)

    def openURLrtps(self):
        '''Open connection to Axis camera using rtps'''
        self.fp = urlparse(self.url)
        #self.axis.get_logger().warning(self.fp)
        return(True)

    def publishFramesContinuously(self):
        '''Continuous loop to publish images'''
        while(True):
            try:
                self.findBoundary()
                self.getImage()
                self.publishMsg()
                #self.publishMsgImg()
                self.publishCameraInfoMsg()

            except:
                self.axis.get_logger().warning('Timed out while trying to get message.')
                break

    def findBoundary(self):
        '''The string "--myboundary" is used to denote the start of an image in
        Axis cameras'''
        while(True):
            boundary = self.fp.readline()
            if boundary == b'--myboundary\r\n':
                #self.axis.get_logger().warning(boundary)
                break

    def getImage(self):
        '''Get the image header and image itself'''
        self.getHeader()
        self.getImageData()

    def getHeader(self):
        self.header = {}
        while(True):
            line = self.fp.readline()
            if line == b'\r\n':
                #print(line) #b'\r\n'
                break
            line = line.strip()
            parts = line.split(b": ", 1)
            try:
                self.header[parts[0]] = parts[1]
            except:
                self.axis.get_logger().warning('Problem encountered with image header.  Setting '
                                                    'content_length to zero')
                self.header[b'Content-Length'] = 0 # set content_length to zero if
                                            # there is a problem reading header
        self.content_length = int(self.header[b'Content-Length'])

    def getImageData(self):
        '''Get the binary image data itself (ie. without header)'''
        if self.content_length>0:
            self.img = self.fp.read(self.content_length)
            self.fp.readline() # Read terminating \r\n and do nothing with it
            #print(self.img) # ok data is in self.img

    def publishMsg(self):
        '''Publish jpeg image as a ROS message'''
        self.msg = CompressedImage()
        self.msg.header.stamp = self.axis.get_clock().now().to_msg()
        self.msg.header.frame_id = self.axis.frame_id
        self.msg.format = "jpeg"
        self.msg.data = self.img
        self.axis.pub.publish(self.msg)
        #self.axis.get_logger().info('%d Images Published' % self.i)
        self.i += 1
    
    def publishMsgImg(self):
        '''Publish image as a ROS message'''
        self.msg = Image()
        self.msg.header.stamp = self.axis.get_clock().now()
        self.msg.header.frame_id = self.axis.frame_id
        self.msg.height = self.axis.height
        self.msg.width = self.axis.width
        self.msg.encoding = "RGB8"
        self.msg.data = self.img_raw
        self.axis.pub.publish(self.msg)

    def publishCameraInfoMsg(self):
        '''Publish camera info manager message'''
        cimsg = self.axis.cinfo.getCameraInfo()
        cimsg.header.stamp = self.msg.header.stamp
        cimsg.header.frame_id = self.axis.frame_id
        cimsg.width = self.axis.width
        cimsg.height = self.axis.height
        #print(cimsg)
        self.axis.caminfo_pub.publish(cimsg)

class Axis(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
        #self.publisher_ = self.create_publisher(String, 'topic', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0
        self.get_logger().warning("axis node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('hostname', rclpy.Parameter.Type.STRING),
                ('username', rclpy.Parameter.Type.STRING),
                ('password', rclpy.Parameter.Type.STRING),
                ('width', rclpy.Parameter.Type.INTEGER),
                ('height', rclpy.Parameter.Type.INTEGER),
                ('fps', rclpy.Parameter.Type.INTEGER),
                ('frame_id', 'axis_camera_link'),
                ('camera_info_url', ''),
                ('use_encrypted_password', rclpy.Parameter.Type.BOOL),
                ('camera', rclpy.Parameter.Type.INTEGER),
                ('ir', False),
                ('defog', False),
                ('wiper', False),
                ('ptz', False) 
            ]
        )

        self.hostname = self.get_parameter('hostname').value
        print(self.hostname)
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        self.camera_info_url = self.get_parameter('camera_info_url').value
        self.use_encrypted_password = self.get_parameter('use_encrypted_password').value
        self.camera = self.get_parameter('camera').value
        self.ir = self.get_parameter('ir').value
        self.defog = self.get_parameter('defog').value
        self.wiper = self.get_parameter('wiper').value
        self.ptz = self.get_parameter('ptz').value

        #self.hostname = args['hostname']
        #self.username = args['username']
        #self.password = args['password']
        #self.width = args['width']
        #self.height = args['height']
        #self.fps = args['fps']
        #self.frame_id = args['frame_id']
        #self.camera_info_url = args['camera_info_url']
        #self.use_encrypted_password = args['use_encrypted_password']
        #self.camera = args['camera']

        self.use_legacy_ir_url = False

        self.http_headers = {
            'User-Agent': 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/111.0.0.0 Safari/537.36',
            'From': f'http://{self.hostname}'
        }
        if self.use_encrypted_password:
            self.http_auth = requests.auth.HTTPDigestAuth(self.username, self.password)
        else:
            self.http_auth = requests.auth.HTTPBasicAuth(self.username, self.password)
        self.http_timeout = (3, 5)

        # generate a valid camera name based on the hostname
        self.cname = genCameraName(self.hostname)
        self.cinfo = CameraInfoManager(self, cname = self.cname,
                                                   url = self.camera_info_url,
                                                   namespace = '/axis_cam')
        self.cinfo.loadCameraInfo()         # required before getCameraInfo()
        self.st = None

        #self.pub = self.create_publisher(Image, 'image_raw', 1)
        self.pub = self.create_publisher(CompressedImage, 'image_raw/compressed', 1)
        self.caminfo_pub = self.create_publisher(CameraInfo, 'camera_info', 1)

        self.last_camera_position = None
        self.iris = -1
        self.focus = -1
        self.brightness = -1
        self.autofocus = False
        self.autoiris = False
        self.state_thread = threading.Thread(target=self.statePubThread)
        self.state_thread.start()

        self.set_iris_srv = self.create_service(SetInt, 'set_iris', self.handle_set_iris)
        self.set_focus_srv = self.create_service(SetInt, 'set_focus', self.handle_set_focus)
        self.set_brightness_srv = self.create_service(SetInt, 'set_brightness', self.handle_set_brightness)
        self.set_autofocus_srv = self.create_service(SetBool, 'set_autofocus', self.handle_set_autofocus)
        self.set_autoiris_srv = self.create_service(SetBool, 'set_autoiris', self.handle_set_autoiris)

        self.iris_pub = self.create_publisher(Int32, 'iris', 1)
        self.focus_pub = self.create_publisher(Int32, 'focus', 1)
        self.brightness_pub = self.create_publisher(Int32, 'brightness', 1)
        self.autofocus_pub = self.create_publisher(Bool, 'autofocus', 1)
        self.autoiris_pub = self.create_publisher(Bool, 'autoiris', 1)

        # The Axis Q62 series supports a night-vision mode with an active IR illuminator
        # If this option is enabled, add the necessary services and topics
        if self.ir:
            self.ir_on = False
            self.ir_on_off_srv = self.create_service(SetBool, 'set_ir_on', self.handle_toggle_ir)
            self.ir_on_pub = self.create_publisher(Bool, 'ir_on', 1, True) # queue_size=1, latch=True
            self.handle_toggle_ir(SetBool.Request(False))

        # The Axis Q62 series is equipped with a wiper on the camera lens
        # If this option is enabled, add the necessary services and topics
        if self.wiper:
            self.wiper_on_pub_thread = None
            self.wiper_start_at = self.get_clock().now()
            self.wiper_on = False
            self.wiper_on_off_srv = self.create_service(SetBool, 'set_wiper_on', self.handle_toggle_wiper)
            self.wiper_on_pub = self.create_publisher(Bool, 'wiper_on', 1, True) # queue_size=1, latch=True
            self.handle_toggle_wiper(SetBool.Request(False))

        # The Axis Q62 series is equipped with a defogger
        # If this option is enabled, add the necessary services and topics
        if self.defog:
            self.defog_on = False
            self.defog_on_off_srv = self.create_service(SetBool, 'set_defog_on', self.handle_toggle_defog)
            self.defog_on_pub = self.create_publisher(Bool, 'defog_on', 1, True) # queue_size=1, latch=True
            self.handle_toggle_defog(SetBool.Request(False))

        self.msg_thread = StreamThread(self)
        self.msg_thread.start()

    def __str__(self):
        """Return string representation."""
        return(self.hostname + ',' + self.username + ',' + self.password +
                       '(' + str(self.width) + 'x' + str(self.height) + ')')

    def statePubThread(self):
        """Queries the state of the Axis hardware using the VAPIX protocol and publishes the camera's state at 1Hz
        """
        
        rate = self.create_rate(1) # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
        while rclpy.ok():
            rate.sleep()
            self.last_camera_position = self.queryCameraPosition()

            if self.last_camera_position is not None:
                if "iris" in self.last_camera_position.keys():
                    self.iris = self.last_camera_position["iris"]
                if "focus" in self.last_camera_position.keys():
                    self.focus = self.last_camera_position["focus"]
                if "brightness" in self.last_camera_position.keys():
                    self.brightness = self.last_camera_position["brightness"]
                if "autofocus" in self.last_camera_position.keys():
                    self.autofocus = self.last_camera_position["autofocus"]
                if "autoiris" in self.last_camera_position.keys():
                    self.autoiris = self.last_camera_position["autoiris"]

                self.iris_pub.publish(self.iris)
                self.focus_pub.publish(self.focus)
                self.brightness_pub.publish(self.brightness)
                self.autofocus_pub.publish(self.autofocus)
                self.autoiris_pub.publish(self.autoiris)

    def queryCameraPosition(self):
        '''Using Axis VAPIX protocol, query the state of the camera

        This will return the following information, with some variation depending on what exactly the camera supports:
            - pan (degrees)
            - tilt (degrees)
            - zoom (1-9999)
            - iris
            - focus
            - brightness
            - autofocus
            - autoiris

        @return A dict with keys & values corresponding to the data provided by the camera, or None if the data
                could not be acquired
        '''
        
        queryParams = { 'query':'position' }
        new_camera_position = None
        try:
            url = f"http://{self.hostname}/axis-cgi/com/ptz.cgi?{urllib.parse.urlencode(queryParams)}"
            resp = requests.get(url, auth=self.http_auth, timeout=self.http_timeout, headers=self.http_headers)

            if resp.status_code == requests.status_codes.codes.ok:
                # returns a string of the form
                #   pan=-0.01
                #   tilt=-45.03
                #   zoom=1
                #   iris=5748
                #   focus=4642
                #   brightness=4999
                #   autofocus=on
                #   autoiris=on
                new_camera_position = {
                }
                body = resp.text.split()
                for row in body:
                    if '=' in row:
                        (key, value) = row.split('=')
                        key = key.strip()
                        value = value.strip()

                        # convert the value into the appropriate Python type, since it's just a string now
                        # use a regex to check for int/float values, and use standard YAML keywords for booleans
                        # otherwise, just save the value as a string
                        float_re = r'^[-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?$'
                        int_re = r'^[-+]?[0-9]+([eE][-+]?[0-9]+)?$'
                        if bool(re.match(int_re, value)):
                            value = int(value)
                        elif bool(re.match(float_re, value)):
                            value = float(value)
                        elif value.lower() == "on" or value.lower() == "true" or value.lower() == "yes":
                            value = True
                        elif value.lower() == "off" or value.lower() == "false" or value.lower() == "no":
                            value = False

                        new_camera_position[key] = value
            else:
                raise Exception(f"HTTP Error querying the camera position: {resp.status_code}")

        except Exception as e:
            exception_error_str = "Exception: '" + str(e) + "' when querying the url: http://" + \
                                  self.hostname + "/axis-cgi/com/ptz.cgi?%s" % urllib.parse.urlencode(queryParams)
            self.get_logger().warning(exception_error_str)
            new_camera_position = None

        return new_camera_position

    def handle_set_iris(self, req):
        get_url = f"http://{self.hostname}/axis-cgi/com/ptz.cgi?iris={req.data}"
        resp = SetInt.Response()
        try:
            http_resp = requests.get(get_url,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout)

            resp.success = http_resp.status_code == requests.status_codes.codes.ok
        except Exception as err:
            self.get_logger().warning("Error setting iris: {err}")
            resp.success = False

        return resp

    def handle_set_focus(self, req, tmp):
        get_url = f"http://{self.hostname}/axis-cgi/com/ptz.cgi?focus={req.data}"
        print(get_url)
        resp = SetInt.Response()
        try:
            http_resp = requests.get(get_url,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout)

            resp.success = http_resp.status_code == requests.status_codes.codes.ok
        except Exception as err:
            self.get_logger().warning("Error setting focus: {err}")
            resp.success = False

        return resp

    def handle_set_brightness(self, req):
        get_url = f"http://{self.hostname}/axis-cgi/com/ptz.cgi?brightness={req.data}"
        resp = SetInt.Response()
        try:
            http_resp = requests.get(get_url,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout)

            resp.success = http_resp.status_code == requests.status_codes.codes.ok
        except Exception as err:
            self.get_logger().warning("Error setting brightness: {err}")
            resp.success = False

        return resp

    def handle_set_autofocus(self, req):
        get_url = f"http://{self.hostname}/axis-cgi/com/ptz.cgi?autofocus={'on' if req.data else 'off'}"
        resp = SetBool.Response()
        try:
            http_resp = requests.get(get_url,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout)

            resp.success = http_resp.status_code == requests.status_codes.codes.ok
        except Exception as err:
            self.get_logger().warning("Error setting autofocus: {err}")
            resp.success = False

        return resp

    def handle_set_autoiris(self, req):
        get_url = f"http://{self.hostname}/axis-cgi/com/ptz.cgi?autoiris={'on' if req.data else 'off'}"
        resp = SetBoolResponse()
        try:
            http_resp = requests.get(get_url,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout)

            resp.success = http_resp.status_code == requests.status_codes.codes.ok
        except Exception as err:
            self.get_logger().warning("Error setting autoiris: {err}")
            resp.success = False

        return resp
    
    def peer_subscribe(self, topic_name, topic_publish, peer_publish): # in ROS1, peer_subscribe is a callback when a peer has subscribed from a topic -> example: rqt subscribe to the topic, then publish
        self.get_logger().warning("peer_subscribe")
        '''Lazy-start the image-publisher.'''
        if self.st is None:
            self.st = StreamThread(self)
            self.st.start()

    def handle_toggle_ir(self, req):
        """Turn the IR mode on/off (if supported)"""
        resp = SetBool.Response()
        resp.success = True
        on_off = {
            True: "on",
            False: "off"
        }
        try:
            # Set the IR led on/off as needed
            if req.data:
                post_data = '{"apiVersion": "1.0", "method": "enableLight", "params": {"lightID": "led0"}}'
            else:
                post_data = '{"apiVersion": "1.0", "method": "disableLight", "params": {"lightID": "led0"}}'
            http_resp = requests.post(f"http://{self.hostname}/axis-cgi/lightcontrol.cgi",  post_data,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout)

            if http_resp.status_code != requests.status_codes.codes.ok:
                raise Exception(f"HTTP Error setting IR illuminator: {http_resp.status_code}")

            # Enable/disable the IR filter
            # By default we try the new URL first, falling back to the legacy one if that fails
            http_resp = self.enable_disable_ir_filter(not req.data)

            if http_resp.status_code != requests.status_codes.codes.ok and not self.use_legacy_ir_url:
                self.get_logger().warning("Failed to set IR filter mode using new URL; falling back to legacy URL")
                self.use_legacy_ir_url = True
                http_resp =  self.enable_disable_ir_filter(not req.data)

            if http_resp.status_code != requests.status_codes.codes.ok:
                raise Exception(f"HTTP Error setting IR filter: {http_resp.status_code}")

            resp.message = f"IR mode is {on_off[req.data]}"
            self.ir_on = req.data

            self.ir_on_pub.publish(self.ir_on)
        except Exception as err:
            self.get_logger().warning(f"Failed to set IR mode: {err}")
            ok = False
            resp.message = str(err)

        return resp

    def enable_disable_ir_filter(self, enable):
        """Sends the needed GET request to the camera to enable/disable the IR filter (if supported)

        At some point in 2022-2023 the URL to access this changed, but we do our best to support both URLs

        @param enable  True/False indicating whether or not the IR filter should be enabled

        @return The HTTP response received from the camera upon making the GET request
        """
        if self.use_legacy_ir_url:
            if enable:
                get_url = f"http://{self.hostname}/axis-cgi/param.cgi?action=update&PTZ.Various.V1.IrCutFilter=on&timestamp={int(time.time())}"
            else:
                get_url = f"http://{self.hostname}/axis-cgi/param.cgi?action=update&PTZ.Various.V1.IrCutFilter=off&timestamp={int(time.time())}"
        else:
            if enable:
                get_url = f"http://{self.hostname}/axis-cgi/param.cgi?action=update&ImageSource.I0.DayNight.IrCutFilter=yes&timestamp={int(time.time())}"
            else:
                get_url = f"http://{self.hostname}/axis-cgi/param.cgi?action=update&ImageSource.I0.DayNight.IrCutFilter=no&timestamp={int(time.time())}"
        http_resp = requests.get(get_url,
            auth=self.http_auth,
            headers=self.http_headers,
            timeout=self.http_timeout)

        return http_resp

    def handle_toggle_wiper(self, req):
        """Turn the wiper on/off (if supported)"""
        on_off = {
            True: "on",
            False: "off"
        }

        resp = SetBool.Response()
        resp.success = True
        try:
            if req.data and not self.wiper_on:  # only turn the wiper on if it's not already on
                post_data = '{"apiVersion": "1.0", "context": "lvc_context", "method": "start", "params": {"id": 0, "duration": 10}}'

                # Start a background thread to publish when the wiper turns off
                self.wiper_start_at = self.get_clock().now()
                if self.wiper_on_pub_thread is None:
                    self.wiper_on_pub_thread = threading.Thread(target=self.wiper_timeout_pub_thread)
                    self.wiper_on_pub_thread.start()
            elif not req.data:
                # turn the wiper off manually
                post_data = '{"apiVersion": "1.0", "context": "lvc_context", "method": "stop", "params": {"id": 0}}'
            else:
                # We're turning the wiper on, but it's already on, so don't do anything
                resp.message = "Wiper is already on"
                resp.success = False
                return resp

            http_resp = requests.post(f"http://{self.hostname}/axis-cgi/clearviewcontrol.cgi", post_data,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout)

            if http_resp.status_code != requests.status_codes.codes.ok:
                raise Exception(f"HTTP Error setting wiper: {http_resp.status_code}")

            resp.message = f"Wiper is {on_off[self.wiper_on]}"
            self.wiper_on = req.data

            self.wiper_on_pub.publish(self.wiper_on)
        except Exception as err:
            self.get_logger().warning(f"Failed to set wiper mode: {err}")
            resp.success = False
            resp.message = str(err)
        return resp

    def wiper_timeout_pub_thread(self):
        """A background thread that starts when the wiper is turned on and sleeps for the duration of the wiper operation
        and then publishes when the wiper turns off
        """
        # the wiper turns off automatically after 10s
        WIPER_DURATION = rclpy.Duration(10)
        rate = self.create_rate(1)
        while self.get_clock().now() - self.wiper_start_at > WIPER_DURATION:
            rate.sleep()
        self.wiper_on = False
        self.wiper_on_pub.publish(self.wiper_on)
        self.wiper_on_pub_thread = None

    def handle_toggle_defog(self, req):
        """Turn the defogger on/off (if supported)"""
        on_off = {
            True: "on",
            False: "off"
        }

        resp = SetBool.Response()
        resp.success = True
        try:
            if req.data:
                get_url = f"http://{self.hostname}/axis-cgi/param.cgi?action=update&ImageSource.I0.Sensor.Defog=on&timestamp={int(time.time())}"
            else:
                get_url = f"http://{self.hostname}/axis-cgi/param.cgi?action=update&ImageSource.I0.Sensor.Defog=off&timestamp={int(time.time())}"

            http_resp = requests.get(get_url,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout)

            if http_resp.status_code != requests.status_codes.codes.ok:
                raise Exception(f"HTTP Error setting defogger: {http_resp.status_code}")

            resp.message = f"Defogger is {on_off[self.defog_on]}"
            self.defog_on = req.data

            self.defog_on_pub.publish(self.defog_on)
        except Exception as err:
            self.get_logger().warning(f"Failed to set defogger mode: {err}")
            resp.success = False
            resp.message = str(err)
        return resp


    #def timer_callback(self):
    #    msg = String()
    #    msg.data = 'Hello World: %d' % self.i
    #    self.publisher_.publish(msg)
    #    self.get_logger().info('Publishing: "%s"' % msg.data)
    #    self.i += 1
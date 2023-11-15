"""Copy/paste backup of code we don't currently need, but may need later
"""

def sanitiseFocus(self):
    '''Focus must be: 1<focus<9999.  continuousfocusmove: -100<rfocus<100'''
    if self.speedControl:
        if abs(self.msg.focus)>100.0:
            self.msg.focus = math.copysign(100.0, self.msg.focus)
    else: # position control:
        if self.msg.focus>9999.0:
            self.msg.focus = 9999.0
        elif self.msg.focus < 1.0:
            self.msg.focus = 1.0

def sanitiseBrightness(self):
    '''Brightness must be: 1<brightness<9999.  continuousbrightnessmove must
    be: -100<rbrightness<100.  Note that it appears that the brightness
    cannot be adjusted on the Axis 214PTZ'''
    if self.speedControl:
        if abs(self.msg.brightness) > 100.0:
            self.msg.brightness = math.copysign(100.0, self.msg.brightness)
    else: # position control:
        if self.msg.brightness>9999.0:
            self.msg.brightness = 9999.0
        elif self.msg.brightness<1.0:
            self.msg.brightness = 1.0

def sanitiseIris(self):
    '''Iris value is read only because autoiris has been set to "on"'''
    if self.msg.iris>0.000001:
        rospy.logwarn("Iris value is read-only.")

#!/usr/bin/env python3

"""
  ArbotiX Node: serial connection to an ArbotiX board w/ PyPose/NUKE/ROS
  Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import threading

import rclpy
from rclpy.node import Node

from arbotix_msgs.msg import *
from arbotix_msgs.srv import *

from arbotix_python.arbotix import ArbotiX, ArbotiXException
#from arbotix_python.diff_controller import DiffController      # TODO Uncomment once ported
#from arbotix_python.follow_controller import FollowController  # TODO Uncomment once ported
from arbotix_python.servo_controller import *
#from arbotix_python.linear_controller import *                 # TODO Uncomment once ported
from arbotix_python.publishers import *
#from arbotix_python.io import *                                # TODO Uncomment once ported

# name: [ControllerClass, pause]
#controller_types = { "follow_controller" : FollowController,
#                     "diff_controller"   : DiffController,
#                    "omni_controller"   : OmniController,
#                     "linear_controller" : LinearControllerAbsolute,
#                     "linear_controller_i" : LinearControllerIncremental }

###############################################################################
# Main ROS interface
class ArbotixROS(ArbotiX, Node):
    
    def __init__(self):
        Node.__init__(self, "arbotix")

        pause = False

        # load configurations    
        port = self.declare_parameter("port", "/dev/ttyUSB0").get_parameter_value().string_value
        baud = self.declare_parameter("baud", 115200).get_parameter_value().integer_value
        timeout = self.declare_parameter("timeout", 0.1).get_parameter_value().double_value

        self.rate = self.declare_parameter("rate", 100.0).get_parameter_value().double_value
        self.fake = self.declare_parameter("sim", False).get_parameter_value().bool_value

        self.use_sync_read = self.declare_parameter("sync_read",True).get_parameter_value().bool_value      # use sync read?
        self.use_sync_write = self.declare_parameter("sync_write",True).get_parameter_value().bool_value    # use sync write?

        # setup publishers
        self.diagnostics = DiagnosticsPublisher(self)
        self.joint_state_publisher = JointStatePublisher(self)

        # start an arbotix driver; differ port opening to properly handle connection failures
        if not self.fake:
            ArbotiX.__init__(self, port, baud, timeout, open_port=True)
            self.connectArbotiX()
        else:
            self.get_logger().info("ArbotiX being simulated.")

        # setup joints
        self.joints = dict()
        # Unfortunately, ROS2 doesn't support YAML dictionary so we have to put the list of available joints
        # to iterate over and set up
        joint_names = self.declare_parameter("joint_names").get_parameter_value().string_array_value
        for name in joint_names:
            joint_type = self.declare_parameter("joints." + name + ".type", "dynamixel").get_parameter_value().string_value
            self.get_logger().info("Add " + joint_type + " joint " + name)
            if joint_type == "dynamixel":
                self.joints[name] = DynamixelServo(self, name)
            elif joint_type == "hobby_servo":
                self.joints[name] = HobbyServo(self, name)
            elif joint_type == "calibrated_linear":
                self.joints[name] = LinearJoint(self, name)

        # setup controller
        self.controllers = [ServoController(self, "servos"), ]
        # TODO Uncomment when more controllers are supported
        #controller_names = self.declare_parameter("controller_names").get_parameter_value().string_array_value
        #for name in controller_names:
        #    try:
        #        controller_type = self.declare_parameter("controllers." + name + ".type", "unknown").get_parameter_value().string_value
        #        self.get_logger().info("Add " + controller_type + " controller " + name)
        #        controller = controller_types[controller_type](self, name)
        #        self.controllers.append( controller )
        #        pause = pause or controller.pause
        #    except Exception as e:
        #        self.get_logger().error(str(type(e)) + str(e))

        # TODO Port I/O
        # wait for arbotix to start up (especially after reset)
        #if not self.fake:
        #    if rclpy.has_param("~digital_servos") or rclpy.has_param("~digital_sensors") or rclpy.has_param("~analog_sensors"):
        #        pause = True
        #    if pause:
        #        while self.getDigital(1) == -1 and not rclpy.is_shutdown():
        #            self.get_logger().info("ArbotiX: waiting for response...")
        #            rclpy.sleep(0.25)
        #    self.get_logger().info("ArbotiX connected.")

        for controller in self.controllers:
            controller.startup()

        # TODO Port I/O
        # services for io
        #self.create_subscription(SetupChannel, 'SetupAnalogIn', self.analogInCb)
        #self.create_subscription(SetupChannel, 'SetupDigitalIn', self.digitalInCb)
        #self.create_subscription(SetupChannel, 'SetupDigitalOut', self.digitalOutCb)
        # initialize digital/analog IO streams
        #self.io = dict()
        #if not self.fake:
        #    for v,t in {"digital_servos":DigitalServo,"digital_sensors":DigitalSensor,"analog_sensors":AnalogSensor}.items():
        #        temp = rclpy.get_param("~"+v,dict())
        #        for name in temp.keys():
        #            pin = rclpy.get_param('~'+v+'/'+name+'/pin',1)
        #            value = rclpy.get_param('~'+v+'/'+name+'/value',0)
        #            rate = rclpy.get_param('~'+v+'/'+name+'/rate',10)
        #            leng = rclpy.get_param('~'+v+'/'+name+'/length',1)  # just for analog sensors
        #            if(v != "analog_sensors"):
        #                self.io[name] = t(name, pin, value, rate, self)
        #            else:
        #                self.io[name] = t(name, pin, value, rate, leng, self)

        # Spin in a separate thread
        # See https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
        thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        thread.start()

        r = self.create_rate(self.rate)

        # main loop -- do all the read/write here
        while rclpy.ok():
            try:
                # update controllers
                for controller in self.controllers:
                    controller.update()
    
                # update io
                # TODO Restore this
                # for io in self.io.values():
                #    io.update()
    
                # publish feedback
                self.joint_state_publisher.update(self.joints.values(), self.controllers)
                self.diagnostics.update(self.joints.values(), self.controllers)
            except ArbotiXException as e:
                # We assume this is a serial connection error (as is the only use of
                # ArbotiXException by now...); try to reconnect to solve the issue 
                self.get_logger().error("ArbotiX error: %s", e)
                self.connectArbotiX()

            r.sleep()

        # do shutdown
        for controller in self.controllers:
            controller.shutdown()
        
        # disconnect from the ArbotiX 
        if not self.fake:
          self.closePort()

    def analogInCb(self, req):
        # TODO: Add check, only 1 service per pin
        if not self.fake:
            self.io[req.topic_name] = AnalogSensor(req.topic_name, req.pin, req.value, req.rate, req.leng, self)
        return SetupChannelResponse()

    def digitalInCb(self, req):
        if not self.fake:
            self.io[req.topic_name] = DigitalSensor(req.topic_name, req.pin, req.value, req.rate, self)
        return SetupChannelResponse()

    def digitalOutCb(self, req):
        if not self.fake:
            self.io[req.topic_name] = DigitalServo(req.topic_name, req.pin, req.value, req.rate, self)
        return SetupChannelResponse()

    def connectArbotiX(self):
        iter = 0
        while True:
            try:
                self.openPort()
                self.get_logger().info("Started ArbotiX connection on port " + self._ser.port + ".")
                return
            except ArbotiXException as e:
                if iter%4 == 0:
                    self.get_logger().error("Unable to connect to ArbotiX: %s.", e)
                self.create_rate(0.5).sleep()
                iter += 1


def main(args=None):
    rclpy.init(args=args)
    a = ArbotixROS()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

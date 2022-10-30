#!/usr/bin/env python3

"""
  diagnostics.py - diagnostic output code
  Copyright (c) 2011 Vanadium Labs LLC.  All right reserved.

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

import rclpy
from rclpy.duration import Duration

from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState

class DiagnosticsPublisher:
    """ Class to handle publications of joint_states message. """

    def __init__(self, node):
        self.node = node
        rate = node.declare_parameter("diagnostic_rate", 1.0).get_parameter_value().double_value
        self.t_delta = Duration(seconds=1.0/rate)
        self.t_next = node.get_clock().now() + self.t_delta
        self.pub = node.create_publisher(DiagnosticArray, 'diagnostics', 5)

    def update(self, joints, controllers):
        """ Publish diagnostics. """    
        now = self.node.get_clock().now()
        if now > self.t_next:
            # create message
            msg = DiagnosticArray()
            msg.header.stamp = now.to_msg()
            for controller in controllers:
                d = controller.getDiagnostics()
                if d:
                    msg.status.append(d)
            for joint in joints:
                d = joint.getDiagnostics()
                if d:
                    msg.status.append(d)
            # publish and update stats
            self.pub.publish(msg)
            self.t_next = now + self.t_delta
        

class JointStatePublisher:
    """ Class to handle publications of joint_states message. """

    def __init__(self, node):
        # parameters: throttle rate and geometry
        self.node = node
        self.rate = node.declare_parameter("read_rate", 10.0).get_parameter_value().double_value
        self.t_delta = Duration(seconds=1.0/self.rate)
        self.t_next = node.get_clock().now() + self.t_delta

        # subscriber
        self.pub = node.create_publisher(JointState, 'joint_states', 5)

    def update(self, joints, controllers):
        """ publish joint states. """
        if self.node.get_clock().now() > self.t_next:
            msg = JointState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.name = [] #list()
            msg.position = [] #list()
            msg.velocity = [] #list()
            for joint in joints:
                msg.name.append(joint.name)
                msg.position.append(joint.position)
                msg.velocity.append(joint.velocity)
            for controller in controllers:
                msg.name += controller.joint_names
                msg.position += controller.joint_positions
                msg.velocity += controller.joint_velocities
            self.pub.publish(msg)
            self.t_next = self.node.get_clock().now() + self.t_delta


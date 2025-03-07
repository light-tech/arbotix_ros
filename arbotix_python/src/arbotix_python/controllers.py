#!/usr/bin/env python3

# Copyright (c) 2010-2011 Vanadium Labs LLC. 
# All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of Vanadium Labs LLC nor the names of its 
#     contributors may be used to endorse or promote products derived 
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## @file controllers.py Base class and support functions for a controllers.

from array import array

## @brief Controllers interact with ArbotiX hardware.
class Controller:

    ## @brief Constructs a Controller instance.
    ##
    ## @param device The arbotix instance.
    ## 
    ## @param name The controller name.
    def __init__(self, device, name):
        self.name = name
        self.device = device
        self.fake = device.fake
        self.pause = False

        # output for joint states publisher
        self.joint_names = array('d') #list()
        self.joint_positions = array('d') #list()
        self.joint_velocities = array('d') #list()

    ## @brief Start the controller, do any hardware setup needed.
    def startup(self):
        pass

    ## @brief Do any read/writes to device.
    def update(self):
        pass

    ## @brief Stop the controller, do any hardware shutdown needed.
    def shutdown(self):
        pass

    ## @brief Is the controller actively sending commands to joints?
    def active(self):
        return False
        
    ## @brief Get a diagnostics message for this joint.
    ##
    ## @return Diagnostics message. 
    def getDiagnostics(self):
        msg = DiagnosticStatus()
        msg.name = self.name
        msg.level = DiagnosticStatus.OK
        msg.message = "OK"
        return msg


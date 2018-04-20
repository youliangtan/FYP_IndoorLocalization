#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *


if __name__=="__main__":
    rospy.init_node("simple_marker")
    
    # # create an interactive marker server on the topic namespace simple_marker
    # server = InteractiveMarkerServer("simple_marker")
    
    # # create an interactive marker for our server
    # int_marker = InteractiveMarker()
    # int_marker.header.frame_id = "world"
    # int_marker.name = "my_marker"
    # int_marker.description = "Simple 1-DOF Control"

    # create a grey box marker
    box_marker = Marker()
    marker.header.frame_id = "world"
    int_marker.name = "my_marker"
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0
    box_marker.pose.position.x = 1;
    box_marker.pose.position.y = 1;
    box_marker.pose.position.z = 1;
    box_marker.pose.orientation.x = 0.0;
    box_marker.pose.orientation.y = 0.0;
    box_marker.pose.orientation.z = 0.0;
    box_marker.pose.orientation.w = 1.0

    print "Marker is starting"

    # 'commit' changes and send to all clients
    server.applyChanges()

    rospy.spin()


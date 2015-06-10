#!/usr/bin/env python
from __future__ import print_function, division
try:
    import cPickle as pickle
except:
    import pickle
import rospy
from qsr_lib.srv import QSRViz, QSRVizRequest, QSRVizResponse

from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import Pose,Point
import rospy
from visualization_msgs.msg import *
from std_msgs.msg import ColorRGBA
#import numpy as np
#from itertools import combinations


class QSRlib_Rviz(object):
    def __init__(self, server):
        self.server = server
        self.object_topic = rospy.resolve_name("/QSR_markers/qsr1")
        self.traj = {}
        
    def handle_qsrlib_rviz(self, req):
        self.uuid = req.uuid
        self.world = pickle.loads(req.world_trace)
        self.world_qsr = pickle.loads(req.world_qsr_trace)
        
        self.parse_world()
        #print(self.traj)
        #print(self.objects)
        self.parse_qsr()
        #print(self.qsr)
        self.plot_traj()
        return QSRVizResponse()
        
    def parse_world(self):
        if self.uuid not in self.traj:
            self.traj[self.uuid] = {}
            self.traj[self.uuid]['x'] = []
            self.traj[self.uuid]['y'] = []
            self.traj[self.uuid]['z'] = []
        self.objects = []
        for i in self.world.get_sorted_timestamps():
            for j in self.world.trace[i].objects:
                if j not in self.objects: self.objects.append(j)
                if j == 'trajectory':
                    self.traj[self.uuid]['x'].append(self.world.trace[i].objects[j].x)
                    self.traj[self.uuid]['y'].append(self.world.trace[i].objects[j].y)
                    self.traj[self.uuid]['z'].append(0)
            
    def parse_qsr(self):
        self.qsr = {}
        for i in self.world_qsr.get_sorted_timestamps():
            for j in self.world_qsr.trace[i].qsrs:
                if j not in self.qsr:   self.qsr[j] = []
                self.qsr[j].append(self.world_qsr.trace[i].qsrs[j].qsr)
                
    def plot_traj(self):
        print(self.traj)
        int_marker = self.create_object_marker(self.uuid,self.traj[self.uuid]['x'],self.traj[self.uuid]['y'],self.traj[self.uuid]['z'])
        self.server.insert(int_marker, self.objFeedback)
        self.server.applyChanges()
        
    # 
    def objFeedback(self, feedback ):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            #compute difference vector for this cube
            x = feedback.pose.position.x
            y = feedback.pose.position.y
            z = feedback.pose.position.z

            server.applyChanges()

        
    def create_object_marker(self,name, X, Y, Z ):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.name = name
        int_marker.description = name
	
        pose = Pose()
        pose.position.x = .0
        pose.position.y = .0
        pose.position.z = .0
        int_marker.pose = pose

        line_marker = Marker()
        line_marker.type = Marker.LINE_LIST
        line_marker.scale.x = 0.005

        line_marker.points = []
        for i in range(len(X)):
                p = Point()
                p.x = X[i]
                p.y = Y[i]
                p.z = Z[i]
                line_marker.points.append(p)

        line_marker.colors = []
        for i in range(len(X)):
                color = ColorRGBA()
                color.r = .1
                color.g = .0
                color.b = .65
                color.a = .7
                line_marker.colors.append(color)

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.markers.append(line_marker)
        int_marker.controls.append(control)

        return int_marker
            


def cl_qsrlib_rviz(uuid, world_trace, world_qsr_trace, srv_name="/qsrlib_rviz"):
    rospy.wait_for_service(srv_name)
    try:
        req = QSRVizRequest()
        req.header.stamp = rospy.get_rostime()
        req.uuid = uuid
        req.world_trace = pickle.dumps(world_trace)
        req.world_qsr_trace = pickle.dumps(world_qsr_trace)
        proxy = rospy.ServiceProxy(srv_name, QSRViz)
        res = proxy(req)
        return res
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)

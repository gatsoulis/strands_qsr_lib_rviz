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
import numpy as np
import colorsys


class QSRlib_Rviz(object):
    def __init__(self, server):
        self.server = server
        self.traj = {}
        self.objects_location = {}
        
    #--------------------------------------------------------------------#
    def handle_qsrlib_rviz(self, req):
        self.uuid = req.uuid
        self.world = pickle.loads(req.world_trace)
        self.world_qsr = pickle.loads(req.world_qsr_trace)
                                                                                                                ###### for Yianni
        self.qsr_range = ['near','medium','far']
        self.parse_world()
        #print(self.traj)
        #print(self.objects)
        #print(self.objects_location)
        self.parse_qsr()
        #print(self.qsr)
        self.colors()                       # generate colors for the different qsrs
        self.plot_traj1()                   # plot the first kind of qsrs (4 lines)
        self.plot_traj2()                   # plot the second kind (lines to objects)
        return QSRVizResponse()
     
    #--------------------------------------------------------------------#   
    def parse_world(self):
        if self.uuid not in self.traj:
            self.traj[self.uuid] = {}
            self.traj[self.uuid]['x'] = []
            self.traj[self.uuid]['y'] = []
            self.traj[self.uuid]['z'] = []
            self.traj[self.uuid]['processed'] = 0
            self.objects_location[self.uuid] = {}
        self.objects = []
        for i in self.world.get_sorted_timestamps():
            for j in self.world.trace[i].objects:
                if j not in self.objects: self.objects.append(j)
                if j == 'trajectory':
                    self.traj[self.uuid]['x'].append(self.world.trace[i].objects[j].x)
                    self.traj[self.uuid]['y'].append(self.world.trace[i].objects[j].y)
                    self.traj[self.uuid]['z'].append(.1)
                elif j not in self.objects_location[self.uuid]:
                    self.objects_location[self.uuid][j] = {}
                    self.objects_location[self.uuid][j]['x'] = self.world.trace[i].objects[j].x
                    self.objects_location[self.uuid][j]['y'] = self.world.trace[i].objects[j].y
                    self.objects_location[self.uuid][j]['z'] = .1
                
                
    
    #--------------------------------------------------------------------#        
    def parse_qsr(self):
        self.qsr = {}
        for i in self.world_qsr.get_sorted_timestamps():
            for j in self.world_qsr.trace[i].qsrs:
                if j not in self.qsr:   self.qsr[j] = []
                self.qsr[j].append(self.world_qsr.trace[i].qsrs[j].qsr)
    
    #--------------------------------------------------------------------#            
    def plot_traj1(self):
        keys = self.qsr.keys()
        for i in range(len(keys)):
            self.qsr_key = keys[i]
            offset = np.ones(len(self.traj[self.uuid]['x']))*i*.12
            int_marker = self.create_object_marker1(self.uuid+'-'+self.qsr_key, self.traj[self.uuid]['x'], self.traj[self.uuid]['y'], self.traj[self.uuid]['z']+offset)
            self.server[1].insert(int_marker, self.objFeedback)
            self.server[1].applyChanges()
     
    #--------------------------------------------------------------------#       
    def plot_traj2(self):
        keys = self.qsr.keys()
        int_marker = self.create_object_marker2(self.uuid, self.traj[self.uuid]['x'], self.traj[self.uuid]['y'], self.traj[self.uuid]['z'])
        self.server[2].insert(int_marker, self.objFeedback)
        self.server[2].applyChanges()
        for i in range(len(keys)):
            self.qsr_key = keys[i]
            int_marker = self.create_object_lines(self.uuid+'-'+self.qsr_key, self.traj[self.uuid]['x'], self.traj[self.uuid]['y'], self.traj[self.uuid]['z'])
            self.server[2].insert(int_marker, self.objFeedback)
            self.server[2].applyChanges()
     
    #--------------------------------------------------------------------#   
    def create_object_marker1(self, name, X, Y, Z):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = name
        int_marker.description = name
	
        pose = Pose()
        pose.position.x = X[0]
        pose.position.y = Y[0]
        pose.position.z = Z[0]
        int_marker.pose = pose

        line_marker = Marker()
        line_marker.type = Marker.LINE_LIST
        line_marker.scale.x = 0.04

        line_marker_qsr = []
        qsr = self.qsr[self.qsr_key]
        line_marker.points = []
        p = Point()
        p.x = X[0]-X[0]
        p.y = Y[0]-Y[0]
        p.z = Z[0]-Z[0]
        line_marker.points.append(p)
        line_marker_qsr.append(qsr[0])
        for i in range(1,len(X)-1):
            p = Point()
            p.x = X[i]-X[0]
            p.y = Y[i]-Y[0]
            p.z = Z[i]-Z[0]
            line_marker.points.append(p)
            line_marker.points.append(p)
            line_marker_qsr.append(qsr[i])
            line_marker_qsr.append(qsr[i])
        p = Point()
        p.x = X[-1]-X[0]
        p.y = Y[-1]-Y[0]
        p.z = Z[-1]-Z[0]
        line_marker.points.append(p)
        line_marker_qsr.append(qsr[-1])
        
        line_marker.colors = []
        for i in range(len(line_marker.points)):
            index = self.qsr_range.index(line_marker_qsr[i])
            color = ColorRGBA()
            color.r = self.RGB[index][0]
            color.g = self.RGB[index][1]
            color.b = self.RGB[index][2]
            color.a = .9
            line_marker.colors.append(color)

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.markers.append(line_marker)
        int_marker.controls.append(control)

        return int_marker
        
    #--------------------------------------------------------------------#   
    def create_object_marker2(self, name, X, Y, Z):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = name
        int_marker.description = name
	
        pose = Pose()
        pose.position.x = X[0]
        pose.position.y = Y[0]
        pose.position.z = Z[0]
        int_marker.pose = pose

        line_marker = Marker()
        line_marker.type = Marker.LINE_LIST
        line_marker.scale.x = 0.04

        line_marker.points = []
        p = Point()
        p.x = X[0]-X[0]
        p.y = Y[0]-Y[0]
        p.z = Z[0]-Z[0]
        line_marker.points.append(p)
        for i in range(1,len(X)-1):
            p = Point()
            p.x = X[i]-X[0]
            p.y = Y[i]-Y[0]
            p.z = Z[i]-Z[0]
            line_marker.points.append(p)
            line_marker.points.append(p)
        p = Point()
        p.x = X[-1]-X[0]
        p.y = Y[-1]-Y[0]
        p.z = Z[-1]-Z[0]
        line_marker.points.append(p)
        
        line_marker.colors = []
        for i in range(len(line_marker.points)):
            color = ColorRGBA()
            color.r = .4
            color.g = .0
            color.b = .9
            color.a = .9
            line_marker.colors.append(color)

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.markers.append(line_marker)
        int_marker.controls.append(control)

        return int_marker
     
    #--------------------------------------------------------------------#   
    def create_object_lines(self, name, X, Y, Z):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = name
        int_marker.description = ''
	
        pose = Pose()
        pose.position.x = X[0]
        pose.position.y = Y[0]
        pose.position.z = Z[0]
        int_marker.pose = pose

        line_marker = Marker()
        line_marker.type = Marker.LINE_LIST
        line_marker.scale.x = 0.04

        line_marker_qsr = []
        qsr = self.qsr[self.qsr_key]
        line_marker.points = []
        line_marker.colors = []
        obj = (name.split(',')[1]).split('\n')[0]
        loc = self.objects_location[self.uuid][obj]

        #if self.traj[self.uuid]['processed'] == 0:              #first time I plot this traj
        start = self.traj[self.uuid]['processed']
        if start == 0:
            p = Point()
            p.x = X[0]-X[0]
            p.y = Y[0]-Y[0]
            p.z = Z[0]-Z[0]
            line_marker.points.append(p)
            p = Point()
            p.x = loc['x']-X[0]
            p.y = loc['y']-Y[0]
            p.z = loc['z']-Z[0]
            line_marker.points.append(p)
            index = self.qsr_range.index(qsr[0])
            color = ColorRGBA()
            color.r = self.RGB[index][0]
            color.g = self.RGB[index][1]
            color.b = self.RGB[index][2]
            color.a = .9
            line_marker.colors.append(color)
            line_marker.colors.append(color)
                
        old_qsr = qsr[start]
        for i in range(start+1,len(qsr)):
            if old_qsr != qsr[i]:       #change!
                p = Point()
                p.x = X[i]-X[0]
                p.y = Y[i]-Y[0]
                p.z = Z[i]-Z[0]
                line_marker.points.append(p)
                p = Point()
                p.x = loc['x']-X[0]
                p.y = loc['y']-Y[0]
                p.z = loc['z']-Z[0]
                line_marker.points.append(p)
                index = self.qsr_range.index(qsr[i])
                color = ColorRGBA()
                color.r = self.RGB[index][0]
                color.g = self.RGB[index][1]
                color.b = self.RGB[index][2]
                color.a = .9
                line_marker.colors.append(color)
                line_marker.colors.append(color)
                old_qsr = qsr[i]

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.markers.append(line_marker)
        int_marker.controls.append(control)
        return int_marker
     
    #--------------------------------------------------------------------#   
    def colors(self):
        N = len(self.qsr_range)
        HSV = [(x*.9/N, 0.9, 0.9) for x in range(N)]
        self.RGB = map(lambda x: colorsys.hsv_to_rgb(*x), HSV)
      
    #--------------------------------------------------------------------#      
    def objFeedback(self, feedback):
        pass
        #if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            #compute difference vector for this cube
            #x = feedback.pose.position.x
            #y = feedback.pose.position.y
            #z = feedback.pose.position.z
            #self.server[1].applyChanges()
        
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

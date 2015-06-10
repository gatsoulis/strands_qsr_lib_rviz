#!/usr/bin/env python
from __future__ import print_function, division
try:
    import cPickle as pickle
except:
    import pickle
import rospy
from qsr_lib.srv import QSRViz, QSRVizRequest, QSRVizResponse


def handle_qsrlib_rviz(req):
    uuid = req.uuid
    world_trace = pickle.loads(req.world_trace)
    world_qsr_trace = pickle.loads(req.world_qsr_trace)
    # here can do what you like with the data
    # print(uuid, world_trace.get_sorted_timestamps()) # dbg
    return QSRVizResponse()


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

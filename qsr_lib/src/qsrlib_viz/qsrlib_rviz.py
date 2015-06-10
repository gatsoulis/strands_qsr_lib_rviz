#!/usr/bin/env python
from __future__ import print_function, division
import rospy
from qsrlib.msg import *
from qsrlib_io.world_trace import World_Trace
from qsrlib_io.world_qsr_trace import World_QSR_Trace


def rviz_trace_and_qsr_trace(data):
    print("Foo")


def listener_rviz_trace_and_qsr_trace(topic_name="qsrlib_viz"):
    rospy.init_node('qsrlib_rviz')
    rospy.Subscriber(topic_name, QSRViz, rviz_trace_and_qsr_trace)
    rospy.spin()

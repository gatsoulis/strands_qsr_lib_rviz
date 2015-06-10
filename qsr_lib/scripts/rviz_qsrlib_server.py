#!/usr/bin/env python
from __future__ import print_function, division
import argparse
import rospy
from qsr_lib.srv import QSRViz
from qsrlib_viz.qsrlib_rviz import handle_qsrlib_rviz


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--topic", help="topic name to subscribe", type=str)
    args = parser.parse_args()

    topic_name = args.topic if args.topic else "/qsrlib_rviz"

    rospy.init_node('qsrlib_rviz_server')
    s = rospy.Service(topic_name, QSRViz, handle_qsrlib_rviz)
    rospy.spin()



# ignore the rest, this is in case of using a publisher/subscriber instead of a service
# def cb_qsrlib_rviz(data):
#     uuid = data.uuid
#     world_trace = pickle.loads(data.world_trace)
#     world_qsr_trace = pickle.loads(data.world_qsr_trace)
#     print(world_trace.get_sorted_timestamps(),"\n")
#
#
# if __name__ == '__main__':
#     parser = argparse.ArgumentParser()
#     parser.add_argument("-t", "--topic", help="topic name to subscribe", type=str)
#     args = parser.parse_args()
#
#     topic_name = args.topic if args.topic else "/qsrlib_rviz"
#
#     rospy.init_node('qsrlib_rviz_subscriber')
#     rospy.Subscriber(topic_name, QSRViz, cb_qsrlib_rviz)
#     rospy.spin()

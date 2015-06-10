#!/usr/bin/env python
from __future__ import print_function, division
try:
    import cPickle as pickle
except:
    import pickle
import os
import argparse
import rospy
from qsrlib_viz.qsrlib_rviz import cl_qsrlib_rviz


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--path", help="path to sample pickle files", required=True, type=str)
    parser.add_argument("-u", "--uuid", help="traj uuid", type=str)
    parser.add_argument("-s", "--sname", help="service name", type=str)
    args = parser.parse_args()

    srv_name = args.sname if args.sname else "/qsrlib_rviz"
    uuid = args.uuid if args.uuid else "uuid0"

    with open(os.path.join(args.path, "World_Trace.p"), "r") as f:
        world_trace = pickle.load(f)
    with open(os.path.join(args.path, "World_QSR_Trace.p"), "r") as f:
        world_qsr_trace = pickle.load(f)

    rospy.init_node('qsrlib_rviz_client', anonymous=True)
    cl_qsrlib_rviz(uuid, world_trace, world_qsr_trace, srv_name)



# ignore the rest, this is in case of using a publisher/subscriber instead of a service
# if __name__ == '__main__':
#     parser = argparse.ArgumentParser()
#     parser.add_argument("-p", "--path", help="path to sample pickle files", required=True, type=str)
#     parser.add_argument("-u", "--uuid", help="traj uuid", type=str)
#     parser.add_argument("-t", "--topic", help="topic name to publish", type=str)
#     args = parser.parse_args()
#
#     topic_name = args.topic if args.topic else "/qsrlib_rviz"
#
#     with open(os.path.join(args.path, "World_Trace.p"), "r") as f:
#         world_trace = pickle.load(f)
#     with open(os.path.join(args.path, "World_QSR_Trace.p"), "r") as f:
#         world_qsr_trace = pickle.load(f)
#
#     pub = rospy.Publisher(topic_name, QSRViz, queue_size=10)
#     rospy.init_node('qsrlib_rviz_publisher', anonymous=True)
#
#     data = QSRViz()
#     data.header.stamp = rospy.get_rostime()
#     data.uuid = args.uuid if args.uuid else "uuid0"
#     data.world_trace = pickle.dumps(world_trace)
#     data.world_qsr_trace = pickle.dumps(world_qsr_trace)
#
#     pub.publish(data)
#
#     rospy.spin()

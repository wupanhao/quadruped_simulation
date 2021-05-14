#!/usr/bin/env python
#!coding:utf-8
import rospy
import tf
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
import config
from kinematics import leg_ki, ik, point_rotate_y
# tip = [4.242640687119291, 30.0, -171.11984104714452]
tip = leg_ki([20, 45, -30])
# tip = leg_ki([0, 45, -45])
print(tip)
scale = 1/60.0
a = 0
a = 0


def computePoints(to):
    y, x, z = to
    angles = ik(to)
    rads = angles/180.0*np.pi
    po = [0, 0, 0]
    pb = [0, 0, -z]
    bh = point_rotate_y([config.ABDUCTION_OFFSET, 0, 0], angles[0])
    bk = point_rotate_y([config.ABDUCTION_OFFSET, -config.LEG_L1 *
                         math.sin(rads[1]), -config.LEG_L1 * math.cos(rads[1])], angles[0])
    bm = point_rotate_y([config.ABDUCTION_OFFSET, 0, -
                         config.LEG_L1 * math.cos(rads[1])], angles[0])
    bn = point_rotate_y([config.ABDUCTION_OFFSET, -config.LEG_L1 *
                         math.sin(rads[1]), -(config.LEG_L1 * math.cos(rads[1])+config.LEG_L2 * math.cos(rads[2]))], angles[0])

    pm = np.array(bm)+np.array(pb)
    pn = np.array(bn)+np.array(pb)
    ph = [bh[0], bh[1], bh[2]-z]
    pk = [bk[0], bk[1], bk[2]-z]
    pf = [x, -y, 0]
    points = [po, pb, ph, pk, pf]
    Y = [x, 0, 0]
    X = [0, -y, 0]
    assists = [pm, pn, Y]
    return points, assists


def toPoint(points):
    pts = [Point(p[0]*scale, p[1]*scale, p[2]*scale) for p in points]
    return pts


br = None


def pubTf(map):
    for key in map:
        # print(key)
        p = map[key]
        br.sendTransform((p.x, p.y, p.z),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         key,
                         "map")


def main():
    global br
    rospy.init_node("points_and_lines", anonymous=True)
    br = tf.TransformBroadcaster()
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(5)
    f = 0.0
    while not rospy.is_shutdown():
        types = [Marker.POINTS, Marker.LINE_STRIP, Marker.LINE_LIST]
        markers = [Marker() for _ in ['points', 'line_strip', 'line_list']]
        # markers = [Marker() for _ in ['points', 'line_strip']]
        for i, m in enumerate(markers):  # 分别处理不同的marker
            m.header.frame_id = 'map'
            m.header.stamp = rospy.Time.now()
            m.ns = 'points_and_lines'
            m.pose.orientation.w = 1.0
            m.action = Marker.ADD
            m.id = i
            m.type = types[i]
            m.color.a = 1.0
            if i == 0:  # point
                m.scale.x = 0.01
                m.scale.y = 0.01
                m.color.g = 1.0
            elif i == 1:  # line_strip
                m.scale.x = 0.02
                m.color.g = 1.0
                # m.color.b = 1.0
            elif i == 2:
                m.scale.x = 0.02
                m.color.r = 1.0
                m.color.a = 0.5
        points, assists = computePoints(tip)
        po, pb, ph, pk, pf = toPoint(points)
        pm, pn, Y = toPoint(assists)
        axisLen = 3.5
        y = Point(axisLen/1.5, 0, 0)
        x = Point(0, -axisLen/1.5, 0)
        z = Point(0.0, 0, axisLen)
        for i in points[1:]:  # 给每一个marker添加points
            p = Point(i[0]*scale, i[1]*scale, i[2]*scale)
            markers[0].points.append(p)
            markers[1].points.append(p)
        markers[2].points.extend(
            [pk, pm, pk, pn, pn, Y, ph, Y, pb, Y, ph, pf, po, y, po, x, po, z])
        for m in markers:
            pub.publish(m)

        pubTf({
            "O": po,
            "B": pb,
            "H": ph,
            "K": pk,
            "F": pf,
            "M": pm,
            "N": pn,
            "Y": Y,
            "x": x,
            "y": y,
            "z": z,
        })
        rate.sleep()
        f += 0.04


if __name__ == '__main__':
    main()

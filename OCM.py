#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov  4 23:20:03 2019

@author: diogo
"""

import numpy as np
import math
import matplotlib.pyplot as mplot
import random
import rosbag
import sensor_msgs.point_cloud2 as pc2
import time
import sys
from sensor_msgs.msg import PointCloud2
import rospy
import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData

np.set_printoptions(threshold=sys.maxsize)

class body():

    def __init__(self, length, width):
        self.l = length
        self.w = width

    def length(self):
        return self.length
    def width(self):
        return self.width

class robot():

    def __init__(self, body):
        self.pose = 0
        self.x = 0
        self.y = 0
        self.body = body

    def pose(self):
        return self.pose
    def body(self):
        return self.body
    def setPose(self, p):
#        if(p < 0):
#            p = p + 2*math.pi
        self.pose = p
    def setX(self, x):
        self.x = x
    def setY(self, y):
        self.y = y
    def Pos(self):
        return self.x, self.y

multiplier = 10

#depth camera specs
scanArea = 86 * math.pi / 180 #radians
steps =1000
# beams = np.linspace(-scanArea/2, scanArea/2, num=steps)
Beta = 0.1#(scanArea - (20*math.pi/180) )/ steps

alpha = 0.3*multiplier 

zmax = 10*multiplier #meters

lfree = -0.5
locc = 0.5
l0 = 0

map_resolution = 1

def worldToMap(x,y):
    return math.ceil(x/map_resolution), math.ceil(y/map_resolution)

def occupancy_grid_mapping(robot, zt, m, newm, beams):

    maximum = max(zt)

    for k in range(int(math.floor(robot.x-maximum)), int(math.ceil(robot.x+maximum)), 1):
        for l in range(int(math.floor(robot.y-maximum)), int(math.ceil(robot.y+maximum)), 1):
            if(check_range_field(k, l, robot.x, robot.y, robot.pose)):
                # m[k][l] = 1
                m[k][l] = m[k][l] + inverse_range_sensor_model(k, l , robot.x, robot.y, robot.pose, zt, beams) - l0

            newm[k][l] = (1 - (1/(1 + math.exp(m[k][l]))))*100
            if newm[k][l] > 20 and newm[k][l] <= 80:
                newm[k][l] = -1

    return newm


def check_range_field(c_x, c_y ,r_x, r_y, r_pose):
    #check if cell is in the camera range field

    phi = math.atan2(c_y - r_y, c_x - r_x)# - thetha
    if(phi < 0):
        phi = phi + 2*math.pi

    polarradius = math.sqrt((r_x - c_x)*(r_x - c_x) + (r_y - c_y)*(r_y - c_y))
    startAngle = r_pose - scanArea/2
    endAngle = r_pose + scanArea/2
    if startAngle < 0:
        if phi > 3*math.pi/2:
            phi = phi - 2*math.pi
        endAngle = endAngle - startAngle
        phi = phi - startAngle
        startAngle = 0
    if endAngle > 2*math.pi:
        if phi < math.pi/2:
            phi = phi + 2*math.pi
        startAngle = startAngle - (endAngle-2*math.pi)
        phi = phi - (endAngle-2*math.pi)
        endAngle = endAngle - (endAngle-2*math.pi)

    if( polarradius <= zmax and phi >= startAngle and phi <= endAngle):
        return 1
    else:
        return 0


def inverse_range_sensor_model(cell_x, cell_y, x, y, thetha, zt, beams):

        r = math.sqrt((((cell_x - x)**2)+((cell_y- y)**2)))

        phi_ = math.atan2(cell_y - y, cell_x - x)
        if phi_ < 0:
            phi_ = phi_ + 2*math.pi

        if thetha > 3*math.pi/2 and phi_ < math.pi/2:
            thetha = thetha - phi_
            phi_ = 2*math.pi

        if thetha < math.pi/2 and phi_ > 3*math.pi/2:
            thetha = thetha + (2*math.pi - phi_)
            phi_ = 0

        phi = phi_ - thetha       #get beam angle

        k = np.argmin(abs(beams - phi))                     #assign to beam with closest angle phi

        if (r > min(zmax, zt[k] + alpha/float(2)) or abs(phi - beams[k]) > Beta/float(2)):
        #celula fora do campo de visao
            return l0
        elif (zt[k] < zmax) and abs(r - zt[k]) < alpha/float(2):
        #celulas no range do beam zt[k] +- alpha/2
            return locc
        elif r <= zt[k]:
        #celulas no caminho do beam
            return lfree

def position(robot_trans, robot_rot, r):
    global is_first, start_pose_x, start_pose_y, map_start_x, map_start_y
    rot = robot_rot
    trans = robot_trans
    try:
        q2[0] = rot[0]
        q2[1] = rot[1]
        q2[2] = rot[2]
        q2[3] = rot[3]

        euler = tf.transformations.euler_from_quaternion(q2)

        if euler[2] < 0:
            robot_theta = euler[2] + (2*math.pi)
        else:
            robot_theta = euler[2]

        # print(str(euler[0]*180/math.pi) + " " + str(euler[1]*180/math.pi) + " " + str(euler[2]*180/math.pi))

        if is_first:
            is_first = False
            start_pose_x = trans[0]*multiplier
            start_pose_y = trans[1]*multiplier

        r.setX((trans[0]*multiplier-start_pose_x) + map_start_x)
        r.setY((trans[1]*multiplier-start_pose_y) + map_start_y)
        r.setPose(robot_theta)
        # print((r.pose)*180/math.pi)
        return r

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return

def publisher(newm):

    grid = OccupancyGrid()
    grid.data = newm.ravel()
    # grid.info = MapMetaData()
    grid.info.resolution = 1
    grid.info.origin.position.x = -map_start_x
    grid.info.origin.position.y = -map_start_y
    grid.info.origin.position.z = 0
    grid.info.origin.orientation.x = 1
    grid.info.origin.orientation.y = 1
    grid.info.origin.orientation.z = 0
    grid.info.origin.orientation.w = 0
    grid.info.height = newm.shape[0]
    grid.info.width = newm.shape[1]
    grid.header.frame_id = "/map"

    pub.publish(grid)

def callback(data):
    data_list.append(data)
    (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    position_list.append([trans,rot])

def process_data(data, r, m, robot_trans, robot_rot, newm):

    r = position(robot_trans, robot_rot, r)

    print("x = " + str(r.x) + " y = " + str(r.y))
    mean_count = []
    mean_angle = []
    mean_distance = []
    angle = []
    distance = []
    sort_distance = []
    sort_angle = []

    for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
        if p[1] >= -1.5 and p[1] <= 0:
            if not p[0] or not p[2]:
                continue
            angle.append((math.atan2(p[2]*multiplier,p[0]*multiplier + 0.2*multiplier) - math.pi/2))  #*180/math.pi para graus
            distance.append(math.sqrt((((p[2]*multiplier - 0)**2)+((p[0]*multiplier + 0.2)**2))))

    if not angle or not distance:
        return

    sort_index = np.argsort(angle)

    for i in sort_index:
        sort_angle.append(angle[i])
        sort_distance.append(distance[i])

    max_angle = max(sort_angle)
    min_angle = min(sort_angle)
    delta_angle = (max_angle - min_angle)/steps
    delta_points = len(sort_angle)/steps

    if delta_angle == 0 or delta_points == 0:
        return

    for i in range(steps):
        sum_ = 0
        for j in range(delta_points):
            sum_ += sort_distance[i*delta_points + j]

        mean_angle.append((min_angle + delta_angle*i))
        mean_distance.append(sum_/delta_points)

    beams = np.asarray(mean_angle)

    newm = occupancy_grid_mapping(r, np.asarray(mean_distance), m, newm, beams)
    publisher(newm)

#START EXECUTION
r = robot((1,1))
map_height = 70*multiplier
map_width = 70*multiplier
m = np.zeros((map_height, map_width))
map_start_x = 25*multiplier
map_start_y = 25*multiplier
start_pose_x = 0
start_pose_y = 0
r.setX(25*multiplier)
r.setY(25*multiplier)
is_first = True
q2 = [0,0,0,0]
data_list = []
position_list = []
newm = [[-1]*m.shape[1]]*m.shape[0]
newm = np.asarray(newm)

rospy.init_node('mapping')

rospy.Subscriber("/raposang/depth_cam/depth/color/points", PointCloud2, callback)

pub = rospy.Publisher('/testmap', OccupancyGrid, queue_size=10)

listener = tf.TransformListener()

rate = rospy.Rate(120)
count=0

while not rospy.is_shutdown():

    if data_list and position_list:
        robot_data = data_list.pop(0)
        robot_pose = position_list.pop(0)
        process_data(robot_data, r, m, robot_pose[0], robot_pose[1], newm)
    else:
        print("No data to analyse!")

    rate.sleep()

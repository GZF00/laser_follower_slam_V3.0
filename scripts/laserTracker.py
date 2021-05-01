#!/usr/bin/env python
# coding=utf-8
import rospy
import math
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String as StringMsg
from laser_follower_slam.msg import Position as PositionMsg


# laserTracker类定义
class laserTracker:
    # 类属性初始化
    def __init__(self):
        # 上一次的跟随距离
        self.lastgoalObjectDistance = float('inf')
        # diff_max   :   差分比较值：差分值大于diff_max则判断为一个边界
        self.diff_max = rospy.get_param('~diff_max')
        # object_min :   跟随物体宽度最小值
        self.object_min = rospy.get_param('~object_min') 
        # object_max :   跟随物体宽度最大值 
        self.object_max = rospy.get_param('~object_max')  
        # max_tracker_distance  : 最大跟随距离-超出此距离的物体小车不会跟随
        self.max_tracker_distance = rospy.get_param('~max_tracker_distance')
        # filter_distance       ：滤波处理阈值
        self.filter_distance = rospy.get_param('~filter_distance')


        # scanSubscriber订阅激光雷达/scan数据
        self.scanSubscriber = rospy.Subscriber('/scan', LaserScan, self.registerScan)
        # positionPublisher发布/object_tracker/current_position跟随物体位置信息（角度和距离）
        self.positionPublisher = rospy.Publisher('/object_tracker/current_position', PositionMsg, queue_size=2)
        # infoPublisher发布没有找到物体和物体距离过远信息，停止小车运动
        self.infoPublisher = rospy.Publisher('/object_tracker/info', StringMsg, queue_size=3)

    # scanSubscriber订阅器回调函数
    def registerScan(self, scan_data):

        # 获取雷达数据
        ranges = np.array(scan_data.ranges)

        # 跟随目标信息
        goalObjectID = None
        goalObjectDistance = float('inf')
        goalObjectAngle = float('inf')

        # 激光扫描角度值
        angles = np.zeros(len(ranges))
        for i in range(len(angles)):
            angles[i] = scan_data.angle_min + i * scan_data.angle_increment

        # 求每个点的差分值
        diff = np.zeros(len(ranges))
        for i in range(len(diff)):
            diff[i] = ranges[i] - ranges[i - 1]

        # 根据差分值，寻找物体边界
        edge = np.zeros([0, 2], dtype=np.int)
        for i in range(len(diff)):
            if abs(diff[i]) > self.diff_max:
                edge = np.append(edge, [[i - 1, i]], axis=0)

        # 根据边界，计算物体宽度(余弦定理)
        object_width = np.zeros(len(edge))
        for i in range(len(object_width)):
            object_width[i] = np.sqrt(
                np.square(ranges[edge[i][0]]) + np.square(ranges[edge[i - 1][1]]) - 2 * ranges[edge[i][0]] * ranges[
                    edge[i - 1][1]] * math.cos(angles[edge[i][0]] - angles[edge[i - 1][1]]))

        # 寻找符合设定宽度阈值的物体ID
        objectID = np.zeros(0, dtype=np.int)
        for i in range(len(object_width)):
            if self.object_min < object_width[i] < self.object_max:
                objectID = np.append(
                    objectID, (edge[i][0] + edge[i - 1][1]) // 2)

        # 如果objectID为空，说明没有找到符合阈值的物体，系统发布没有找到物体，停止小车运动
        if len(objectID) == 0:
            self.infoPublisher.publish(StringMsg('Laser not find object!!!'))
        else:

            # 得到符合设定宽度阈值的物体的距离信息
            distance = np.zeros(len(objectID))
            for i in range(len(distance)):
                distance[i] = ranges[objectID[i]]

            # 将距离从小到大排序
            sortedIndex = np.argsort(distance)

            goalObjectID = objectID[sortedIndex[0]]
            goalObjectDistance = ranges[goalObjectID]
            goalObjectAngle = angles[goalObjectID]

            # 滤波处理（判断当前距离与上一次的距离，如果差值过大，则当前可能是噪声，将当前值改为上次的距离）
            if goalObjectDistance - self.lastgoalObjectDistance > self.filter_distance:
                goalObjectDistance = self.lastgoalObjectDistance
            # 距离太远，系统发布物体距离过远，小车停止运动
            if goalObjectDistance > self.max_tracker_distance:
                self.infoPublisher.publish(StringMsg('Object is too far!!!'))
                rospy.logwarn('goalObjectDistance: {} M'.format(goalObjectDistance))
            else:
                # 发布目标的角度和距离
                self.positionPublisher.publish(PositionMsg(goalObjectAngle, goalObjectDistance))
            # 将当前距离值赋给上一次距离值
            self.lastgoalObjectDistance = goalObjectDistance
            


if __name__ == '__main__':
    # 提示laser_tracker节点开始运行
    print('laser_tracker starting...')
    # 初始化laser_tracker节点
    rospy.init_node('laser_tracker')
    # 实例化laserTracker
    tracker = laserTracker()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print('ROSInterruptException')

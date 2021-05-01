#!/usr/bin/env python
# coding=utf-8
import rospy
import time
import numpy as np
from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import Twist, Vector3
from laser_follower_slam.msg import Position as PositionMsg
from std_msgs.msg import String as StringMsg

# Follower类定义
class Follower:
    def __init__(self):
        self.max_speed = rospy.get_param('~maxSpeed')   # maxSpeed          小车最大速度  
        targetDist = rospy.get_param('~targetDist')     # targetDist        目标距离   
        PID_param = rospy.get_param('~PID_controller')  # PID_controller    PID参数   

        # cmdVelPublisher发布底盘控制指令/cmd_vel
        self.cmdVelPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
        # positionSubscriber订阅了/object_tracker/current_position跟随物体位置信息
        self.positionSubscriber = rospy.Subscriber('/object_tracker/current_position', PositionMsg, self.positionUpdateCallback)
        # trackerInfoSubscriber订阅/object_tracker/info系统故障信息
        self.trackerInfoSubscriber = rospy.Subscriber('/object_tracker/info', StringMsg, self.trackerInfoCallback)
        # 第一个参数是角度目标（始终为0度），第二个参数是目标距离（例如1米）
        self.PID_controller = simplePID([0, targetDist], PID_param['P'], PID_param['I'], PID_param['D'])

        # 当使用Ctrl + C终止进程时，将调用此方法
        rospy.on_shutdown(self.stopMoving)

    # trackerInfoSubscriber回调函数
    def trackerInfoCallback(self, info):
        rospy.logwarn(info.data)
        self.stopMoving()

    # positionSubscriber回调函数
    def positionUpdateCallback(self, position):

        if position.angle > 2.8 or position.angle < -2.8:
            angle = 0
        else:
            angle = position.angle
        
        distance = position.distance

        rospy.loginfo('Angle: {} Deg, Distance: {} M'.format(angle, distance))

        # 调用PID控制器进行更新并获得新的速度
        [uncliped_ang_speed, uncliped_lin_speed] = self.PID_controller.update([angle, distance])

        # 限制小车速度在max_speed范围内
        angularSpeed = -np.clip(-uncliped_ang_speed, -self.max_speed, self.max_speed)
        linearSpeed = np.clip(-uncliped_lin_speed, -self.max_speed, self.max_speed)

        # 创建Twist消息以发送到cmd_vel主题
        velocity = Twist()
        velocity.linear = Vector3(linearSpeed, 0, 0.)
        velocity.angular = Vector3(0., 0., angularSpeed)
        rospy.loginfo('linearSpeed: {}, angularSpeed: {}'.format(linearSpeed, angularSpeed))
        self.cmdVelPublisher.publish(velocity)

    # 停止小车运动函数
    def stopMoving(self):
        velocity = Twist()
        velocity.linear = Vector3(0., 0., 0.)
        velocity.angular = Vector3(0., 0., 0.)
        self.cmdVelPublisher.publish(velocity)
        rospy.loginfo('Stop Moving!')


class simplePID:
    def __init__(self, target, P, I, D):
        # 检查参数格式
        if (not (np.size(P) == np.size(I) == np.size(D)) or ((np.size(target) == 1) and np.size(P) != 1) or (
                np.size(target) != 1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
            raise TypeError('input parameters shape is not compatable')
        rospy.loginfo('PID initialised with P:{}, I:{}, D:{}'.format(P, I, D))
        self.Kp = np.array(P)
        self.Ki = np.array(I)
        self.Kd = np.array(D)
        self.setPoint = np.array(target)

        self.last_error = 0
        self.integrator = 0
        self.integrator_max = float('inf')
        self.timeOfLastCall = None

    def update(self, current_value):

        current_value = np.array(current_value)
        if (np.size(current_value) != np.size(self.setPoint)):
            raise TypeError('current_value and target do not have the same shape')
        if (self.timeOfLastCall is None):
            # PID是第一次被调用。 我们还不知道变化量
            # 没有施加控制信号
            self.timeOfLastCall = time.clock()
            return np.zeros(np.size(current_value))

        # 比例环节
        error = self.setPoint - current_value
        P = error

        # 积分环节
        currentTime = time.clock()
        deltaT = (currentTime - self.timeOfLastCall)
        self.integrator = self.integrator + (error * deltaT)
        I = self.integrator

        # 微分环节
        D = (error - self.last_error) / deltaT

        self.last_error = error
        self.timeOfLastCall = currentTime

        # 返回控制量
        return self.Kp * P + self.Ki * I + self.Kd * D


if __name__ == '__main__':
    # 提示follower节点开始运行
    print('follower starting...')
    # 初始化follower节点
    rospy.init_node('follower')
    # 实例化Follower
    follower = Follower()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print('exception')

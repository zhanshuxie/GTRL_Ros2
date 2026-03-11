#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 21 16:13:09 2023

@author: oscar
"""

import time
import sys, select, termios, tty

# [ROS 2 修改] 引入 rclpy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop

CTRL-C to quit
"""
class TeleKey():
    def __init__(self):
        self.twist = Twist()

    def getKey(self):  # 读取按键字符
        tty.setraw(sys.stdin.fileno())
        # select 是非阻塞的，这很好，允许我们循环并处理 spin_once
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
    def vels(self, target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

# 全局变量
backup_linear_vel = 0
backup_angular_vel = 0

# 订阅 /scout/cmd_vel 以获取备份速度
def cmd_callback(cmd):
    global backup_linear_vel, backup_angular_vel
    backup_linear_vel = cmd.linear.x  # 备份scout在Gazebo中的实际线速度
    # print("Updated linear_vel in gazebo: {}".format(backup_linear_vel))
    backup_angular_vel = cmd.angular.z

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    # [ROS 2 修改] 初始化
    rclpy.init()
    node = rclpy.create_node('turtlebot3_teleop')
    
    # [ROS 2 修改] 创建发布者和订阅者
    pub = node.create_publisher(Twist, '/scout/telekey', 5)
    # 订阅 /scout/cmd_vel 以获取备份速度
    cmd_sub = node.create_subscription(Twist, '/scout/cmd_vel', cmd_callback, 1)

    status = 0
    target_linear_vel = 0
    target_angular_vel = 0
    backup_linear_vel = 0
    backup_angular_vel = 0
    linear_vel_limit = 1.0
    angular_vel_limit = 1.0
    telekey = TeleKey()
    flag = False
    
    try:
        print (msg)
        while(1):
            # [ROS 2 修改] 必须调用 spin_once 才能触发 cmd_callback 更新 backup_vel
            rclpy.spin_once(node, timeout_sec=0.1)

            key = telekey.getKey()
            # 若未接管，且 Gazebo 中的线速度为0，则保持线速度为-1.0（Gazebo中静止时的线速度），否则保持当前线速度
            if flag == False:
                target_linear_vel= -1.0  
            if key == '1' :  # 字符1, 开始接管
                target_linear_vel = backup_linear_vel * 2 - 1.0 # 把目标速度对齐当前值,并且把线速度映射到[-1, 1]
                target_angular_vel = backup_angular_vel / 2
                telekey.twist.angular.x = 1.0 # 确保是 float
                flag = True
                print('Engage!!!')
            elif key == '2' :  # 字符2
                telekey.twist.angular.x = 0.0
                flag = False
                print('DisEngage!!!')
            elif key == '\x03' : # CTRL-C
                break

            if flag:
                if key == 'w' :
                    # 增加线速度0.05
                    # 先判断增加后是否过零，如果过零则直接置零，否则增加0.05
                    if (target_linear_vel + 0.40)*target_linear_vel < 0:
                        target_linear_vel = 0.0
                    else:
                        target_linear_vel = target_linear_vel + 0.40
                    print (telekey.vels(0.5*(target_linear_vel+1),target_angular_vel * 2))
                elif key == 'x' :
                    # 减少线速度0.05
                    if (target_linear_vel - 0.40)*target_linear_vel < 0:
                        target_linear_vel = 0.0
                    else:
                        target_linear_vel = target_linear_vel - 0.40
                    print (telekey.vels(0.5*(target_linear_vel+1),target_angular_vel * 2))
                elif key == 'a' :
                    # 增加角速度0.1
                    if (target_angular_vel + 0.40)*target_angular_vel < 0:
                        target_angular_vel = 0.0
                    else:
                        target_angular_vel = target_angular_vel + 0.40
                    print (telekey.vels(0.5*(target_linear_vel),target_angular_vel * 2))
                elif key == 'd' :
                    # 减少角速度0.1
                    if (target_angular_vel - 0.40)*target_angular_vel < 0:
                        target_angular_vel = 0.0
                    else:
                        target_angular_vel = target_angular_vel - 0.40
                    print (telekey.vels(0.5*(target_linear_vel+1),target_angular_vel * 2))
                elif key == 's' :
                    # 紧急停止，实际线速度0.5,角速度置零
                    target_linear_vel   = 0.0
                    target_angular_vel  = 0.0
                    print (telekey.vels(0.5, 0))
                elif key == 'q' :
                    # 角速度置零
                    target_angular_vel = 0.0
                    print (telekey.vels(0.5*(target_linear_vel+1), 0))
                elif key == ' ' :
                    target_linear_vel   = -1.0
                    target_angular_vel  = 0.0
                    print (telekey.vels(0, 0))

            # 线速度和角速度限制[-1, 1]
            if target_linear_vel >= 0:
                target_linear_vel = min(target_linear_vel, linear_vel_limit)
            else:
                target_linear_vel = max(target_linear_vel, -linear_vel_limit)
            
            if target_angular_vel >= 0:
                target_angular_vel = min(target_angular_vel, angular_vel_limit)
            else:
                target_angular_vel = max(target_angular_vel, -angular_vel_limit)
            
            telekey.twist.linear.x = float(target_linear_vel)
            telekey.twist.angular.z = float(target_angular_vel)

            pub.publish(telekey.twist)
    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
        
        # [ROS 2 修改] 关闭节点
        rclpy.shutdown()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
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

    def getKey(self):
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

def cmd_callback(cmd):
    global backup_linear_vel, backup_angular_vel
    backup_linear_vel = cmd.linear.x
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
            rclpy.spin_once(node, timeout_sec=0)

            key = telekey.getKey()
            if key == '1' :
                target_linear_vel = backup_linear_vel
                target_angular_vel = backup_angular_vel
                telekey.twist.angular.x = 1.0 # 确保是 float
                flag = True
                print('Engage!!!')
            elif key == '2' :
                telekey.twist.angular.x = 0.0
                flag = False
                print('DisEngage!!!')
            elif key == '\x03' : # CTRL-C
                break

            if flag:
                if key == 'w' :
                    if (target_linear_vel + 0.05)*target_linear_vel < 0:
                        target_linear_vel = 0.0
                    else:
                        target_linear_vel = target_linear_vel + 0.05
                    print (telekey.vels(0.5*(target_linear_vel+1),target_angular_vel))
                elif key == 's' :
                    if (target_linear_vel - 0.05)*target_linear_vel < 0:
                        target_linear_vel = 0.0
                    else:
                        target_linear_vel = target_linear_vel - 0.05
                    print (telekey.vels(0.5*(target_linear_vel+1),target_angular_vel))
                elif key == 'a' :
                    if (target_angular_vel + 0.1)*target_angular_vel < 0:
                        target_angular_vel = 0.0
                    else:
                        target_angular_vel = target_angular_vel + 0.1
                    print (telekey.vels(0.5*(target_linear_vel),target_angular_vel))
                elif key == 'd' :
                    if (target_angular_vel - 0.1)*target_angular_vel < 0:
                        target_angular_vel = 0.0
                    else:
                        target_angular_vel = target_angular_vel - 0.1
                    print (telekey.vels(0.5*(target_linear_vel+1),target_angular_vel))
                elif key == 'x' :
                    target_linear_vel   = 0.0
                    target_angular_vel  = 0.0
                    print (telekey.vels(0.5, 0))
                elif key == 'q' :
                    target_angular_vel = 0.0
                    print (telekey.vels(0.5*(target_linear_vel+1), 0))
                elif key == ' ' :
                    target_linear_vel   = -1.0
                    target_angular_vel  = 0.0
                    print (telekey.vels(0, 0))

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
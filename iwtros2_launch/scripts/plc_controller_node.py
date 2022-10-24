#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from iwtros2_interface.msg import KukaControl, PlcControl

import snap7 as sn
from snap7.util import * 
import struct

READ_AREA = 0x82
WRITE_AREA = 0x81
START = 200
LENGTH = 1

class PlcController(Node):

    def __init__(self):
        super().__init__("plc_controller_node")

        self.pub = self.create_publisher(KukaControl, "kuka_control", 10)
        self.sub = self.create_subscription(PlcControl, "plc_control", self.callback, 10)
        self.sub

        self._kuka_control = sn.client.Client()
        self._kuka_control.connect("192.168.0.1", 0, 1)
        self._plc_control = sn.client.Client()
        self._plc_control.connect("192.168.0.1", 0, 1)

        self._reached_home = False
        self._placed_conveyor = False
        self._placed_hochregal = False
        aa = KukaControl()
        aa.reached_home
        aa.conveyor_placed()
        aa.hochregallager_placed()

        # wait for go home signal
        mByte = self._kuka_control.read_area(READ_AREA, 0, START, LENGTH)
        wait_for_go_home = get_bool(mByte, 0, 0)
        self.rate = self.create_rate(1)
        while not wait_for_go_home and rclpy.ok():
            mByte = self._kuka_control.read_area(READ_AREA, 0, START, LENGTH)
            wait_for_go_home = get_bool(mByte, 0, 0)
            self.rate.sleep()

        self.get_logger().info("Conveyor system is active and sending robot to HOME position")
        self.pubControl(home=True, conveyor=False, hochreagal=False)

        timer_period = 0.5
        self._timer = self.create_timer(timer_period, self._control_loop)
    

    def __del__(self):
        self._kuka_control.destroy()
        self._plc_control.destroy()

    
    def callback(self, data):
        mByte = self._plc_control.read_area(READ_AREA, 0, START, LENGTH)
        set_bool(mByte, 0, 0, 0)
        set_bool(mByte, 0, 1, 0)
        set_bool(mByte, 0, 2, 0)

        if(data.reached_home):
            set_bool(mByte, 0, 0, 1)
            self._plc_control.write_area(WRITE_AREA, 0, START, mByte)
            self._reached_home = True
            self.get_logger().info("Reached Home")
        if(data.conveyor_placed):
            set_bool(mByte, 0, 1, 1)
            self._plc_control.write_area(WRITE_AREA, 0, START, mByte)
            self._placed_conveyor = True
            self.get_logger().info("Placed Object in Conveyor Belt System")
        if(data.hochregallager_placed):
            set_bool(mByte, 0, 2, 1)
            self._plc_control.write_area(WRITE_AREA, 0, START, mByte)
            self._placed_conveyor = True
            self.get_logger().info("Placed Object in Hochregallager")


    def pubControl(self, home = False, conveyor = False, hochreagal = False):
        msg = KukaControl()
        msg.move_home = home;
        msg.conveyor_pick = conveyor
        msg.hochregallager_pick = hochreagal
        self.pub.publish(msg)


    def _control_loop(self):
        mByte = self._kuka_control.read_area(READ_AREA, 0, START, LENGTH)
        wait_for_home = get_bool(mByte, 0, 0)
        wait_for_conveyor = get_bool(mByte, 0, 1)
        wait_for_hochregal = get_bool(mByte, 0, 2)

        if wait_for_home:
            self._reached_home = False
            self.get_logger().info("Moving the robot to home position")
            self.pubControl(home = True, conveyor=False, hochreagal=False)
            while not self._reached_home and rclpy.ok():
                #rclpy.spin_once(self)
                self.rate.sleep()
        if wait_for_conveyor:
            self._placed_conveyor = False
            self.get_logger().info("Moving the robot to pick from Conveyor position")
            self.pubControl(home = False, conveyor=True, hochreagal=False)
            while not self._placed_conveyor and rclpy.ok():
                #rclpy.spin_once(self)
                self.rate.sleep()
        if wait_for_hochregal:
            self._reached_home = False
            self.get_logger().info("Moving the robot to pick from hochregal position")
            self.pubControl(home=False, conveyor=False, hochreagal=True)
            while not self._placed_hochregal and rclpy.ok():
                #rclpy.spin_once(self)
                self.rate.sleep()



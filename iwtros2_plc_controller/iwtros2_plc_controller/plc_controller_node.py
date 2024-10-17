#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from iwtros2_interface.msg import KukaControl, PlcControl

import snap7 as sn
from snap7.util import *
import struct
import threading
import time

READ_AREA = sn.types.Areas.PA
WRITE_AREA = sn.types.Areas.PE
START = 200
START_ADDR_SLOT_ID = 333
LENGTH = 1


class PlcController(Node):
    def __init__(self):
        super().__init__("plc_controller_node")

        self.pub = self.create_publisher(KukaControl, "/move_group/kuka_control", 10)
        self.sub = self.create_subscription(
            PlcControl, "/move_group/plc_control", self.callback, 10
        )
        self.sub

        self._plc_control = sn.client.Client()
        self._plc_control.connect("192.168.0.1", 0, 1)
        self.get_logger().info(
            "PLC connect is established: %r" % self._plc_control.get_connected()
        )

        self.rate = self.create_rate(2)

        self._reached_home = False
        self._wait_for_go_home = False
        self._placed_conveyor = False
        self._wait_for_conveyor = False
        self._placed_hochregal = False
        self._placed_table = False
        self._use_table = True
        self.get_logger().warn(f"Use Table: {self._use_table}")

    def __del__(self):
        self._plc_control.destroy()

    def callback(self, data):
        mByte = bytearray(1)
        set_bool(mByte, 0, 0, 0) # PLC tag: KUKA_HOME_erreicht
        set_bool(mByte, 0, 1, 0) # PLC tag: KUKA_DHBW_erreicht
        set_bool(mByte, 0, 2, 0) # PLC tag: KUKA_IWT_erreicht
        set_bool(mByte, 0, 3, 0) 
        set_bool(mByte, 0, 4, 0) # PLC tag: Kuka_auslagern_picked
        set_bool(mByte, 0, 5, 0) # plc tag: Kuka_tisch_picked (used only for signal passing - unused within PLC)

        self.get_logger().info("Response from the ROBOT Action completion")
        if data.reached_home:
            set_bool(mByte, 0, 0, 1)
            self._plc_control.write_area(WRITE_AREA, 0, START, mByte)
            time.sleep(0.5)
            set_bool(mByte, 0, 0, 0)
            self._plc_control.write_area(WRITE_AREA, 0, START, mByte)
            self._reached_home = True
            self.get_logger().info("Reached Home")
        if data.picked_from_hochregallager:
            set_bool(mByte, 0, 4, 1)
            self._plc_control.write_area(WRITE_AREA, 0, START, mByte)
            time.sleep(0.5)
            set_bool(mByte, 0, 4, 0)
            self._plc_control.write_area(WRITE_AREA, 0, START, mByte)
            self.get_logger().info("Picked from hochregallager and sending signal to PLC")
        if data.conveyor_placed:
            set_bool(mByte, 0, 2, 1)
            self._plc_control.write_area(WRITE_AREA, 0, START, mByte)
            time.sleep(0.5)
            set_bool(mByte, 0, 2, 0)
            self._plc_control.write_area(WRITE_AREA, 0, START, mByte)
            self._placed_conveyor = True
            self.get_logger().info("Placed Object in Conveyor Belt System")
        if data.hochregallager_placed:
            set_bool(mByte, 0, 1, 1)
            self._plc_control.write_area(WRITE_AREA, 0, START, mByte)
            time.sleep(0.5)
            set_bool(mByte, 0, 1, 0)
            self._plc_control.write_area(WRITE_AREA, 0, START, mByte)
            self._placed_hochregal = True
            self.get_logger().info("Placed Object in Hochregallager")
        if data.table_placed:
            set_bool(mByte, 0, 5, 1)
            self._plc_control.write_area(WRITE_AREA, 0, START, mByte)
            time.sleep(0.5)
            set_bool(mByte, 0, 5, 0)
            self._plc_control.write_area(WRITE_AREA, 0, START, mByte)
            self._placed_table = True
            self.get_logger().info("Placed Object on Table")
        if data.use_table:
            self._use_table = True
        else:
            self._use_table = False
        

    def pubControl(self, home: bool, conveyor: bool, hochreagal: bool, table: bool, slot_id: int):
        msg = KukaControl()
        msg.move_home = home
        msg.conveyor_pick = conveyor
        msg.hochregallager_pick = hochreagal
        msg.table_pick = table
        msg.slot_id = slot_id
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    contl = PlcController()

    plc_control = sn.client.Client()
    plc_control.connect("192.168.0.1", 0, 1)

    while rclpy.ok():
        rclpy.spin_once(contl)
        time.sleep(0.25)
        mByte = plc_control.read_area(READ_AREA, 0, START, LENGTH)
        wait_for_go_home = get_bool(mByte, 0, 0)
        wait_for_conveyor = get_bool(mByte, 0, 2)
        wait_for_endswitch_hoch = get_bool(mByte, 0, 3)
        wait_for_hochregal = get_bool(mByte, 0, 1)

        if wait_for_go_home:  # if(not contl._reached_home):
            contl.get_logger().info("Conveyor system is active and sending robot to HOME position")
            contl.pubControl(home=True, conveyor=False, hochreagal=False)
            while not contl._reached_home and rclpy.ok():
                contl.get_logger().info("Waiting for the robot to reach HOME")
                rclpy.spin_once(contl)
                time.sleep(1)

        if wait_for_conveyor and wait_for_endswitch_hoch:
            contl._placed_conveyor = False
            contl.get_logger().info("Moving the robot to pick from Conveyor position")
            contl.pubControl(home=False, conveyor=True, hochreagal=False)
            while not contl._placed_hochregal and rclpy.ok():
                contl.get_logger().info(
                    "Waiting for the robot complete Picking from Conveyor and Placing on Hochregal"
                )
                rclpy.spin_once(contl)
                time.sleep(1)

            contl._reached_home = False
            contl.pubControl(home=True, conveyor=False, hochreagal=False)
            while not contl._reached_home and rclpy.ok():
                contl.get_logger().info("Waiting for the robot to reach HOME")
                rclpy.spin_once(contl)
                time.sleep(1)

        if wait_for_hochregal:
            time.sleep(2)
            while not wait_for_endswitch_hoch and rclpy.ok():
                mByte = plc_control.read_area(READ_AREA, 0, START, LENGTH)
                wait_for_endswitch_hoch = get_bool(mByte, 0, 3)
                contl.get_logger().info("Waiting for product in Hochregallage to reach Pick Pose")
                time.sleep(1)

            contl._placed_hochregal = False
            contl.get_logger().info("Moving the robot to pick from hochregal position")
            contl.pubControl(home=False, conveyor=False, hochreagal=True)

            while not contl._placed_conveyor and rclpy.ok():
                contl.get_logger().info(
                    "Waiting for the robot complete Picking from Hochregal and Placing on Conveyor"
                )
                rclpy.spin_once(contl)
                time.sleep(1)

            contl._reached_home = False
            contl.pubControl(home=True, conveyor=False, hochreagal=False)
            while not contl._reached_home and rclpy.ok():
                contl.get_logger().info("Waiting for the robot to reach HOME")
                rclpy.spin_once(contl)
                time.sleep(1)

    contl.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

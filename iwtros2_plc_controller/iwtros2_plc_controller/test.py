#!/usr/bin/env python2
import snap7 as sn
from snap7.util import *
import struct
import time


READ_AREA = 0x82
WRITE_AREA = 0x81
START = 200
LENGTH = 1

# kuka_control = sn.client.Client()
# res = kuka_control.connect("192.168.0.1", 0, 1)

# sn.types.Areas.DB

# print(kuka_control.get_connected())


# Go home signal Q200.0
# mByte = kuka_control.read_area(sn.types.Areas.PA, 0, 200, 1)
# print(mByte)

# # Reached Home I200.0
# set_bool(mByte, 0, 0, 1)
# print(mByte)
# kuka_control.write_area(sn.types.Areas.PE, 0, 200, mByte)
# set_bool(mByte, 0, 0, 0)
# time.sleep(1)
# kuka_control.write_area(sn.types.Areas.PE, 0, 200, mByte)

# Pick from Conveyor Q200.2
# mByte = kuka_control.read_area(sn.types.Areas.PA, 0, 200, 1)
# print(mByte)

# # Place from Hochregal I200.1
# set_bool(mByte, 0, 0, 0)
# set_bool(mByte, 0, 1, 1)
# set_bool(mByte, 0, 2, 0)
# print(mByte)
# kuka_control.write_area(sn.types.Areas.PE, 0, 200, mByte)
# time.sleep(5)
# set_bool(mByte, 0, 1, 0)
# print(mByte)
# kuka_control.write_area(sn.types.Areas.PE, 0, 200, mByte)

# Home Reached I200.0
# set_bool(mByte, 0, 0, 1)
# set_bool(mByte, 0, 1, 0)
# set_bool(mByte, 0, 2, 0)
# print(mByte)
# kuka_control.write_area(sn.types.Areas.PE, 0, 200, mByte)


# kuka_control.destroy()

mByte = bytearray(1)
print(mByte)


#!/usr/bin/env python3

from time import sleep
import snap7 as sn
from snap7.util import *
import struct

plc = sn.client.Client()
plc.connect("192.168.0.1", 0, 1)

area = 0x81    # area for Q memory
start = 200      # location we are going to start the read
length = 1     # length in bytes of the read
bit = 1        # which bit in the Q memory byte we are reading

byte = plc.read_area(area, 0, start, length)
print ("Q200.1:",get_bool(byte,0,bit))


set_bool(byte, 0, 1, 1)
plc.write_area(area, 0, 200, byte)

plc.disconnect()


# #! /usr/bin/python
# from time import sleep
# import snap7 as sn
# from snap7.util import *
# import struct

# plc = sn.client.Client()
# plc.connect("192.168.0.1", 0, 1)

# area = 0x82    # area for Q memory
# area_2 = 0x81 #area for I memory
# start = 200      # location we are going to start the read
# length = 1     # length in bytes of the read
# bit = 0        # which bit in the Q memory byte we are reading: Memory:start.bit

# mbyte = plc.read_area(area, 0, start, length)
# print "Q0.0:",get_bool(mbyte,0,bit)
# set_bool(mbyte, 0, bit, 1)
# print "Q0.0:",get_bool(mbyte,0,bit)
# plc.write_area(area_2, 0, start, mbyte)
# plc.disconnect()
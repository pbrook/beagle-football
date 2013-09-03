#! /usr/bin/python

import pypruss
import struct
import time

event_nr = 1 # PRU1_ARM_INTERRUPT

# Must be kept in sync with PRU assembly
COM_SET_POS = 0x00
COM_RANGE = 0x80
COM_INPUT_BITS = 0x84
COM_CURRENT_POS = 0x88

class PRUMonitor(object):
    def __init__(self, prog):
        self.val = {}
        pypruss.modprobe()
        pypruss.init()
        pypruss.open(event_nr)
        pypruss.pruintc_init()
        pypruss.pru_disable(1)
        self.data = pypruss.map_prumem(pypruss.PRUSS0_PRU1_DATARAM)
        self.write32(COM_RANGE, 0)
        pypruss.exec_program(1, prog)

    def read32(self, offset):
        return struct.unpack('L', self.data[offset:offset+4])[0]
    def write32(self, offset, val):
        self.data[offset:offset+4] = struct.pack('L', val)
    def poll(self, offset):
        newval = self.read32(offset)
        prev = self.val.get(offset)
        if prev != newval:
            self.val[offset] = newval
            print "%x = %x" % (offset, newval)
        return newval
    def stop(self):
        pypruss.pru_disable(1)				# Disable PRU 1, this is already done by the firmware
        pypruss.exit()
        self.data = None

if __name__ == '__main__':
    mon = PRUMonitor("./pru_stick.bin")
    mon.write32(COM_SET_POS, 0x808080)
    while True:
        mon.poll(0xf0)
        mon.poll(COM_RANGE)
        mon.poll(COM_CURRENT_POS)
        mon.poll(COM_INPUT_BITS)
        time.sleep(1.0/1000)
    pypruss.wait_for_event(event_nr)
    pypruss.clear_event(event_nr)
    mon.stop()

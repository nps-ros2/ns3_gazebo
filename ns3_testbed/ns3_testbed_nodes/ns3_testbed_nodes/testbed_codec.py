#!/usr/bin/env python3
from time import perf_counter

"""Return CSV encoding of source, message_name, message number, time,
   padding size.  Padded data is message_name[0]."""
def testbed_encode(source, message_name, message_number, padding_size):
    msg = "%s,%s,%d,%f,%s"%(source, message_name, message_number,
                            perf_counter(), message_name[0]*padding_size)
    return msg

"""Return tuple of source, message name, message number, message size,
   delta time in epoc seconds."""
def testbed_decode(msg):
    t1=perf_counter()
    parts=msg[:500].split(",") # 500 should be fine, just don't parse MB size.
    dt = t1 - float(parts[3])
    return parts[0], parts[1], int(parts[2]), len(msg), dt


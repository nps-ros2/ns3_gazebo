#!/usr/bin/env python3
# Adapted from https://github.com/larsks/python-netns/blob/master/netns.py
import os
import time, subprocess, threading, time
from ctypes import CDLL, get_errno

CLONE_NEWNET = 0x40000000

def errcheck(ret, func, args):
    if ret == -1:
        e = get_errno()
        raise OSError(e, os.strerror(e))

libc = CDLL('libc.so.6', use_errno=True)
libc.setns.errcheck=errcheck

"""Set NNS or raise ValueError.  See libc.setns."""
def set_nns(nns_name):
    print("set_nns %s..."%nns_name)
    nnspath = "/var/run/netns/%s"%nns_name
    if not os.path.exists(nnspath):
        error = "Error: NNS %s is not defined.  " \
                "Please run the setup script."%nnspath
        print(error)
        raise ValueError(error)
    with open(nnspath) as fd:
        if hasattr(fd, 'fileno'):
            print("hasattr fileno.")
            fd = fd.fileno()
        status = libc.setns(fd, CLONE_NEWNET)
        if status:
            error = "Error: failure with %s"%nnspath
            print(error)
            raise ValueError(error)
        else:
            print("set_nns %s Done."%nns_name)

def output_handler(proc, name):
    for line in iter(proc.stdout.readline, b''):
        print("%s: %s"%(name, line.decode('utf-8')))

def nns_start(name, nns, cmd):
    set_nns(nns)
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    t = threading.Thread(target=output_handler, args=(p, name))
    t.start()

if __name__ == '__main__':
#    print("start ip...")
#    nns_start("ip_a", "nns3", ["ip", "a"])
    print("start GS...")
    nns_start("GS", "nns1", ["ros2","run","ns3_testbed_nodes", "gs"])
    print("start R1...")
    nns_start("R1", "nns2", ["ros2","run","ns3_testbed_nodes", "r","1"])
    print("Running...")
    time.sleep(10)
    print("Ending...")


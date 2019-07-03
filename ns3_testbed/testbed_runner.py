#!/usr/bin/env python3
# Adapted from https://github.com/larsks/python-netns/blob/master/netns.py
from argparse import ArgumentParser
import os
import time, subprocess, threading, time
from ctypes import CDLL, get_errno

from pipe_logger import PipeReader

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
    if args.local_test:
        # local test
        cmd.append("-l")
    else:
        set_nns(nns)

    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    t = threading.Thread(target=output_handler, args=(p, name))
    t.start()

if __name__ == '__main__':
    parser = ArgumentParser(description="Start testbed robots.")
    parser.add_argument("-l","--local_test", action="store_true",
                        help="Local test mode, do not send output to pipe")
    args = parser.parse_args()
    print("Starting testbed runner...")

    print("start GS...")
    nns_start("GS", "nns1", ["ros2","run","ns3_testbed_nodes", "testbed_robot", "GS"])
    print("start R1...")
    nns_start("R1", "nns2", ["ros2","run","ns3_testbed_nodes", "testbed_robot", "R1"])
    print("Running...")

# no, use GUI
#    reader = PipeReader()
#    while True:
#        print("Queue: %s"%reader.queue.get())


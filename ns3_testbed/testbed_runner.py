#!/usr/bin/env python3
# Adapted from https://github.com/larsks/python-netns/blob/master/netns.py
from argparse import ArgumentParser
from sys import exit
import os
from os.path import join, expanduser
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

    # --setup_file
    cmd.append("-s")
    cmd.append(args.setup_file)

    # --no_nns
    if args.no_nns:
        # suppress nns
        pass
    else:
        # default uses nns
        try:
            set_nns(nns)
        except PermissionError as e:
            print("Error: This program must be run from root when using "
                  "network namespaces.\n"
                  "Use '--no_nns' to run outside namespaces.\nAborting.")
            exit(1)

    # --no_pipe
    if args.no_pipe:
        cmd.append("-p")

    # --verbose
    if args.verbose:
        cmd.append("-v")

    # start
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    t = threading.Thread(target=output_handler, args=(p, name))
    t.start()

if __name__ == '__main__':
    DEFAULT_COUNT = 5
    default_setup_file = join(expanduser("~"),
                         "gits/ns3_gazebo/ns3_testbed/csv_setup/example1.csv")
    parser = ArgumentParser(description="Start testbed robots.")
    parser.add_argument("-c", "--count", type=int, default=DEFAULT_COUNT,
                        help="The number of testbed robot nodes to set up for, "
                             "default %d."%DEFAULT_COUNT)
    parser.add_argument("-s","--setup_file", type=str,
                        help="The CSV setup file.",
                        default = default_setup_file)
    parser.add_argument("-n","--no_nns", action="store_true",
                        help="Do not use network namespaces.")
    parser.add_argument("-p","--no_pipe", action="store_true",
                        help="Do not send output to pipe")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Enable verbose diagnostics.")

    args = parser.parse_args()
    if args.count < 1:
        raise ValueError("Invalid count.")

    print("Starting testbed runner...")

    print("start nns1 GS...")
    nns_start("GS", "nns1",
              ["ros2","run","ns3_testbed_nodes", "testbed_robot", "GS"])

    for i in range(2,args.count+1):
        nns="nns%d"%i
        r="R%d"%(i-1)
        print("start %s %s..."%(nns,r))
        nns_start(r, nns,
                  ["ros2","run","ns3_testbed_nodes", "testbed_robot", r])
    print("Running...")


#!/usr/bin/env python3

# calculate IP values given index
def _ip_from_index(i):
    if i < 1 or i > pow(2,20)//3:
        raise Exception("bad")

    # wifi IP values are contiguous starting at 1
    j=(i-1)*3+1
    k=(i-1)*3+2
    l=(i-1)*3+3
    wifi_veth = "10.%d.%d.%d/9"%(j//65536, (j//256)%256, j%256)
    wifi_vethb = "10.%d.%d.%d/9"%(k//65536, (k//256)%256, k%256)
    wifi_tap = "10.%d.%d.%d/9"%(l//65536, (l//256)%256, l%256)

    # direct IP values are triplets step 8 starting at 1
    j=(i-1)*8+1
    k=(i-1)*8+2
    l=(i-1)*8+3
    direct_br = "10.%d.%d.%d/29"%(128+j//65536, (j//256)%256, j%256)
    direct_vethn = "10.%d.%d.%d/29"%(128+k//65536, (k//256)%256, k%256)
    direct_default = "10.%d.%d.%d"%(128+l//65536, (l//256)%256, l%256)

    return wifi_veth, wifi_vethb, wifi_tap, \
           direct_br, direct_vethn, direct_default

# create tap device: ip tuntap add tap1 mode tap; ip link set dev tap1 up

from argparse import ArgumentParser
import subprocess
import sys

COUNT=5

def _run_cmd(cmd):
    print("Command: %s"%cmd)
    subprocess.run(cmd.split(), stdout=subprocess.PIPE).stdout.decode('utf-8')

def do_setup_nns(i):

    # network namespace
    _run_cmd("ip netns add nns%d"%i)

    # enable loopback - helps Ctrl-C activate on first rather than second
    _run_cmd("ip netns exec nns%d ip link set dev lo up"%i)

def do_setup_wifi(i):

    wifi_veth, wifi_vethb, wifi_tap, direct_br, direct_vethn, direct_default = \
                    _ip_from_index(i)

    # create veth pair
    _run_cmd("ip link add wifi_veth%d type veth peer name wifi_vethb%d"%(
                                                                      i,i))
    _run_cmd("ip address add %s dev wifi_vethb%d"%(wifi_vethb,i))

    # associate wifi_veth with nns
    _run_cmd("ip link set wifi_veth%d netns nns%d"%(i,i))

    # set wifi_veth IP at nns
    _run_cmd("ip netns exec nns%d ip addr add %s "
             " dev wifi_veth%d"%(i, wifi_veth, i))

    # create bridge
    _run_cmd("ip link add name wifi_br%d type bridge"%i)

    # bring up bridge and veth pair
    _run_cmd("ip link set wifi_br%d up"%i)
    _run_cmd("ip link set wifi_vethb%d up"%i)
    _run_cmd("ip netns exec nns%d ip link set wifi_veth%d up"%(i,i))

    # add wifi_vethb to bridge
    _run_cmd("ip link set wifi_vethb%d master wifi_br%d"%(i,i))

    # create tap device
    _run_cmd("ip tuntap add wifi_tap%d mode tap"%i)
    _run_cmd("ip addr flush dev wifi_tap%d"%i) # clear IP
    _run_cmd("ip address add %s dev wifi_tap%d"%(wifi_tap, i))
    _run_cmd("ip link set wifi_tap%d up"%i)

    # add tap to bridge
    _run_cmd("ip link set wifi_tap%d master wifi_br%d"%(i,i))

def do_setup_direct(i):

    wifi_veth, wifi_vethb, wifi_tap, direct_br, direct_vethn, direct_default = \
                    _ip_from_index(i)

    # direct bridge
    _run_cmd("ip link add direct_br%d type bridge"%i)

    # direct veth pair
    _run_cmd("ip link add direct_veth%d type veth peer name direct_vethn%d"%(
                                                                        i,i))

    # veth to bridge
    _run_cmd("ip link set direct_veth%d up"%i)
    _run_cmd("ip link set direct_veth%d master direct_br%d"%(i,i))

    # bridge IP
    _run_cmd("ip address add %s dev direct_br%d "%(direct_br,i))

    # veth to nns
    _run_cmd("ip link set direct_vethn%d netns nns%d"%(i,i))
    _run_cmd("ip netns exec nns%d ip addr add %s "
             "dev direct_vethn%d"%(i,direct_vethn,i))
    _run_cmd("ip netns exec nns%d ip link set direct_vethn%d up"%(i,i))
    _run_cmd("ip netns exec nns%d ip route add default via %s"%(i,direct_default))

    # bridge up
    _run_cmd("ip link set direct_br%d up"%i)


def do_teardown_wifi(i):
    # wifi
    _run_cmd("ip link set wifi_br%d down"%i)
    _run_cmd("ip link set wifi_tap%d down"%i)
    _run_cmd("ip link set wifi_vethb%d down"%i)
    _run_cmd("ip link delete wifi_vethb%d"%i)
    _run_cmd("ip link delete wifi_tap%d"%i)
    _run_cmd("ip link delete wifi_br%d type bridge"%i)

def do_teardown_direct(i):
    # direct
    _run_cmd("ip link set direct_br%d down"%i)
    _run_cmd("ip link delete direct_br%d"%i)
    _run_cmd("ip link set direct_veth%d down"%i)
    _run_cmd("ip link delete direct_veth%d"%i)

def do_teardown_nns(i):
    # network namespace
    _run_cmd("ip netns del nns%d"%i)


if __name__=="__main__":
    parser = ArgumentParser(prog='lxc_setup.py',
                            description="Manage containers used with ns-3.")
    parser.add_argument("command", type=str, help="The command to execute.",
                        choices=["setup", "teardown"])
    parser.add_argument("-c", "--count", type=int, default=5,
                        help="The number of network namespaces to set up for.")
    parser.add_argument("-s", "--suppress_direct", action="store_true",
                        help="Suppress direct connection setup")
    args = parser.parse_args()

    print("Providing '%s' services for %d network namespaces..."%(
                                             args.command, args.count))

    if args.command == "setup":
        for i in range(1,args.count+1):
            do_setup_nns(i)
            do_setup_wifi(i)
            if not args.suppress_direct:
                do_setup_direct(i)
    elif args.command == "teardown":
        for i in range(1,args.count+1):
            do_teardown_wifi(i)
            if not args.suppress_direct:
                do_teardown_direct(i)
            do_teardown_nns(i)
    else:
        print("Invalid command: %s"%args.command)
        sys.exit(1)

    print("Done providing '%s' services for %d network namespaces."%(
                                             args.command, args.count))

    # startup
    # sudo ip netns exec nns1 /bin/bash

    # status
    # network spaces: sudo ip netns list
    # network devices: ip a


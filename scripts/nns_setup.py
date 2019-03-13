#!/usr/bin/env python3
#
# https://superuser.com/questions/764986/howto-setup-a-veth-virtual-network/765078
# https://unix.stackexchange.com/questions/391193/how-to-forward-traffic-between-linux-network-namespaces


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

    # create veth pair
    _run_cmd("ip link add wifi_veth%d type veth peer name wifi_vethb%d"%(
                                                                      i,i))
    _run_cmd("ip address add 10.0.0.%d/24 dev wifi_vethb%d"%(i+20,i))

    # associate wifi_veth with nns
    _run_cmd("ip link set wifi_veth%d netns nns%d"%(i,i))

    # set wifi_veth IP at nns
    _run_cmd("ip netns exec nns%d ip addr add 10.0.0.%d/24 "
             " dev wifi_veth%d"%(i,i,i))

    # create bridge
    _run_cmd("ip link add name wifi_br%d type bridge"%i)

    # bring up bridge and veth pair
    _run_cmd("ip link set wifi_br%d up"%i)
    _run_cmd("ip link set wifi_vethb%d up"%i)
    _run_cmd("ip netns exec nns%d ip link set wifi_veth%d up"%(i,i))

#    _run_cmd("ip netns exec nns%d ip address add 10.0.%d/24 dev "
#             "wifi_veth%d"%(i,i+30,i))

    # add wifi_vethb to bridge
    _run_cmd("ip link set wifi_vethb%d master wifi_br%d"%(i,i))

    # create tap device
    _run_cmd("ip tuntap add wifi_tap%d mode tap"%i)
    _run_cmd("ip addr flush dev wifi_tap%d"%i) # clear IP
    _run_cmd("ip address add 10.0.0.%d/24 dev wifi_tap%d"%(i+50,i))
    _run_cmd("ip link set wifi_tap%d up"%i)

    # add tap to bridge
    _run_cmd("ip link set wifi_tap%d master wifi_br%d"%(i,i))

def do_setup_direct(i):

    # direct bridge
    _run_cmd("ip link add direct_br%d type bridge"%i)

    # direct veth pair
    _run_cmd("ip link add direct_veth%d type veth peer name direct_vethn%d"%(
                                                                        i,i))

    # veth to bridge
    _run_cmd("ip link set direct_veth%d up"%i)
    _run_cmd("ip link set direct_veth%d master direct_br%d"%(i,i))

    # bridge IP
    _run_cmd("ip address add 10.0.%d.1/24 dev direct_br%d "%(i,i))

    # veth to nns
    _run_cmd("ip link set direct_vethn%d netns nns%d"%(i,i))
    _run_cmd("ip netns exec nns%d ip addr add 10.0.%d.2/24 "
             "dev direct_vethn%d"%(i,i,i))
    _run_cmd("ip netns exec nns%d ip link set direct_vethn%d up"%(i,i))
    _run_cmd("ip netns exec nns%d ip route add default via 10.0.%d.3"%(i,i))

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
    args = parser.parse_args()

    print("Providing '%s' services for %d conainers."%(args.command, COUNT))

    if args.command == "setup":
        for i in range(1,COUNT+1):
            do_setup_nns(i)
            do_setup_wifi(i)
            do_setup_direct(i)
    elif args.command == "teardown":
        for i in range(1,COUNT+1):
            do_teardown_wifi(i)
            do_teardown_direct(i)
            do_teardown_nns(i)
    else:
        print("Invalid command: %s"%args.command)
        sys.exit(1)

    # startup
    # sudo ip netns exec nns1 /bin/bash

    # status
    # network spaces: sudo ip netns list
    # network devices: ip a


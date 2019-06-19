# Adapted from https://github.com/larsks/python-netns/blob/master/netns.py
import os
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
    nnspath = "/var/run/netns/%s"%nns_name
    if not os.path.exists(nnspath):
        error = "Error: NNS %s is not defined.  Please run the setup script."
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


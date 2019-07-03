# Provides PipeLogger and PipeReader
import stat, os
from os.path import expanduser
from fcntl import flock, LOCK_EX, LOCK_UN
from threading import Thread
from queue import Queue
from time import sleep

PIPE_NAME=os.path.join(expanduser("~"), "_testbed_pipe.pipe")
def _maybe_make_pipe():
    if os.path.exists(PIPE_NAME) and stat.S_ISFIFO(os.stat(PIPE_NAME).st_mode):
        # path exists and is pipe
        return
    else:
        # create pipe
        os.mkfifo(PIPE_NAME)

# use PipeLogger to threadsafely write to pipe
class PipeLogger():

    def __init__(self):
        _maybe_make_pipe()
        self.f = open(PIPE_NAME, "w")

    def log(self, text):
        f=self.f
        # secure exclusive lock, write, flush, unlock
        flock(f, LOCK_EX)
        print(text,file=f) # print appends \n
        f.flush()
#        print("log text: %s"%text)
        flock(f, LOCK_UN)

# use PipeReader to consume pipe to queue
def _pipe_consumer_thread(queue):
    with open(PIPE_NAME) as f:
        while True:
            line = f.readline()
            if not line:
                # we can get an empty line if the pipe is not connected
                # so try not to busy-wait and wait for it to open again
                sleep(0.1)
                continue

            try:
                queue.put(line)
            except queue.full:
                print("pipe_logger queue full.  "
                      "Dropping line '%s'"%line)
               
class PipeReader():
    def __init__(self):
        _maybe_make_pipe()
        self.queue = Queue()
        t = Thread(target=_pipe_consumer_thread, args=(self.queue,))
        t.start()


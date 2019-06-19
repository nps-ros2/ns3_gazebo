#!/usr/bin/env python3
# inspired by https://eli.thegreenplace.net/2017/interacting-with-a-long-running-child-process-in-python/

from argparse import ArgumentParser
import subprocess
import threading
import time

def output_handler(proc, proc_id):
    for line in iter(proc.stdout.readline, b''):
        print("%s: %s"%(proc_id, line.decode('utf-8')))

def start_all(num_robots):

    # run .bashrc since sudo loses environment setup
    subprocess.run(["source","~/.bashrc"])
#    # start ns3
#    p_ns3 = subprocess.Popen(["ns3_mobility/build/ns3_mobility"],
#                           stdout=subprocess.PIPE,
#                           stderr=subprocess.STDOUT)
#
#    t_ns3 = threading.Thread(target=output_handler, args=(p_ns3, "ns3"))
#    t_ns3.start()

    # start GS
    p_gs = subprocess.Popen(["ros2","run","ns3_testbed_nodes","gs","-n"],
                          stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT)

    t_gs = threading.Thread(target=output_handler, args=(p_gs, "GS"))
    t_gs.start()

    # start robots
    for i in range(num_robots):

        p_r = subprocess.Popen(["ros2","run","ns3_testbed_nodes",
                                "r","-n", "%d"%i],
                          stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT)

        t_r = threading.Thread(target=output_handler, args=(p_r, "R%d"%i))
        t_r.start()


#    # end relatively gracefully
#    try:
#        time.sleep(20)
#    except KeyboardInterrupt:
#        print("zz Keyboard interrupt")
#    finally:
#        pass
##        p_ns3.terminate()

    # start GS in nns1
    print("zz: %d"%num_robots)


# main
if __name__=="__main__":
    parser = ArgumentParser(description="Launch ns3, GS, and <n> robots.")
    parser.add_argument("num_robots", type=int, nargs="?", default=1,
                        help="The number of robots to launch, "
                             "in addition to the Ground Station.")
    args = parser.parse_args()

    start_all(args.num_robots)

#    # create the "application" and the main window
#    application = QApplication(sys.argv)
#    main_window = QMainWindow()
#
#    gui_manager = GUIManager(main_window)

#    # start the GUI
#    gui_manager.w.show()
#    sys.exit(application.exec_())


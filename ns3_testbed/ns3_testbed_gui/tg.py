#!/usr/bin/env python3

from argparse import ArgumentParser
import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QMainWindow

from gui_manager import GUIManager

# main
if __name__=="__main__":
    parser = ArgumentParser(description="GUI for graphing "
                                        "network characteristics.")
    args = parser.parse_args()

    # create the "application" and the main window
    application = QApplication(sys.argv)
    main_window = QMainWindow()

    gui_manager = GUIManager(main_window)

    # start the GUI
    gui_manager.w.show()
    sys.exit(application.exec_())


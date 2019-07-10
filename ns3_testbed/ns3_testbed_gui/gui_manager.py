from PyQt5.QtWidgets import qApp
from PyQt5.QtWidgets import QTableView
from PyQt5.QtWidgets import QAbstractItemView
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt
from PyQt5.QtCore import QUrl
from PyQt5.QtCore import QObject
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtCore import QSortFilterProxyModel

from version_file import VERSION
from testbed_codec import COLUMNS
from network_data_table_model import NetworkDataTableModel

"""Main window."""

class GUIManager(QObject):

    def __init__(self, main_window):
        super(GUIManager, self).__init__()

        # main window
        self.w = main_window

        # main window decoration
        self.w.setGeometry(0,0,800,600)
        self.w.setWindowTitle("Testbed GUI Version %s"%VERSION)
        self.w.setWindowIcon(QIcon('icons/animals-whale.png'))

        # the network data table model
        self.network_data_table_model = NetworkDataTableModel(COLUMNS)

        # the proxy model
        self.proxy_model = QSortFilterProxyModel()

        # the network data table
        self.network_table = QTableView()
        self.network_table.setSortingEnabled(True)
        self.network_table.setSelectionMode(QAbstractItemView.NoSelection)
        self.proxy_model.setSourceModel(self.network_data_table_model)
        self.network_table.setModel(self.proxy_model)
        self.w.setCentralWidget(self.network_table)
 

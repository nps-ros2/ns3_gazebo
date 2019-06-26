from PyQt5.QtCore import (QLineF, QPointF, qrand, QRect, QRectF, QSizeF, Qt)
from PyQt5.QtCore import QPoint
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtCore import QObject
from PyQt5.QtGui import (QBrush, QColor, QLinearGradient, QPainter,
                         QPainterPath, QPen, QPolygonF, QRadialGradient)
from PyQt5.QtGui import QTransform
from PyQt5.QtWidgets import (QGraphicsItem, QGraphicsView, QStyle)
from PyQt5.QtWidgets import QGraphicsScene
from annotation import Annotation
from texture_graph import TextureGraph
from similarity_lines import SimilarityLines
from histogram_graph import HistogramGraph
from settings_manager import settings

"""GraphMainWidget provides the main QGraphicsView.  It manages signals
and wraps these:
  * GraphMainView
  * GraphMainScene
"""

# GraphicsView
class GraphMainView(QGraphicsView):
    def __init__(self):
        super(GraphMainView, self).__init__()

        self.viewport_cursor = Qt.ArrowCursor
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setRenderHint(QPainter.Antialiasing)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)

        self.setMinimumSize(300, 300)


class GraphMainScene(QGraphicsScene):

    def __init__(self):
        super(GraphMainScene, self).__init__()
        y=0

        # general annotation
        self.annotation = Annotation()
        self.addItem(self.annotation)
        self.annotation.setPos(QPointF(0,100-18*(6+1))) # height depends
        y+= 18*(6+1)

        # texture graph 1
        self.texture_graph1 = TextureGraph()
        self.addItem(self.texture_graph1)
        self.texture_graph1.setPos(QPointF(0,y))
        y+= 100

        # similarity lines
        self.similarity_lines = SimilarityLines()
        self.addItem(self.similarity_lines)
        self.similarity_lines.setPos(QPointF(0,y))
        y+= 200

        # texture graph 2
        self.texture_graph2 = TextureGraph()
        self.addItem(self.texture_graph2)
        self.texture_graph2.setPos(QPointF(0,y))
        y+= 140

        # histogram graph
        self.histogram_graph = HistogramGraph()
        self.addItem(self.histogram_graph)
        self.histogram_graph.setPos(QPointF(0,y))

# GraphMainWidget
class GraphMainWidget(QObject):

    def __init__(self, signal_data_loaded, signal_scale_changed):
        super(GraphMainWidget, self).__init__()

        # GraphMainWidget's scene and view objects
        self.scene = GraphMainScene()
        self.view = GraphMainView()
        self.view.setScene(self.scene)

        # connect to schedule repaint on change
        signal_data_loaded.connect(self.change_data)
        signal_scale_changed.connect(self.change_scale)

    # call this to accept data change
    @pyqtSlot(dict, dict, int, dict)
    def change_data(self, tv_data1, tv_data2, step, similarity_data):
        self.scene.annotation.set_data(tv_data1, tv_data2, step,
                                       similarity_data)
        self.scene.texture_graph1.set_data(tv_data1, step)
        self.scene.texture_graph2.set_data(tv_data2, step)
        self.scene.similarity_lines.set_data(tv_data1, tv_data2, step,
                                       similarity_data["similarity_lines"])
        self.scene.histogram_graph.set_data(tv_data1, tv_data2,
                                       similarity_data)
        self.scene.setSceneRect(self.scene.itemsBoundingRect())

    # call this to accept scale change
    @pyqtSlot(float)
    def change_scale(self, scale):
        self.scene.annotation.set_scale(scale)
        self.scene.texture_graph1.set_scale(scale)
        self.scene.texture_graph2.set_scale(scale)
        self.scene.similarity_lines.set_scale(scale)
        self.scene.setSceneRect(self.scene.itemsBoundingRect())


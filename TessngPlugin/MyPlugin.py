# -*- coding: utf-8 -*-

from PySide2.QtGui import *
from PySide2.QtWidgets import *

from Tessng import TessPlugin
from MyNet import *
from MySimulator import *
from TESS_API_EXAMPLE import *

class MyPlugin(TessPlugin):
    def __init__(self):
        super(MyPlugin, self).__init__()
        self.mNetInf = None
        self.mSimuInf = None

    def init(self):
        self.mNetInf = MyNet()
        self.mSimuInf = MySimulator()

    def customerNet(self):
        return self.mNetInf

    def customerSimulator(self):
        return self.mSimuInf

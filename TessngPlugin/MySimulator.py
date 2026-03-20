import os

from PySide2.QtCore import *
from shiboken2.shiboken2 import wrapInstance

from Tessng import *

from AutoPilot.PlayerManager import PlayerManager
from ExternVehicleLogicTessAuto import TessAutoPyInterface


class MySimulator(QObject, PyCustomerSimulator):
    def __init__(self):
        super().__init__()
        PyCustomerSimulator.__init__(self)

        self.iface = None
        self.simIface = None
        self.netIface = None
        self.scene = None

        self.manager = PlayerManager()

        iface = tessngIFace()
        self.tessAuto = TessAutoPyInterface(iface)

    def beforeStart(self, keepOn: bool) -> None:
        self.iface = tessngIFace()
        self.simIface = self.iface.simuInterface()
        self.netIface = self.iface.netInterface()
        self.scene = self.netIface.graphicsScene()

        self.manager.load_all()

    def ref_beforeNextPoint(self, pIVehicle, ref_keepOn):
        if pIVehicle:
            self.tessAuto.vehicleUpdate(pIVehicle)

    def afterOneStep(self):
        states = self.manager.step_all()

        self.tessAuto.setAvChannel2AvMsgMap(states)
        self.tessAuto.vehicleCreate()
        print(states)


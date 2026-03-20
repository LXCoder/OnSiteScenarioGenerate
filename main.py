# -*- coding: utf-8 -*-
import os
from pathlib import Path
import sys

workspace = Path(__file__).resolve().parent
dll_dir = workspace / "TessngLib"
tessng_plugin_dir = workspace / "TessngPlugin"
os.environ["PATH"] = str(dll_dir) + os.pathsep + os.environ["PATH"]
os.environ["PATH"] = str(tessng_plugin_dir) + os.pathsep + os.environ["PATH"]
sys.path.insert(0, str(dll_dir))
sys.path.insert(0, str(tessng_plugin_dir))


from PySide2.QtWidgets import QApplication
import Tessng
from TessngPlugin.MyPlugin import *


if __name__ == '__main__':
    app = QApplication()

    workspace = os.fspath(Path(__file__).resolve().parent)
    config = {'__workspace':workspace,
              '__netfilepath': r"F:\TJST_Project\TJST_VirtualReality_Fusion\MultiUser_TJST_request\TJST_Request_Vtessngauto\Data\YYT_TJST_0619_unlimited.tess",
              '__simuafterload': True,
              '__custsimubysteps': False
              }
    plugin = MyPlugin()
    factory = TessngFactory()
    tessng = factory.build(plugin, config)
    if tessng is None:
        sys.exit(0)
    else:
        sys.exit(app.exec_())

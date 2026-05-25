# -*- coding: utf-8 -*-
"""
Created on Fri Feb  2 13:01:45 2024

@author: HSINGHAO_SU
"""

from PySide2.QtWidgets import QApplication, QProgressDialog
from PySide2.QtCore import Qt, QCoreApplication


class ProgressBar(QProgressDialog):
    _instance = None
    _init = False
    def __new__(cls,):
        if ProgressBar._instance is None:
            ProgressBar._instance = super().__new__(cls)
        return ProgressBar._instance

    def __init__(self):
        if ProgressBar._init:
            return
        ProgressBar._init = True
        self.app = QApplication()
        super(ProgressBar, self).__init__()
        self.setWindowTitle('评价进度')
        self.setCancelButton(None)  # 禁用取消按钮
        self.setRange(0, 100+1)
        self.setValue(0)
        # self.setWindowFlag(Qt.WindowStaysOnTopHint)  # 设置窗口显示在最上面
        self.setFixedWidth(400)

    # 更新进度条
    def update_progress_bar(self, index, all_count, new_text=""):
        self.setLabelText(new_text)
        new_value = int(round(index / all_count * 100, 0))
        self.setValue(new_value)
        self.show()
        # 立刻更新界面
        QCoreApplication.processEvents()


# 进度条
def show_progress_bar(iterable_items, text=""):
    iterable_items_list = list(iterable_items)
    all_count = len(iterable_items_list)
    ProgressBar().setValue(0)
    for index, item in enumerate(iterable_items_list):
        yield item
        ProgressBar().update_progress_bar(index + 1, all_count, text)
    ProgressBar().hide()



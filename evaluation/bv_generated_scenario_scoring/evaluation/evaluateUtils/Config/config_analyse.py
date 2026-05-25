import json


class Analyse(object):
    def __init__(self, file):
        self.file = file
        self.data = None

    def open(self):
        file = open(self.file, encoding='UTF-8')
        data = json.load(file)
        self.data = data

    def info(self, index):
        self.open()
        return self.data[index]

import struct


class JausMessage:
    def __init__(self):
        self.output = ""

    def execute(self):
        pass

    def parse(self, data):
        data_fmt = self.__class__.data_fmt
        self.data = struct.unpack(data_fmt, data)

    def pack(self, data):
        data_fmt = self.__class__.data_fmt
        self.output = struct.pack(data_fmt, *data)

    def verify_query_message(self, query_message):
        return 4


from xmlrpclib import ServerProxy


class Robot:
    def __init__(self, ip, port):
        addr = "".join(["http://", ip, ":", port])
        self.proxy = ServerProxy(addr)

    def __getattr__(self, item):
        return getattr(self.proxy, item)
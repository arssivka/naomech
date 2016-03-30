from xmlrpclib import ServerProxy

from threading import Lock


class Robot:
    def __init__(self, ip, port):
        addr = "".join(["http://", ip, ":", port])
        self.proxy = ServerProxy(addr)
        self.mutex = Lock()

    def __getattr__(self, item):
        self.mutex.acquire()
        try:
            data = getattr(self.proxy, item)
        finally:
            self.mutex.release()
        return data

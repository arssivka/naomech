from xmlrpclib import ServerProxy

from threading import Lock


class _ThreadSafeMethod:
    def __init__(self, function, lock):
        self.function = function
        self.lock = lock

    def __getattr__(self, item):
        return _ThreadSafeMethod(getattr(self.function, item), self.lock)

    def __call__(self, *args, **kwargs):
        with self.lock:
            data = self.function(*args, **kwargs)
        return data


class Robot:
    def __init__(self, ip, port):
        addr = "".join(["http://", ip, ":", port])
        self.proxy = ServerProxy(addr)
        self.lock = Lock()

    def __getattr__(self, item):
        method = getattr(self.proxy, item)
        return _ThreadSafeMethod(method, self.lock)

import threading


def threadsafe(fn, lock=None):
    """decorator making sure that the decorated function is thread safe"""
    lock = lock or threading.Lock()

    def new(*args, **kwargs):
        lock.acquire()
        try:
            res = fn(*args, **kwargs)
        finally:
            lock.release()
        return res
    return new


class ThreadSafe(object):
    def __init__(self):
        super(ThreadSafe, self).__setattr__("_lock", threading.Lock())
        super(ThreadSafe, self).__setattr__("_thr_safe_func", dict())
        super(ThreadSafe, self).__setattr__("_thr_safe_vars", set())

    def is_thread_safe(self, name):
        return name in self._thr_safe_vars or name in self._thr_safe_func

    def set_thread_safe(self, name):
        if name in ("_lock", "_thr_safe_func", "_thr_safe_vars"):
            raise RuntimeError("Can't set thread-safe accessor for %s variable" % name)
        attr = getattr(self, name)
        if hasattr(attr, "__call__"):
            self._thr_safe_func[name] = threadsafe(attr, self._lock)
        else:
            self._thr_safe_vars.add(name)

    def __setattr__(self, key, value):
        if key in ("_lock", "_thr_safe_func", "_thr_safe_vars"):
            raise AttributeError()
        if key in self._thr_safe_vars or key in self._thr_safe_func:
            self._lock.acquire()
            try:
                super(ThreadSafe, self).__setattr__(key, value)
            finally:
                self._lock.release()
        else:
            super(ThreadSafe, self).__setattr__(key, value)

    def __getattribute__(self, item):
        thr_safe_func = super(ThreadSafe, self).__getattribute__("_thr_safe_func")
        res = thr_safe_func.get(item, None)
        if not res:
            thr_save_vars = super(ThreadSafe, self).__getattribute__("_thr_safe_vars")
            if item in thr_save_vars:
                lock = super(ThreadSafe, self).__getattribute__("_lock")
                lock.acquire()
                try:
                    res = super(ThreadSafe, self).__getattribute__(item)
                finally:
                    lock.release()
            else:
                res = super(ThreadSafe, self).__getattribute__(item)
        return res

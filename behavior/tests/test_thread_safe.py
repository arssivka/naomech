from unittest import TestCase

from behavior.threadsafe import ThreadSafe


class TestThreadSafe(TestCase):
    def _setter(self, value):
        self.assertEqual(value, "value")

    def test_unsetted(self):
        ts = ThreadSafe()
        with self.assertRaises(AttributeError):
            ts.set_thread_safe("unsetted")

    def test_var_not_thread_safe(self):
        ts = ThreadSafe()
        ts.var = "value"
        self.assertFalse(ts.is_thread_safe("var"))
        self.assertEqual(ts.var, "value")

    def test_var_thread_safe(self):
        ts = ThreadSafe()
        ts.var = "value"
        ts.set_thread_safe("var")
        self.assertTrue(ts.is_thread_safe("var"))
        self.assertEqual(ts.var, "value")

    def test_func_not_thread_safe(self):
        ts = ThreadSafe()
        ts.func = self._setter
        self.assertFalse(ts.is_thread_safe("func"))
        ts.func("value")

    def test_func_thread_safe(self):
        ts = ThreadSafe()
        ts.func = self._setter
        ts.set_thread_safe("func")
        self.assertTrue(ts.is_thread_safe("func"))
        ts.func("value")

    def test_set_system_vars(self):
        ts = ThreadSafe()
        with self.assertRaises(RuntimeError):
            ts.set_thread_safe("_lock")
        with self.assertRaises(RuntimeError):
            ts.set_thread_safe("_thr_safe_func")
        with self.assertRaises(RuntimeError):
            ts.set_thread_safe("_thr_safe_vars")

    def test_set_ts_all(self):
        ts = ThreadSafe()
        items = ("var1", "var2", "var3")
        for item in items:
            setattr(ts, item, item)
            self.assertFalse(ts.is_thread_safe(item))
        ts.set_thread_safe_all(items)
        for item in items:
            self.assertTrue(ts.is_thread_safe(item))
"""executor_main.freeze_startup_objects (docs/steering/EXECUTOR_PERF_KO.md): one full collection, then every object of the
started process leaves the cyclic collector's reach, so a later full collection walks only the objects made while
running; the event gc_freeze is logged. The loop's results do not depend on it (the fast SIM never calls main()).
"""
import gc
import unittest

import rowexec_testlib as L  # noqa: F401  (path setup)
from row_executor import config as C
from row_executor import executor_main as M


class GcFreezeTest(unittest.TestCase):
    def tearDown(self):
        gc.unfreeze()

    def test_freeze_and_event(self):
        log = M._NullLog()
        loop = M.ExecutorLoop(C.ExecutorConfig().validate(), log=log)
        held = [[k] for k in range(1000)]                # tracked containers that exist at the start
        count = M.freeze_startup_objects(loop)
        self.assertGreater(count, 1000)
        self.assertEqual(gc.get_freeze_count(), count)
        events = [r for r in log.records if r.get('event') == 'gc_freeze']
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['frozen'], count)
        self.assertEqual(len(events[0]['thresholds']), 3)
        self.assertEqual(len(held), 1000)
        gc.collect()                                      # a later full collection does not unfreeze them
        self.assertEqual(gc.get_freeze_count(), count)

    def test_main_freezes_before_the_loop(self):
        # executor_main.main and steering.exec_container_main call it right before the loop (source order)
        import inspect
        source = inspect.getsource(M.main)
        self.assertLess(source.index('freeze_startup_objects(loop)'), source.index('run_loop(loop, stop'))


if __name__ == '__main__':
    unittest.main()

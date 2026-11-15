"""A13 revision 30 (PC-12): the GNSS fix topic is allowed in the executor node file only; other /gps/* strings and the
same string in other decision files stay errors."""
import sys
import textwrap
import unittest

sys.dont_write_bytecode = True

from steering import truth_isolation as ti  # noqa: E402

SOURCE = '''
def subscribe(node, loop):
    node.create_subscription(object, '/gps/fix', loop.on_gnss_msg, 5)
'''


class GnssTopicTest(unittest.TestCase):
    def test_fix_topic_only_in_the_executor_node(self):
        errors, info = ti.check_source(textwrap.dedent(SOURCE), 'tools/row_executor/executor_main.py',
                                       ti.ROLE_DECISION)
        self.assertEqual(errors, [])
        self.assertEqual([i['rule'] for i in info], ['exempted:truth_topic'])
        errors, _ = ti.check_source(textwrap.dedent(SOURCE), 'tools/row_executor/core.py', ti.ROLE_DECISION)
        self.assertEqual([e['rule'] for e in errors], ['truth_topic'])
        errors, _ = ti.check_source(textwrap.dedent(SOURCE.replace('/gps/fix', '/gps/data')),
                                    'tools/row_executor/executor_main.py', ti.ROLE_DECISION)
        self.assertEqual([e['rule'] for e in errors], ['truth_topic'])

    def test_gnss_files_have_their_roles(self):
        self.assertEqual(ti.classify('sim/ubts/steering/gnss_sim.py')[0], ti.ROLE_PLANT)
        for rel in ('tools/row_executor/gnss_layer.py', 'tools/row_executor/gnss_rtk.py',
                    'tools/row_executor/local_grid.py',
                    'tools/row_executor/perception_layers.py', 'tools/row_executor/perception_settings.py'):
            self.assertEqual(ti.classify(rel)[0], ti.ROLE_DECISION, rel)
        self.assertEqual(ti.classify('sim/ubts/steering/safety_scripts/perception_compare.py')[0], ti.ROLE_EVALUATOR)


if __name__ == '__main__':
    unittest.main()

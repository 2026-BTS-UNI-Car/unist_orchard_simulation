"""Keep a ROS node in one executor across a stoppable callback loop."""
from contextlib import contextmanager
import os


class SimulationDiagnosticEvents:
    """Suppress only implicit QoS warning callbacks on fixed simulation topics.

    This is a node-local mixin, not a patch to rclpy. Live device nodes retain
    their defaults; explicitly supplied callbacks are always passed through.
    Message QoS, callbacks, status reporting and stale-data checks are unchanged.
    """
    def create_publisher(self, *args, **kwargs):
        if os.environ.get('DASHBOARD_MODE') == 'simulation' and 'event_callbacks' not in kwargs:
            from rclpy.event_handler import PublisherEventCallbacks
            kwargs['event_callbacks'] = PublisherEventCallbacks(use_default_callbacks=False)
        return super().create_publisher(*args, **kwargs)

    def create_subscription(self, *args, **kwargs):
        if os.environ.get('DASHBOARD_MODE') == 'simulation' and 'event_callbacks' not in kwargs:
            from rclpy.event_handler import SubscriptionEventCallbacks
            kwargs['event_callbacks'] = SubscriptionEventCallbacks(use_default_callbacks=False)
        return super().create_subscription(*args, **kwargs)


@contextmanager
def persistent_executor(node):
    # Import lazily so sensor codecs and offline tests do not require ROS.
    from rclpy.executors import SingleThreadedExecutor

    executor = SingleThreadedExecutor(context=node.context)
    try:
        executor.add_node(node)
        yield executor
    finally:
        executor.remove_node(node)
        executor.shutdown()

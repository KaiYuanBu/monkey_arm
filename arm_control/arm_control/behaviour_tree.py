import operator
import sys

import py_trees
import py_trees_ros.trees
import py_trees.console as console
import py_trees_ros_interfaces.action as py_trees_actions  # noqa
import rclpy

def create_root() -> py_trees.behaviour.Behaviour:
    """Create control behaviour for Arm.

    Returns:
        the root of the tree
    """
    root = py_trees.composites.Parallel(
        name="Tutorial Five",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )

    return root

def main():
    """
    Entry point for the demo script.
    """
    rclpy.init(args=None)
    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
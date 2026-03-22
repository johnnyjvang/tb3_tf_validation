"""
reset_results.py

Reset the tb3_tf_validation results file before running a test suite.
"""

import rclpy
from rclpy.node import Node

from tb3_tf_validation.result_utils import reset_results_file


class ResetResults(Node):
    def __init__(self):
        super().__init__('reset_results')

        reset_results_file()
        self.get_logger().info('Reset results file for tb3_tf_validation')


def main(args=None):
    rclpy.init(args=args)
    node = ResetResults()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
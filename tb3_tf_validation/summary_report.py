"""
summary_report.py

Read the shared TF validation CSV results file and print a clean summary table.
"""

import csv
import textwrap

import rclpy
from rclpy.node import Node

from tb3_tf_validation.result_utils import RESULTS_FILE


TEST_ORDER = [
    'tf_tree_check',
    'tf_static_check',
    'tf_dynamic_check',
    'tf_rate_check',
    'tf_delay_check',
    'tf_lookup_test',
    'tf_motion_consistency',
]

MAX_WIDTHS = [28, 10, 24, 60]  # Test, Status, Measurement, Notes


class SummaryReport(Node):
    def __init__(self):
        super().__init__('summary_report')
        self.print_summary()

    def print_summary(self):
        results = {}

        if RESULTS_FILE.exists():
            with open(RESULTS_FILE, 'r', newline='') as f:
                reader = csv.DictReader(f)

                if reader.fieldnames is None:
                    self.get_logger().warn('Results file is empty or missing a header row.')
                elif 'test' not in reader.fieldnames:
                    self.get_logger().warn(
                        f'Invalid CSV header in {RESULTS_FILE}. Found: {reader.fieldnames}'
                    )
                else:
                    for row in reader:
                        test_name = row.get('test', '').strip()
                        if test_name:
                            results[test_name] = row
        else:
            self.get_logger().warn(f'Results file not found: {RESULTS_FILE}')

        rows = []
        for test_name in TEST_ORDER:
            if test_name in results:
                row = results[test_name]
                rows.append([
                    row.get('test', test_name),
                    row.get('status', 'UNKNOWN'),
                    row.get('measurement', ''),
                    row.get('notes', '')
                ])
            else:
                rows.append([test_name, 'MISSING', '', 'no result found'])

        headers = ['Test', 'Status', 'Measurement', 'Notes']

        def wrap_cell(text, width):
            return textwrap.wrap(str(text), width=width) or ['']

        wrapped_headers = [
            wrap_cell(headers[i], MAX_WIDTHS[i]) for i in range(len(headers))
        ]
        max_header_lines = max(len(cell) for cell in wrapped_headers)
        for cell in wrapped_headers:
            while len(cell) < max_header_lines:
                cell.append('')

        wrapped_rows = []
        for row in rows:
            wrapped = [
                wrap_cell(row[i], MAX_WIDTHS[i]) for i in range(len(row))
            ]
            max_lines = max(len(cell) for cell in wrapped)
            for cell in wrapped:
                while len(cell) < max_lines:
                    cell.append('')
            wrapped_rows.append(wrapped)

        def format_wrapped_row(wrapped_row):
            lines = []
            num_lines = len(wrapped_row[0])

            for line_idx in range(num_lines):
                line = '| ' + ' | '.join(
                    wrapped_row[col_idx][line_idx].ljust(MAX_WIDTHS[col_idx])
                    for col_idx in range(len(wrapped_row))
                ) + ' |'
                lines.append(line)

            return '\n'.join(lines)

        border = '+-' + '-+-'.join('-' * w for w in MAX_WIDTHS) + '-+'

        print()
        print('========================================')
        print('TurtleBot3 TF Validation Summary')
        print('========================================')
        print(border)
        print(format_wrapped_row(wrapped_headers))
        print(border)

        for wrapped_row in wrapped_rows:
            print(format_wrapped_row(wrapped_row))
            print(border)

        print()


def main(args=None):
    rclpy.init(args=args)
    node = SummaryReport()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
"""
result_utils.py

Utility functions for managing the TF validation results CSV file.

Goal:
- Create/reset the results file
- Append standardized test rows
- Keep all package test outputs consistent
"""

import csv
from pathlib import Path


RESULTS_DIR = Path('/tmp/tb3_tf_validation')
RESULTS_FILE = RESULTS_DIR / 'results.csv'


def reset_results_file():
    """
    Reset the results CSV file and write the header row.
    """
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    with open(RESULTS_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['test', 'status', 'measurement', 'notes'])


def append_result(test_name, status, measurement, notes=''):
    """
    Append one standardized test result row to the CSV file.
    """
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    with open(RESULTS_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([test_name, status, measurement, notes])
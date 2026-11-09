"""Append task records; dry-run results are always labeled explicitly."""
import csv
from pathlib import Path

FIELDS = ('task_id', 'execution_mode', 'start_time', 'end_time', 'total_duration_s',
          'quantity', 'completed_quantity', 'task_success', 'final_error_code',
          'final_error_message')


class ExperimentLogger:
    def __init__(self, output_path):
        self.path = Path(output_path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def record(self, **values):
        header = not self.path.exists() or self.path.stat().st_size == 0
        with self.path.open('a', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            if header:
                writer.writeheader()
            writer.writerow(values)

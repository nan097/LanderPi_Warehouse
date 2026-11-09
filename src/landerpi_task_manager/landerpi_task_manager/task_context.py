from dataclasses import dataclass, field
from time import monotonic


@dataclass
class TaskContext:
    task_id: str
    quantity: int
    completed_quantity: int = 0
    start_time: float = field(default_factory=monotonic)
    error_code: int = 0
    error_message: str = ''

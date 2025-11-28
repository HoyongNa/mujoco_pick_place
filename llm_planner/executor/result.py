
from dataclasses import dataclass
from typing import Optional, Dict
from .status import ExecutionStatus

@dataclass
class ExecutionResult:
    """작업 실행 결과"""
    task: any  # llm_planner.SubTask 타입 (런타임 의존)
    status: ExecutionStatus
    start_time: float
    end_time: float
    error_message: Optional[str] = None
    data: Optional[Dict] = None

    @property
    def duration(self) -> float:
        return self.end_time - self.start_time

    def to_dict(self):
        return {
            'task': self.task.to_dict(),
            'status': self.status.value,
            'duration': self.duration,
            'error_message': self.error_message,
            'data': self.data
        }

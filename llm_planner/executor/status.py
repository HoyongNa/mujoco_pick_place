
from enum import Enum

class ExecutionStatus(Enum):
    """작업 실행 상태"""
    PENDING = "pending"
    RUNNING = "running"
    SUCCESS = "success"
    FAILED = "failed"
    CANCELLED = "cancelled"

"""
작업 유형과 데이터 모델 정의 (단순화 버전)
"""
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import List, Dict


class TaskType(Enum):
    NAVIGATE = "navigate"
    PICK     = "pick"
    PLACE    = "place"


@dataclass
class SubTask:
    task_type: "TaskType"
    parameters: Dict
    description: str = ""
    priority: int = 0
    estimated_duration: float = 5.0
    dependencies: List[int] = field(default_factory=list)

    def to_dict(self) -> Dict:
        d = asdict(self)
        d["task_type"] = self.task_type.value
        return d


@dataclass
class TaskPlan:
    original_command: str
    subtasks: List[SubTask]
    total_estimated_time: float
    metadata: Dict = field(default_factory=dict)

    def to_dict(self) -> Dict:
        return {
            "original_command": self.original_command,
            "subtasks": [t.to_dict() for t in self.subtasks],
            "total_estimated_time": self.total_estimated_time,
            "metadata": self.metadata,
        }

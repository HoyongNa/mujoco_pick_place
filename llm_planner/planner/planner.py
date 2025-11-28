"""
LLM 기반 자연어 → 작업 계획 변환 플래너 (MapProcessor 제거 버전)
- scene.xml이 있으면 로드, 없으면 생략
- OpenAI 호출 실패는 그대로 예외 발생 (은닉하지 않음)
- map_file 인자는 API 호환성 위해 받지만 사용하지 않음
"""
import os
import json
import logging
from typing import Dict, List, Tuple, Optional

import numpy as np
from .task_types import TaskType, SubTask, TaskPlan
from .scene_parser import SceneParser

logger = logging.getLogger(__name__)

ROOM_LOCATIONS = {
    "방1": {"position": [-2.5, 2.5], "zone": "northwest"},
    "방2": {"position": [2.5, 2.5], "zone": "northeast"},
    "방3": {"position": [-2.5, -2.5], "zone": "southwest"},
    "방4": {"position": [2.5, -2.5], "zone": "southeast"},
    "중앙": {"position": [0.0, 0.0], "zone": "center"},
    "복도": {"position": [0.0, 1.0], "zone": "hallway"},
}

class LLMPlanner:
    def __init__(
        self,
        api_key: Optional[str] = None,
        model: str = "gpt-4o",
        scene_xml: Optional[str] = None,
    ):
        self.api_key = api_key or os.environ.get("OPENAI_API_KEY")
        self.model = model
        self.use_llm = bool(self.api_key)

        # scene.xml → OBJECT_LOCATIONS (있으면 로드)
        self.OBJECT_LOCATIONS: Dict[str, dict] = {}
        self._load_scene(scene_xml)

        # 시스템 프롬프트
        self.system_prompt = self._create_system_prompt()

    # ---------- 내부 유틸 ----------

    def _load_scene(self, scene_xml: Optional[str]):
        if scene_xml and os.path.exists(scene_xml):
            parsed = SceneParser.parse_scene_xml(scene_xml)
            self.OBJECT_LOCATIONS = {v.get("display_name", k): v for k, v in parsed.items()}
            return

        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        for path in (
            os.path.join(base_dir, "model", "stanford_tidybot", "scene.xml"),
            os.path.join(base_dir, "scene.xml"),
        ):
            if os.path.exists(path):
                parsed = SceneParser.parse_scene_xml(scene_xml)
                self.OBJECT_LOCATIONS = {v.get("display_name", k): v for k, v in parsed.items()}
                return
        # 없으면 비움

    def _create_system_prompt(self) -> str:
        # (선택) scene.xml에서 로드한 박스 목록을 한 줄 요약으로 제공
        box_lines = []
        if self.OBJECT_LOCATIONS:
            for name, info in sorted(self.OBJECT_LOCATIONS.items()):
                x, y = float(info["position"][0]), float(info["position"][1])
                box_lines.append(f"- {name}: [{x:.2f}, {y:.2f}]")
        boxes_section = (
            "## Known boxes (from scene.xml)\n" + "\n".join(box_lines) + "\n"
            if box_lines else "## Known boxes\n(none loaded)\n"
        )

        return f"""
Convert robot commands to JSON. Output ONLY a single JSON object (no markdown, no comments).

{boxes_section}

## 🚨 IMPORTANT - Navigation includes automatic 3-second wait
**NAVIGATE automatically includes a 3-second wait period after reaching the target**
**This ensures the robot is completely stopped before subsequent operations**

✅ CORRECT pattern for pick-and-place:
Step 1: NAVIGATE (includes 3s wait) → Step 2: PICK → Step 3: NAVIGATE (includes 3s wait) → Step 4: PLACE

❌ WRONG (adding unnecessary WAIT after NAVIGATE):
NAVIGATE → WAIT(3.0) → PICK  ← NO! NAVIGATE already includes the wait!

**DO NOT add WAIT tasks after NAVIGATE - the wait is handled automatically**

## Task Types (UPPERCASE only)
- NAVIGATE: {{"target_position": [x, y], "robot_id": "robot1"|"robot2"}}  # Includes automatic 3s wait
- WAIT: {{"duration": <seconds>, "robot_id": "robot1"|"robot2"}}  # OPTIONAL - only if extra delay needed
- PICK: {{"object_name": "box_name", "robot_id": "robot1"|"robot2"}}
- PLACE: {{"target_position": "box_name"|[x,y], "robot_id": "robot1"|"robot2"}}

## Multi-Robot Support
- Parse from command: "Robot1"/"로봇1" → "robot1", "Robot2"/"로봇2" → "robot2"
- Default: "robot1"
- Each task MUST have "robot_id"

## Other Rules
1. Coordinates: float with 2 decimals
2. PLACE on box name: use exact box name (no extra NAVIGATE)
3. PLACE at [x,y]: if distance > 0.70m from last NAVIGATE, add NAVIGATE first
4. Gripper reach: 0.70m from last NAVIGATE position
5. NAVIGATE automatically waits 3s after reaching target - no need to add WAIT

## Output Schema
{{
  "subtasks": [
    {{
      "task_type": "NAVIGATE"|"WAIT"|"PICK"|"PLACE",
      "parameters": {{...}},  // must include robot_id
      "description": "brief text",
      "priority": <int>,
      "estimated_duration": <float>
    }}
  ],
  "metadata": {{
    "difficulty": "easy"|"medium"|"hard",
    "estimated_total_time": <float>,
    "robot_id": "robot1"|"robot2"
  }}
}}

## Examples
- "Pick blue, place on purple":
  NAV(to blue, auto-wait 3s) → PICK(blue) → NAV(to purple, auto-wait 3s) → PLACE(on purple)
  
- "Robot2 pick green":
  NAV(robot2, auto-wait 3s) → PICK(green, robot2)

- "Pick red, wait 5s, place on table":
  NAV(to red, auto-wait 3s) → PICK(red) → WAIT(5.0) → NAV(to table, auto-wait 3s) → PLACE(on table)

Remember: NAVIGATE includes automatic 3s wait - only add explicit WAIT for special timing needs!
""".strip()

    def _safe_float(self, v) -> float:
        return float(v)

    def _pos_to_float(self, pos):
        if pos is None or isinstance(pos, str):
            return pos
        if isinstance(pos, (list, tuple)) and len(pos) >= 2:
            return [float(pos[0]), float(pos[1])]
        return None
    
    def _extract_robot_id_from_command(self, command: str) -> str:
        """
        명령에서 robot_id 추출 (natural language understanding)
        
        Examples:
            "Robot1, go to room 1" → "robot1"
            "Robot2, pick the box" → "robot2"
            "로봇1, 방1로 가" → "robot1"
            "로봇2, 파란 박스 집어" → "robot2"
            "Go to room 1" → "robot1" (default)
        """
        command_lower = command.lower()
        
        # Check for English patterns
        if "robot2" in command_lower or "robot 2" in command_lower:
            return "robot2"
        if "robot1" in command_lower or "robot 1" in command_lower:
            return "robot1"
        
        # Check for Korean patterns
        if "로봇2" in command or "로봇 2" in command:
            return "robot2"
        if "로봇1" in command or "로봇 1" in command:
            return "robot1"
        
        # Default to robot1
        return "robot1"

    # ---------- 퍼블릭 API ----------

    def parse_command(self, command: str, current_position: Tuple[float, float] = (0, 0), robot_id: int = 1) -> TaskPlan:
        """✅ Parse command with robot_id support
        
        Args:
            command: User command
            current_position: Current robot position
            robot_id: Which robot (1 or 2) - converted to "robot1" or "robot2"
        """
        if not self.use_llm:
            raise RuntimeError("LLM 플래너가 비활성화되었습니다. OpenAI API 키가 필요합니다.")
        
        # ✅ Convert robot_id to string format
        robot_id_str = f"robot{robot_id}"
        
        # ✅ Extract robot_id from command, but prefer explicit parameter
        default_robot_id = self._extract_robot_id_from_command(command)
        # If robot_id was explicitly passed, use it (overrides command parsing)
        if robot_id != 1:  # User explicitly specified robot
            default_robot_id = robot_id_str
        logger.info(f"✅ Using robot_id: {default_robot_id} (parameter: {robot_id}, detected: {self._extract_robot_id_from_command(command)})")

        # 컨텍스트(언급된 박스와 접근 좌표) 구성
        mentioned = []
        for obj_name, info in self.OBJECT_LOCATIONS.items():
            keywords = [obj_name] + obj_name.split() + info.get("aliases", [])
            if any(kw in command for kw in keywords):
                mentioned.append((obj_name, info))

        context_lines = []
        if mentioned:
            context_lines.append("🎯 명령에서 언급된 박스들:")
            for obj_name, info in mentioned:
                pos = info["position"]; room = info.get("room", "중앙")
                context_lines.append(f"- {obj_name} @ ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.3f}) / {room}")

                same_room = [(n, i) for n, i in self.OBJECT_LOCATIONS.items() if i.get("room") == room and n != obj_name]
                approach = [
                    (pos[0]-0.5, pos[1], "서쪽"),
                    (pos[0]+0.5, pos[1], "동쪽"),
                    (pos[0], pos[1]-0.5, "남쪽"),
                    (pos[0], pos[1]+0.5, "북쪽"),
                ]
                for x, y, dir_name in approach:
                    min_d = min((float(np.hypot(i["position"][0]-x, i["position"][1]-y)) for _, i in same_room), default=999.0)
                    mark = "✅" if min_d >= 0.3 else "❌"
                    context_lines.append(f"  · {dir_name}: [{x:.2f}, {y:.2f}] min_dist={min_d:.2f}m {mark}")

        enhanced = (
            f"{command}\n\n"
            f"📍 현재 로봇 위치: ({current_position[0]:.2f}, {current_position[1]:.2f})\n"
            + ("\n".join(context_lines) + "\n" if context_lines else "")
            + "⚠️ 지침: 1) PICK Navigate는 안전 접근 좌표 사용  2) PLACE는 박스 이름 권장  3) 멀면 Navigate 후 PLACE  4) NAVIGATE는 자동으로 3초 대기 포함 (WAIT 불필요)\n"
            + f"🤖 기본 robot_id: {default_robot_id} (명령에서 감지됨)"
        )
        return self._parse_with_llm(enhanced, default_robot_id)

    def _parse_with_llm(self, command: str, default_robot_id: str = "robot1") -> TaskPlan:
        """
        ✅ LLM을 사용하여 명령을 TaskPlan으로 변환 (multi-robot support)
        
        Args:
            command: 사용자 명령
            default_robot_id: 명령에서 감지된 기본 robot_id
        """
        from openai import OpenAI  # 실패 시 그대로 ImportError 발생
        client = OpenAI(api_key=self.api_key)

        logger.info("LLM 요청 시작")
        resp = client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": f"다음 명령을 작업 계획으로 변환하세요: {command}"},
            ],
            temperature=0.1,
            max_tokens=1500,
        )
        content = resp.choices[0].message.content

        # ```json ... ``` 블록 우선 추출
        start, end = content.find("```json"), content.rfind("```")
        if 0 <= start < end:
            content = content[start + 7 : end].strip()

        result = json.loads(content)  # 형식 오류 시 그대로 예외

        # SubTask 구성
        subtasks: List[SubTask] = []
        last_nav_pos = None
        for t in result.get("subtasks", []):
            ttype = TaskType[t["task_type"]]
            params = (t.get("parameters") or {}).copy()
            
            # ✅ Ensure robot_id is present (use default if not specified by LLM)
            if "robot_id" not in params:
                params["robot_id"] = default_robot_id
                logger.debug(f"Added default robot_id '{default_robot_id}' to {ttype.value} task")

            if "target_position" in params:
                params["target_position"] = self._pos_to_float(params["target_position"])

            subtasks.append(SubTask(
                task_type=ttype,
                parameters=params,
                description=t.get("description", ""),
                priority=t.get("priority", 0),
                estimated_duration=t.get("estimated_duration", 5.0),
            ))

            if ttype == TaskType.NAVIGATE and isinstance(params.get("target_position"), (list, tuple)):
                last_nav_pos = params["target_position"]

        metadata = result.get("metadata", {})
        return TaskPlan(
            original_command=command.splitlines()[0],
            subtasks=subtasks,
            total_estimated_time=sum(st.estimated_duration for st in subtasks),
            metadata=metadata,
        )

    # ---------- 계획 분석 ----------

    def get_robot_ids_in_plan(self, plan: TaskPlan) -> set:
        """
        Extract all robot IDs present in the plan
        
        Args:
            plan: TaskPlan to analyze
            
        Returns:
            set: Set of robot_id strings (e.g., {"robot1", "robot2"})
        """
        robot_ids = set()
        for task in plan.subtasks:
            robot_id = task.parameters.get("robot_id", "robot1")
            robot_ids.add(robot_id)
        return robot_ids

    def is_multi_robot_plan(self, plan: TaskPlan) -> bool:
        """
        Check if plan contains tasks for multiple robots
        
        Args:
            plan: TaskPlan to check
            
        Returns:
            bool: True if plan has tasks for more than one robot
        """
        return len(self.get_robot_ids_in_plan(plan)) > 1

    def split_plan_by_robot(self, plan: TaskPlan) -> Dict[str, TaskPlan]:
        """
        Split a mixed-robot plan into separate plans for each robot
        
        Args:
            plan: TaskPlan that may contain tasks for multiple robots
            
        Returns:
            dict: {robot_id: TaskPlan} - separate plans for each robot
            
        Example:
            >>> mixed_plan = planner.parse_command("Robot1 pick blue, Robot2 pick red")
            >>> split_plans = planner.split_plan_by_robot(mixed_plan)
            >>> # Returns: {"robot1": TaskPlan(...), "robot2": TaskPlan(...)}
        """
        # Group tasks by robot_id
        robot_tasks = {"robot1": [], "robot2": []}
        
        for task in plan.subtasks:
            task_robot_id = task.parameters.get("robot_id", "robot1")
            if task_robot_id in robot_tasks:
                robot_tasks[task_robot_id].append(task)
        
        # Create separate TaskPlan for each robot
        split_plans = {}
        
        for robot_id_str, tasks in robot_tasks.items():
            if not tasks:  # Skip if no tasks for this robot
                continue
            
            robot_plan = TaskPlan(
                original_command=f"{plan.original_command} [{robot_id_str.upper()}]",
                subtasks=tasks,
                total_estimated_time=sum(t.estimated_duration for t in tasks),
                metadata=plan.metadata.copy()
            )
            split_plans[robot_id_str] = robot_plan
        
        return split_plans

    # ---------- 설명 (디버깅용) ----------

    def explain_plan(self, plan: TaskPlan) -> str:
        """작업 계획 설명 (multi-robot support)"""
        lines = [f"명령: {plan.original_command}", f"총 {len(plan.subtasks)}개 작업 (예상 시간: {plan.total_estimated_time:.1f}초)", ""]
        for i, t in enumerate(plan.subtasks, 1):
            # Show robot_id in description
            robot_id = t.parameters.get("robot_id", "robot1")
            lines.append(f"{i}. [{robot_id.upper()}] {t.description}")
            lines.append(f"   - 유형: {t.task_type.value}")
            lines.append(f"   - 로봇: {robot_id}")
            lines.append(f"   - 예상 시간: {t.estimated_duration:.1f}초")
            if t.task_type == TaskType.NAVIGATE:
                pos = t.parameters.get("target_position", [0, 0])
                lines.append(f"   - 목표 위치: ({pos[0]:.1f}, {pos[1]:.1f})")
                lines.append(f"   - 특이사항: 도착 후 3초 자동 대기 포함")
            elif t.task_type == TaskType.PICK:
                obj = t.parameters.get("object_name", "알 수 없음")
                lines.append(f"   - 대상 물체: {obj}")
            lines.append("")
        return "\n".join(lines)
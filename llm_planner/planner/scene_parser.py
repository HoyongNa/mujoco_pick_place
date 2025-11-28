"""
scene.xml 파일에서 박스 정보를 파싱하는 모듈 (단순화 버전)
- try/except 제거(은닉 X): XML 파싱 실패는 그대로 예외
- 가드 continue로 불필요한 분기 최소화
"""
import logging
import xml.etree.ElementTree as ET
from typing import Dict, Tuple

logger = logging.getLogger(__name__)


class SceneParser:
    """scene.xml 파싱을 담당하는 클래스"""
    COLOR_MAP: Dict[Tuple[float, float, float], Tuple[str, str]] = {
        (1.0, 0.0, 0.0): ("red", "빨간색"),
        (0.0, 0.0, 1.0): ("blue", "파란색"),
        (0.0, 1.0, 0.0): ("green", "초록색"),
        (1.0, 1.0, 0.0): ("yellow", "노란색"),
        (1.0, 0.5, 0.0): ("orange", "주황색"),
        (0.5, 0.0, 0.5): ("purple", "보라색"),
        (0.0, 1.0, 1.0): ("cyan", "청록색"),
        (1.0, 0.5, 0.8): ("pink", "분홍색"),
    }

    @staticmethod
    def get_room_from_position(x: float, y: float) -> str:
        if x < 0 and y > 0:  return "방1"
        if x > 0 and y > 0:  return "방2"
        if x < 0 and y < 0:  return "방3"
        if x > 0 and y < 0:  return "방4"
        return "중앙"

    @classmethod
    def parse_scene_xml(cls, xml_path: str) -> Dict:
        root = ET.parse(xml_path).getroot()
        objects: Dict[str, dict] = {}

        for body in root.iter("body"):
            name = body.get("name")
            if not name or "_box" not in name:
                continue

            pos_str = body.get("pos")
            if not pos_str:
                continue
            pos = [float(v) for v in pos_str.split()]

            geom = body.find("geom")
            if geom is None:
                continue

            rgba_str = geom.get("rgba")
            if rgba_str:
                rgb = tuple(float(x) for x in rgba_str.split()[:3])
                # 가장 가까운 색상 매칭
                color_en, color_kr = min(
                    cls.COLOR_MAP.items(),
                    key=lambda kv: (rgb[0] - kv[0][0]) ** 2 + (rgb[1] - kv[0][1]) ** 2 + (rgb[2] - kv[0][2]) ** 2,
                )[1]
            else:
                color_en, color_kr = "unknown", "알 수 없음"

            room = cls.get_room_from_position(pos[0], pos[1])
            display_name = f"{color_kr} 박스" if color_kr != "알 수 없음" else name

            aliases = []
            if color_kr != "알 수 없음":
                aliases.append(f"{color_kr.replace('색', '')} 박스")
            if color_en != "unknown":
                aliases.append(f"{color_en} box")
            aliases.append(name)

            objects[name] = {
                "position": pos,
                "color": color_en,
                "color_kr": color_kr,
                "display_name": display_name,
                "type": "box",
                "room": room,
                "xml_name": name,
                "aliases": aliases,
            }
            logger.info("✅ XML 파싱: %s at %s in %s", display_name, pos, room)

        return objects

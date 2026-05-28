"""由人工记录点生成简化 forbidden zone。

第一版只支持简单几何体：
- cylinder：以 so101_base 的 (0,0) 为中心，半径由记录点最大 xy 半径外扩得到；
- aabb：记录点 xyz 轴对齐包围盒，按 safety_margin 外扩。

这个禁入区只检查末端点，不做全连杆碰撞检测。
"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import math


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INPUT = REPO_ROOT / "config" / "forbidden_zone_points.json"
DEFAULT_OUTPUT = REPO_ROOT / "config" / "forbidden_zone.json"


def resolve_path(path):
    """把相对路径解析到项目根目录下。"""
    result = Path(path)
    if not result.is_absolute():
        result = REPO_ROOT / result
    return result


def now_iso():
    """生成本地时间戳。"""
    return datetime.now().astimezone().isoformat(timespec="seconds")


def load_points(path):
    """读取人工记录点。"""
    data = json.loads(path.read_text(encoding="utf-8"))
    samples = data.get("samples", [])
    if not samples:
        raise SystemExit(f"没有可用记录点: {path}")
    points = []
    for sample in samples:
        pos = sample["end_effector_position"]
        points.append((float(pos["x"]), float(pos["y"]), float(pos["z"])))
    return data, points


def build_cylinder(points, safety_margin, z_margin, source_file):
    """生成以 base 原点为中心的圆柱禁区。"""
    radius = max(math.hypot(x, y) for x, y, _ in points) + safety_margin
    z_values = [z for _, _, z in points]
    return {
        "type": "cylinder",
        "frame": "so101_base",
        "center": [0.0, 0.0],
        "radius": round(radius, 6),
        "z_min": round(min(z_values) - z_margin, 6),
        "z_max": round(max(z_values) + z_margin, 6),
        "safety_margin": safety_margin,
        "z_margin": z_margin,
        "source": "manual_recorded_boundary",
        "source_file": str(source_file),
        "created_at": now_iso(),
        "sample_count": len(points),
        "note": "简化圆柱禁入区，只检查末端点；不等价于完整碰撞检测。",
    }


def build_aabb(points, safety_margin, z_margin, source_file):
    """生成轴对齐包围盒禁区。"""
    xs = [x for x, _, _ in points]
    ys = [y for _, y, _ in points]
    zs = [z for _, _, z in points]
    return {
        "type": "aabb",
        "frame": "so101_base",
        "x_min": round(min(xs) - safety_margin, 6),
        "x_max": round(max(xs) + safety_margin, 6),
        "y_min": round(min(ys) - safety_margin, 6),
        "y_max": round(max(ys) + safety_margin, 6),
        "z_min": round(min(zs) - z_margin, 6),
        "z_max": round(max(zs) + z_margin, 6),
        "safety_margin": safety_margin,
        "z_margin": z_margin,
        "source": "manual_recorded_boundary",
        "source_file": str(source_file),
        "created_at": now_iso(),
        "sample_count": len(points),
        "note": "简化 AABB 禁入区，只检查末端点；不等价于完整碰撞检测。",
    }


def main():
    parser = argparse.ArgumentParser(description="由人工记录点生成 forbidden_zone.json。")
    parser.add_argument("--input", default=str(DEFAULT_INPUT), help="人工记录点 JSON。")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT), help="禁入区输出 JSON。")
    parser.add_argument("--type", choices=["cylinder", "aabb"], default="cylinder")
    parser.add_argument("--safety-margin", type=float, default=0.02, help="xy 外扩安全距离，单位米。")
    parser.add_argument("--z-margin", type=float, help="z 外扩安全距离，单位米；默认等于 safety-margin。")
    args = parser.parse_args()

    input_path = resolve_path(args.input)
    output_path = resolve_path(args.output)
    _, points = load_points(input_path)
    z_margin = args.safety_margin if args.z_margin is None else args.z_margin

    if args.type == "cylinder":
        zone = build_cylinder(points, args.safety_margin, z_margin, input_path)
    else:
        zone = build_aabb(points, args.safety_margin, z_margin, input_path)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(zone, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"已生成禁入区: {output_path}")
    print(json.dumps(zone, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
